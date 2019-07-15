#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"

#include <sdf/Element.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Distortion.hh>
#include <ignition/math/Helpers.hh>

#include <camera_info_manager/camera_info_manager.h>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <std_msgs/msg/empty.hpp>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/logging.hpp>
namespace
{
    std::string extractCameraName(const std::string& name);
    sensor_msgs::msg::CameraInfo cameraInfo(const sensor_msgs::msg::Image& image, float horizontal_fov);
}

namespace gazebo
{

    class GazeboRealsensePrivate
    {
        public:
            /// A pointer to the GazeboROS node.
            gazebo_ros::Node::SharedPtr rosnode_{nullptr};

            /// \brief ROS image messages
            sensor_msgs::msg::Image image_msg, depth_msg;

            image_transport::CameraPublisher color_pub_, ir1_pub_, ir2_pub_, depth_pub_;

            /// Camera info publisher.
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_{nullptr};

            /// Trigger subscriber, in case it's a triggered camera
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_{nullptr};

            /// Camera info manager
            std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

            /// Image encoding
            std::string type_;

            /// Frame name, to be used by TF.
            std::string frame_name_;

            /// Step size
            int skip_;

            /// Connects to pre-render events.
            gazebo::event::ConnectionPtr pre_render_connection_;

            /// Keeps track of how many times the camera has been triggered since it last published an image.
            int triggered{0};

            /// Protects trigger.
            std::mutex trigger_mutex_;


    };
    // Register the plugin
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

        GazeboRosRealsense::GazeboRosRealsense()
        : impl_(std::make_unique<GazeboRealsensePrivate>())
        {
        }

    GazeboRosRealsense::~GazeboRosRealsense()
    {
        RCLCPP_INFO(impl_->rosnode_->get_logger(), "realsense Unloaded");
    }

    void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Make sure the ROS node for Gazebo has already been initialized
        // TODO [jc]: what is the ros2 check for this guy 
        /*
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        */

        RealSensePlugin::Load(_model, _sdf);

        this->impl_->rosnode_ = gazebo_ros::Node::Get(_sdf);

        RCLCPP_INFO(impl_->rosnode_->get_logger(), "Realsense Gazebo ROS plugin loading");

        // initialize camera_info_manager
        // Initialize camera_info_manager
        impl_->camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
                impl_->rosnode_.get(), this->GetHandle());


        this->impl_->color_pub_ = image_transport::create_camera_publisher(impl_->rosnode_.get(), "camera/color/image_raw");
        this->impl_->ir1_pub_ =   image_transport::create_camera_publisher(impl_->rosnode_.get(),"camera/ir/image_raw"    );
        this->impl_->ir2_pub_ =   image_transport::create_camera_publisher(impl_->rosnode_.get(),"camera/ir2/image_raw"   );
        this->impl_->depth_pub_ = image_transport::create_camera_publisher(impl_->rosnode_.get(),"camera/depth/image_raw" );
    }

    void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
            const transport::PublisherPtr pub)
    {
        common::Time current_time = this->world->SimTime();

        // identify camera
        std::string camera_id = extractCameraName(cam->Name());
        const std::map<std::string, image_transport::CameraPublisher*> camera_publishers = {
            {COLOR_CAMERA_NAME, &(this->impl_->color_pub_)},
            {IRED1_CAMERA_NAME, &(this->impl_->ir1_pub_)},
            {IRED2_CAMERA_NAME, &(this->impl_->ir2_pub_)},
        };
        const auto image_pub = camera_publishers.at(camera_id);

        // copy data into image
        this->impl_->image_msg.header.frame_id = camera_id;
        this->impl_->image_msg.header.stamp.sec = current_time.sec;
        this->impl_->image_msg.header.stamp.nanosec = current_time.nsec;

        // set image encoding
        const std::map<std::string, std::string> supported_image_encodings = {
            {"L_INT8", sensor_msgs::image_encodings::MONO8},
            {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
        };
        const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

        // copy from simulation image to ROS msg
        sensor_msgs::fillImage(this->impl_->image_msg,
                pixel_format,
                cam->ImageHeight(), cam->ImageWidth(),
                cam->ImageDepth() * cam->ImageWidth(),
                reinterpret_cast<const void*>(cam->ImageData()));

        // identify camera rendering
        const std::map<std::string, rendering::CameraPtr> cameras = {
            {COLOR_CAMERA_NAME, this->colorCam},
            {IRED1_CAMERA_NAME, this->ired1Cam},
            {IRED2_CAMERA_NAME, this->ired2Cam},
        };

        // publish to ROS
        auto camera_info_msg = cameraInfo(this->impl_->image_msg, cameras.at(camera_id)->HFOV().Radian());
        image_pub->publish(this->impl_->image_msg, camera_info_msg);
    }

    void GazeboRosRealsense::OnNewDepthFrame()
    {
        // get current time
        common::Time current_time = this->world->SimTime();

        RealSensePlugin::OnNewDepthFrame();

        // copy data into image
        this->impl_->depth_msg.header.frame_id = COLOR_CAMERA_NAME;
        this->impl_->depth_msg.header.stamp.sec = current_time.sec;
        this->impl_->depth_msg.header.stamp.nanosec = current_time.nsec;

        // set image encoding
        std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

        // copy from simulation image to ROS msg
        sensor_msgs::fillImage(this->impl_->depth_msg,
                pixel_format,
                this->depthCam->ImageHeight(), this->depthCam->ImageWidth(),
                2 * this->depthCam->ImageWidth(),
                reinterpret_cast<const void*>(this->depthMap.data()));

        // publish to ROS
        auto depth_info_msg = cameraInfo(this->impl_->depth_msg, this->depthCam->HFOV().Radian());
        this->impl_->depth_pub_.publish(this->impl_->depth_msg, depth_info_msg);
    }

}

namespace
{
    std::string extractCameraName(const std::string& name)
    {
        if (name.find(COLOR_CAMERA_NAME) != std::string::npos) return COLOR_CAMERA_NAME;
        if (name.find(IRED1_CAMERA_NAME) != std::string::npos) return IRED1_CAMERA_NAME;
        if (name.find(IRED2_CAMERA_NAME) != std::string::npos) return IRED2_CAMERA_NAME;

        //jc [TODO] - how to throw an error without a node
        //RCLCPP_ERROR(impl_->rosnode_->get_logger(), "Unknown camera name");
        return COLOR_CAMERA_NAME;
    }

    sensor_msgs::msg::CameraInfo cameraInfo(const sensor_msgs::msg::Image& image, float horizontal_fov)
    {
        sensor_msgs::msg::CameraInfo info_msg;

        info_msg.header = image.header;
        info_msg.height = image.height;
        info_msg.width = image.width;

        float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

        info_msg.k[0] = focal;
        info_msg.k[4] = focal;
        info_msg.k[2] = info_msg.width * 0.5;
        info_msg.k[5] = info_msg.height * 0.5;
        info_msg.k[8] = 1.;

        info_msg.p[0] = info_msg.k[0];
        info_msg.p[5] = info_msg.k[4];
        info_msg.p[2] = info_msg.k[2];
        info_msg.p[6] = info_msg.k[5];
        info_msg.p[10] = info_msg.k[8];

        return info_msg;
    }
}

