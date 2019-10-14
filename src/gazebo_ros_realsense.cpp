#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"
#include "realsense_gazebo_plugin/depth_conversions.hpp"

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
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <std_msgs/msg/empty.hpp>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/logging.hpp>
namespace
{
    static const auto depth_frame_name = "camera_depth_optical_frame";
    static const auto color_frame_name = "camera_color_optical_frame";
    static const auto ired1_frame_name = "camera_right_ir_optical_frame";
    static const auto ired2_frame_name = "camera_left_ir_optical_frame";

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

            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_;

            /// Trigger subscriber, in case it's a triggered camera
            rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_{nullptr};


            /// Camera info manager
            std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

            /// Image encoding
            std::string type_;

            /// Frame name, to be used by TF.
            std::string frame_name_;

            image_geometry::PinholeCameraModel model_;

            /// Step size
            int skip_;

            /// Connects to pre-render events.
            gazebo::event::ConnectionPtr pre_render_connection_;

            /// Keeps track of how many times the camera has been triggered since it last published an image.
            int triggered{0};

            /// Protects trigger.
            std::mutex trigger_mutex_;

            /// Publish a sensor_msgs/PointCloud message from a gazebo laser scan
            void pubPC(
                    const sensor_msgs::msg::Image &depth_msg,
                    const sensor_msgs::msg::CameraInfo &info_msg)
            {
                auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                cloud_msg->header = depth_msg.header;
                cloud_msg->height = depth_msg.height;
                cloud_msg->width = depth_msg.width;
                cloud_msg->is_dense = false;
                cloud_msg->is_bigendian = false;

                sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
                pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

                // Update camera model
                model_.fromCameraInfo(info_msg);

                if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                    depthimage_to_pointcloud2::convert<uint16_t>(depth_msg, cloud_msg, model_);
                } else if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                    depthimage_to_pointcloud2::convert<float>(depth_msg, cloud_msg, model_);
                } else {
                    RCLCPP_ERROR(rosnode_->get_logger(), "Depth image has unsupported encoding [%s]", depth_msg.encoding.c_str());
                    return;
                }

                pub_point_cloud_->publish(cloud_msg);
            }

    };
    // Register the plugin
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

        GazeboRosRealsense::GazeboRosRealsense()
        : impl_(std::make_unique<GazeboRealsensePrivate>())
        {
        }

    GazeboRosRealsense::~GazeboRosRealsense()
    {
        //RCLCPP_INFO(impl_->rosnode_->get_logger(), "realsense Unloaded");
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


        this->impl_->color_pub_ = image_transport::create_camera_publisher(impl_->rosnode_.get(),"camera/color/image_raw");
        this->impl_->ir1_pub_ =   image_transport::create_camera_publisher(impl_->rosnode_.get(),"camera/ir/image_raw"    );
        this->impl_->ir2_pub_ =   image_transport::create_camera_publisher(impl_->rosnode_.get(),"camera/ir2/image_raw"   );
        this->impl_->depth_pub_ = image_transport::create_camera_publisher(impl_->rosnode_.get(),"camera/depth/image_raw" );


        this->impl_->pub_point_cloud_ = this->impl_->rosnode_->create_publisher<sensor_msgs::msg::PointCloud2>(
                "camera/pointcloud_jc/image_raw", rclcpp::SensorDataQoS());

    }

    void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
            const transport::PublisherPtr pub)
    {
        common::Time current_time = this->world->SimTime();

        // identify camera
        auto cam_name = cam->Name(); 
        std::string camera_id = extractCameraName(cam_name);
        const std::map<std::string, image_transport::CameraPublisher*> camera_publishers = {
            {color_frame_name, &(this->impl_->color_pub_)},
            {ired1_frame_name, &(this->impl_->ir1_pub_)},
            {ired2_frame_name, &(this->impl_->ir2_pub_)},
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
            {color_frame_name, this->colorCam},
            {ired1_frame_name, this->ired1Cam},
            {ired2_frame_name, this->ired2Cam},
        };

        // publish  ROS
        auto camera_info_msg = cameraInfo(this->impl_->image_msg, cameras.at(camera_id)->HFOV().Radian());
        image_pub->publish(this->impl_->image_msg, camera_info_msg);
    }

    void GazeboRosRealsense::OnNewDepthFrame()
    {
        // get current time
        common::Time current_time = this->world->SimTime();

        RealSensePlugin::OnNewDepthFrame();

        // copy data into image
        this->impl_->depth_msg.header.frame_id = depth_frame_name;
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
        this->impl_->pubPC(this->impl_->depth_msg, depth_info_msg);
        this->impl_->depth_pub_.publish(this->impl_->depth_msg, depth_info_msg);

    }

}



/*
bool GazeboRosDepthCamera::FillPointCloudHelper(
    sensor_msgs::PointCloud2 &point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(rows_arg*cols_arg);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");

  point_cloud_msg.is_dense = true;

  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  double hfov = this->parentSensor->DepthCamera()->HFOV().Radian();
  double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));

  // convert depth to point cloud
  for (uint32_t j=0; j<rows_arg; j++)
  {
    double pAngle;
    if (rows_arg>1) pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
    else            pAngle = 0.0;

    for (uint32_t i=0; i<cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      double yAngle;
      if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
      else            yAngle = 0.0;

      double depth = toCopyFrom[index++];

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iter_x      = depth * tan(yAngle);
      *iter_y      = depth * tan(pAngle);
      if(depth > this->point_cloud_cutoff_)
      {
        *iter_z    = depth;
      }
      else //point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
        point_cloud_msg.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = (uint8_t*)(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == rows_arg*cols_arg*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*cols_arg*3+0];
        iter_rgb[1] = image_src[i*3+j*cols_arg*3+1];
        iter_rgb[2] = image_src[i*3+j*cols_arg*3+2];
      }
      else if (this->image_msg_.data.size() == rows_arg*cols_arg)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*cols_arg];
        iter_rgb[1] = image_src[i+j*cols_arg];
        iter_rgb[2] = image_src[i+j*cols_arg];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }

  return true;
}
*/

namespace
{
    std::string extractCameraName(const std::string& name)
    {
        if (name.find("color") != std::string::npos) return color_frame_name;
        if (name.find("left_ir") != std::string::npos) return ired1_frame_name;
        if (name.find("right_ir") != std::string::npos) return ired2_frame_name;

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

