#ifndef _GAZEBO_ROS_REALSENSE_PLUGIN_
#define _GAZEBO_ROS_REALSENSE_PLUGIN_

#include "realsense_gazebo_plugin/RealSensePlugin.h"

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <sdf/sdf.hh>

#include <string>
#include <memory>

namespace gazebo 
{

  class GazeboRealsensePrivate;
    /// \brief A plugin that simulates Real Sense camera streams.
  class GazeboRosRealsense : public RealSensePlugin {
    /// \brief Constructor.
    public: GazeboRosRealsense();

    /// \brief Destructor.
    public: ~GazeboRosRealsense();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewDepthFrame();

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: virtual void OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub);

private:
  /// Private data pointer
  std::unique_ptr<GazeboRealsensePrivate> impl_;
  };
}
#endif /* _GAZEBO_ROS_REALSENSE_PLUGIN_ */
