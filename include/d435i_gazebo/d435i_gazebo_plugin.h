#ifndef D435I_GAZEBO_PLUGIN_H
#define D435I_GAZEBO_PLUGIN_H

#include <string>
#include <vector>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

namespace gazebo
{
  class D435iGazeboPlugin : public ModelPlugin
  {
  public:
    D435iGazeboPlugin();
    virtual ~D435iGazeboPlugin();

    // Gazebo plugin interface
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    virtual void OnUpdate();

  private:
    // Camera callbacks
    void OnNewColorFrame();
    void OnNewDepthFrame();

    // Helper functions
    sensor_msgs::CameraInfo CameraInfo(const sensor_msgs::Image& image, float horizontal_fov);
    void GenerateAlignedDepthToColor();

    // Gazebo components
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    
    // Camera renderers  
    rendering::CameraPtr color_cam_;
    rendering::DepthCameraPtr depth_cam_;
    
    // Connections
    event::ConnectionPtr color_connection_;
    event::ConnectionPtr depth_connection_;
    event::ConnectionPtr update_connection_;

    // ROS components
    ros::NodeHandle* nh_;
    image_transport::ImageTransport* it_;
    image_transport::CameraPublisher color_pub_;
    image_transport::CameraPublisher depth_pub_;
    image_transport::CameraPublisher aligned_depth_pub_;
    
    // Camera info managers
    boost::shared_ptr<camera_info_manager::CameraInfoManager> color_info_manager_;
    boost::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_;

    // Parameters
    std::string robot_namespace_;
    std::string camera_name_;
    std::string color_frame_id_;
    std::string depth_frame_id_;
    double update_rate_;
    
    // Image data
    sensor_msgs::Image color_msg_;
    sensor_msgs::Image depth_msg_;
    sensor_msgs::Image aligned_depth_msg_;
    
    // Depth map for processing
    std::vector<uint16_t> depth_map_;
    
    // Timing
    common::Time last_update_time_;
    
    // Configuration
    bool initialized_;
  };
}

#endif // D435I_GAZEBO_PLUGIN_H
