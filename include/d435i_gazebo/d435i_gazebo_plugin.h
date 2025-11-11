#ifndef D435I_GAZEBO_PLUGIN_H
#define D435I_GAZEBO_PLUGIN_H

#include <string>
#include <vector>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

namespace gazebo
{
  class D435iGazeboPlugin : public ModelPlugin
  {
  public:
    D435iGazeboPlugin();
    virtual ~D435iGazeboPlugin();

    // Gazebo plugin interface
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  private:
    // Camera callbacks
    void OnNewColorFrame();
    void OnNewDepthFrame();

    // Helper functions
    void GenerateAlignedDepthToColor();
    bool InitializeCameraParameters();
    sensor_msgs::CameraInfo CreateCameraInfoMsg(unsigned int width, unsigned int height,
                                                float fx, float fy, float cx, float cy,
                                                const std::string& frame_id);

    // Gazebo components
    physics::ModelPtr model_;
    physics::WorldPtr world_;
    
    // Camera renderers  
    rendering::CameraPtr color_cam_;
    rendering::DepthCameraPtr depth_cam_;
    
    // Connections
    event::ConnectionPtr color_connection_;
    event::ConnectionPtr depth_connection_;

    // ROS components
    ros::NodeHandle* nh_;
    image_transport::ImageTransport* it_;
    image_transport::CameraPublisher color_pub_;
    image_transport::CameraPublisher depth_pub_;
    image_transport::CameraPublisher aligned_depth_pub_;

    // TF listener for camera extrinsics
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string robot_namespace_;
    std::string camera_name_;
    std::string color_frame_id_;
    std::string depth_frame_id_;
    
    // Image data
    sensor_msgs::Image color_msg_;
    sensor_msgs::Image depth_msg_;
    sensor_msgs::Image aligned_depth_msg_;
    
    // Depth map for processing
    std::vector<uint16_t> depth_map_;
    std::vector<uint16_t> aligned_depth_map_;
    
    // Cached camera info messages (static, created once)
    sensor_msgs::CameraInfo depth_camera_info_;
    sensor_msgs::CameraInfo color_camera_info_;
    sensor_msgs::CameraInfo aligned_camera_info_;
    
    // Cached image dimensions
    unsigned int depth_width_, depth_height_;
    unsigned int color_width_, color_height_;
    
    // Camera parameters (cached)
    bool params_initialized_;
    
    // Depth camera intrinsics
    float depth_fx_, depth_fy_, depth_cx_, depth_cy_;
    
    // Color camera intrinsics
    float color_fx_, color_fy_, color_cx_, color_cy_;
    
    // Camera extrinsics (depth to color transform - translation only)
    double tx_, ty_, tz_;
    
    // Configuration
    bool initialized_;
  };
}

#endif // D435I_GAZEBO_PLUGIN_H
