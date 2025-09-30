#include "d435i_gazebo/d435i_gazebo_plugin.h"

#include <algorithm>
#include <sstream>

#define DEPTH_SCALE_M 0.001

namespace gazebo
{
  D435iGazeboPlugin::D435iGazeboPlugin()
    : ModelPlugin()
    , nh_(nullptr)
    , it_(nullptr)
    , initialized_(false)
    , update_rate_(30.0)
  {
  }

  D435iGazeboPlugin::~D435iGazeboPlugin()
  {
    if (nh_)
    {
      delete nh_;
      nh_ = nullptr;
    }
    if (it_)
    {
      delete it_;
      it_ = nullptr;
    }
  }

  void D435iGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    model_ = _model;
    world_ = model_->GetWorld();

    // Initialize ROS if it hasn't been initialized already
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = nullptr;
      ros::init(argc, argv, "d435i_gazebo_plugin", ros::init_options::NoSigintHandler);
      ROS_INFO("ROS initialized in D435i plugin");
    }

    ROS_INFO("D435i Gazebo plugin loading for model: %s", model_->GetName().c_str());

    // Parse SDF parameters
    robot_namespace_ = "";
    if (_sdf->HasElement("robotNamespace"))
      robot_namespace_ = _sdf->Get<std::string>("robotNamespace");

    camera_name_ = "camera";
    if (_sdf->HasElement("cameraName"))
      camera_name_ = _sdf->Get<std::string>("cameraName");

    color_frame_id_ = camera_name_ + "_color_optical_frame";
    if (_sdf->HasElement("colorFrameId"))
      color_frame_id_ = _sdf->Get<std::string>("colorFrameId");

    depth_frame_id_ = camera_name_ + "_depth_optical_frame";
    if (_sdf->HasElement("depthFrameId"))
      depth_frame_id_ = _sdf->Get<std::string>("depthFrameId");

    if (_sdf->HasElement("updateRate"))
      update_rate_ = _sdf->Get<double>("updateRate");

    std::string sdf_prefix;
    if (_sdf->HasElement("prefix"))
      sdf_prefix = _sdf->Get<std::string>("prefix");

    // Build list of candidate prefixes for Gazebo sensor lookup
    auto trim = [](const std::string& input) {
      const auto start = input.find_first_not_of(" \t\r\n");
      if (start == std::string::npos)
        return std::string();
      const auto end = input.find_last_not_of(" \t\r\n");
      return input.substr(start, end - start + 1);
    };

    auto ensure_scope_suffix = [](const std::string& value) {
      if (value.empty())
        return value;
      if (value.size() >= 2 && value.substr(value.size() - 2) == "::")
        return value;
      if (value.back() == ':')
        return value + ':';
      return value + "::";
    };

    std::vector<std::string> sensor_prefixes;
    auto add_prefix = [&](const std::string& raw) {
      const auto cleaned = trim(raw);
      if (cleaned.empty())
        return;
      const auto normalized = ensure_scope_suffix(cleaned);
      if (std::find(sensor_prefixes.begin(), sensor_prefixes.end(), normalized) == sensor_prefixes.end())
        sensor_prefixes.push_back(normalized);
    };

    add_prefix(sdf_prefix);
    add_prefix(model_->GetName());
    add_prefix(model_->GetScopedName());
    add_prefix(model_->GetName() + "::" + camera_name_);
    add_prefix(model_->GetScopedName() + "::" + camera_name_);
    add_prefix(model_->GetName() + "::" + camera_name_ + "_link");
    add_prefix(model_->GetScopedName() + "::" + camera_name_ + "_link");
    sensor_prefixes.push_back("");

    // Create ROS node handle
    nh_ = new ros::NodeHandle(robot_namespace_);
    it_ = new image_transport::ImageTransport(*nh_);

    const std::string color_ns = camera_name_ + "/color";
    const std::string depth_ns = camera_name_ + "/depth";
    
    // Setup publishers on the shared image transport
    std::string color_topic = color_ns + "/image_raw";
    std::string depth_topic = depth_ns + "/image_raw";
    std::string aligned_depth_topic = camera_name_ + "/aligned_depth_to_color/image_raw";

    color_pub_ = it_->advertiseCamera(color_topic, 1);
    depth_pub_ = it_->advertiseCamera(depth_topic, 1);
    aligned_depth_pub_ = it_->advertiseCamera(aligned_depth_topic, 1);

    // Initialize camera info managers with namespaced node handles to avoid /set_camera_info collisions
    color_info_manager_.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle(*nh_, color_ns), camera_name_ + "_color"));
    depth_info_manager_.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle(*nh_, depth_ns), camera_name_ + "_depth"));

    // Get sensors through sensor manager
    sensors::SensorManager* sensor_manager = sensors::SensorManager::Instance();

    auto collect_attempts = [](const std::vector<std::string>& prefixes, const std::string& sensor) {
      std::ostringstream oss;
      bool first = true;
      for (const auto& prefix_candidate : prefixes)
      {
        if (!first)
          oss << ", ";
        first = false;
        oss << (prefix_candidate.empty() ? sensor : prefix_candidate + sensor);
      }
      return oss.str();
    };

    auto find_sensor = [&](const std::string& sensor_name) -> sensors::SensorPtr {
      for (const auto& prefix_candidate : sensor_prefixes)
      {
        const auto scoped_name = prefix_candidate.empty() ? sensor_name : prefix_candidate + sensor_name;
        auto sensor = sensor_manager->GetSensor(scoped_name);
        if (sensor)
        {
          ROS_INFO("Found sensor '%s' as '%s'", sensor_name.c_str(), scoped_name.c_str());
          return sensor;
        }
      }
      ROS_ERROR("Could not find sensor '%s'. Tried: %s", sensor_name.c_str(), collect_attempts(sensor_prefixes, sensor_name).c_str());
      return sensors::SensorPtr();
    };

    // Wait for sensors to be ready and get camera renderers
    common::Time timeout = common::Time(5.0);
    common::Time start_time = world_->SimTime();
    
    while ((world_->SimTime() - start_time) < timeout)
    {
      if (!color_cam_)
      {
        auto color_sensor = std::dynamic_pointer_cast<sensors::CameraSensor>(find_sensor("color"));
        if (color_sensor)
          color_cam_ = color_sensor->Camera();
      }

      if (!depth_cam_)
      {
        auto depth_sensor = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(find_sensor("depth"));
        if (depth_sensor)
          depth_cam_ = depth_sensor->DepthCamera();
      }

      if (color_cam_ && depth_cam_)
        break;

      common::Time::MSleep(100);
    }

    if (!color_cam_)
    {
      ROS_ERROR("Could not acquire color camera sensor after waiting.");
      return;
    }

    if (!depth_cam_)
    {
      ROS_ERROR("Could not acquire depth camera sensor after waiting.");
      return;
    }

    // Initialize depth map
    try {
      depth_map_.resize(depth_cam_->ImageWidth() * depth_cam_->ImageHeight());
    } catch (std::bad_alloc &e) {
      ROS_ERROR("D435iGazeboPlugin: depthMap allocation failed: %s", e.what());
      return;
    }

    // Setup camera connections
    color_connection_ = color_cam_->ConnectNewImageFrame(
        std::bind(&D435iGazeboPlugin::OnNewColorFrame, this));

    depth_connection_ = depth_cam_->ConnectNewDepthFrame(
        std::bind(&D435iGazeboPlugin::OnNewDepthFrame, this));

    // Connect to world update
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&D435iGazeboPlugin::OnUpdate, this));

    last_update_time_ = world_->SimTime();
    initialized_ = true;

    ROS_INFO("D435i Gazebo plugin initialized successfully!");
  }

  void D435iGazeboPlugin::OnUpdate()
  {
    if (!initialized_)
      return;

    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_update_time_).Double();

    if (dt >= 1.0 / update_rate_)
    {
      last_update_time_ = current_time;
      // Update logic if needed
    }
  }

  void D435iGazeboPlugin::OnNewColorFrame()
  {
    if (!initialized_ || !color_cam_)
      return;

    // Get current time
    common::Time current_time = world_->SimTime();

    // Fill image message
    color_msg_.header.stamp.sec = current_time.sec;
    color_msg_.header.stamp.nsec = current_time.nsec;
    color_msg_.header.frame_id = color_frame_id_;

    // Set image encoding based on format
    std::string pixel_format = sensor_msgs::image_encodings::RGB8;

    // Copy from simulation image to ROS msg
    sensor_msgs::fillImage(color_msg_, pixel_format, 
                          color_cam_->ImageHeight(), color_cam_->ImageWidth(),
                          color_cam_->ImageDepth() * color_cam_->ImageWidth(),
                          reinterpret_cast<const void*>(color_cam_->ImageData()));

    // Generate camera info
    auto color_info = CameraInfo(color_msg_, color_cam_->HFOV().Radian());
    
    // Publish color image with camera info
    color_pub_.publish(color_msg_, color_info);
  }

  void D435iGazeboPlugin::OnNewDepthFrame()
  {
    if (!initialized_ || !depth_cam_)
      return;

    // Get current time
    common::Time current_time = world_->SimTime();

    // Get depth map dimensions
    unsigned int imageSize = depth_cam_->ImageWidth() * depth_cam_->ImageHeight();

    // Convert Float depth data to RealSense depth data
    const float* depthDataFloat = depth_cam_->DepthData();
    for (unsigned int i = 0; i < imageSize; ++i) {
      // Check clipping and overflow  
      if (depthDataFloat[i] < 0.2f ||  // rangeMinDepth
          depthDataFloat[i] > 10.0f || // rangeMaxDepth  
          depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
          depthDataFloat[i] < 0) {
        depth_map_[i] = 0;
      } else {
        depth_map_[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
      }
    }

    // Fill depth image message
    depth_msg_.header.stamp.sec = current_time.sec;
    depth_msg_.header.stamp.nsec = current_time.nsec;
    depth_msg_.header.frame_id = depth_frame_id_;

    // Copy depth data to ROS message
    sensor_msgs::fillImage(depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1,
                          depth_cam_->ImageHeight(), depth_cam_->ImageWidth(),
                          2 * depth_cam_->ImageWidth(),
                          reinterpret_cast<const void*>(depth_map_.data()));

    // Generate camera info
    auto depth_info = CameraInfo(depth_msg_, depth_cam_->HFOV().Radian());
    
    // Publish depth image with camera info
    depth_pub_.publish(depth_msg_, depth_info);

    // Generate and publish aligned depth to color
    GenerateAlignedDepthToColor();
  }

  void D435iGazeboPlugin::GenerateAlignedDepthToColor()
  {
    if (!color_cam_ || !depth_cam_)
      return;

    // Get current time
    common::Time current_time = world_->SimTime();

    // Create aligned depth image using color camera frame
    aligned_depth_msg_.header.stamp.sec = current_time.sec;
    aligned_depth_msg_.header.stamp.nsec = current_time.nsec;
    aligned_depth_msg_.header.frame_id = color_frame_id_; // Use color frame!

    sensor_msgs::fillImage(aligned_depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1,
                          depth_cam_->ImageHeight(), depth_cam_->ImageWidth(),
                          2 * depth_cam_->ImageWidth(),
                          reinterpret_cast<const void*>(depth_map_.data()));

    // Generate camera info using COLOR camera parameters
    auto aligned_info = CameraInfo(aligned_depth_msg_, color_cam_->HFOV().Radian());
    
    // Publish aligned depth with COLOR camera info
    aligned_depth_pub_.publish(aligned_depth_msg_, aligned_info);
  }

  sensor_msgs::CameraInfo D435iGazeboPlugin::CameraInfo(const sensor_msgs::Image& image, float horizontal_fov)
  {
    sensor_msgs::CameraInfo info_msg;

    info_msg.header = image.header;
    info_msg.distortion_model = "plumb_bob";
    info_msg.height = image.height;
    info_msg.width = image.width;

    float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

    info_msg.K[0] = focal;  // fx
    info_msg.K[4] = focal;  // fy
    info_msg.K[2] = info_msg.width * 0.5;   // cx
    info_msg.K[5] = info_msg.height * 0.5;  // cy
    info_msg.K[8] = 1.0;

    info_msg.P[0] = info_msg.K[0];  // fx
    info_msg.P[5] = info_msg.K[4];  // fy
    info_msg.P[2] = info_msg.K[2];  // cx
    info_msg.P[6] = info_msg.K[5];  // cy
    info_msg.P[10] = info_msg.K[8]; // 1.0

    return info_msg;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(gazebo::D435iGazeboPlugin)
