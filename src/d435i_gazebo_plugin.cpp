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
    , params_initialized_(false)
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

    // Initialize TF2 listener
    tf_buffer_.reset(new tf2_ros::Buffer());
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

    const std::string color_ns = camera_name_ + "/color";
    const std::string depth_ns = camera_name_ + "/depth";
    
    // Setup publishers on the shared image transport
    std::string color_topic = color_ns + "/image_raw";
    std::string depth_topic = depth_ns + "/image_raw";
    std::string aligned_depth_topic = camera_name_ + "/aligned_depth_to_color/image_raw";

    color_pub_ = it_->advertiseCamera(color_topic, 1);
    depth_pub_ = it_->advertiseCamera(depth_topic, 1);
    aligned_depth_pub_ = it_->advertiseCamera(aligned_depth_topic, 1);

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
      aligned_depth_map_.resize(color_cam_->ImageWidth() * color_cam_->ImageHeight());
    } catch (std::bad_alloc &e) {
      ROS_ERROR("D435iGazeboPlugin: depthMap allocation failed: %s", e.what());
      return;
    }

    // Setup camera connections
    color_connection_ = color_cam_->ConnectNewImageFrame(
        std::bind(&D435iGazeboPlugin::OnNewColorFrame, this));

    depth_connection_ = depth_cam_->ConnectNewDepthFrame(
        std::bind(&D435iGazeboPlugin::OnNewDepthFrame, this));

    initialized_ = true;

    ROS_INFO("D435i Gazebo plugin initialized successfully!");
  }

  void D435iGazeboPlugin::OnNewColorFrame()
  {
    if (!initialized_ || !color_cam_)
      return;

    // Initialize camera parameters on first call
    if (!params_initialized_)
    {
      if (!InitializeCameraParameters())
      {
        ROS_ERROR_THROTTLE(5.0, "Failed to initialize camera parameters. Cannot publish images.");
        return;
      }
    }

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

    // Use cached camera info (only update timestamp)
    color_camera_info_.header = color_msg_.header;
    
    // Publish color image with camera info
    color_pub_.publish(color_msg_, color_camera_info_);
  }

  void D435iGazeboPlugin::OnNewDepthFrame()
  {
    if (!initialized_ || !depth_cam_)
      return;

    // Initialize camera parameters on first call
    if (!params_initialized_)
    {
      if (!InitializeCameraParameters())
      {
        ROS_ERROR_THROTTLE(5.0, "Failed to initialize camera parameters. Cannot publish images.");
        return;
      }
    }

    // Get current time
    common::Time current_time = world_->SimTime();

    // Convert Float depth data to RealSense depth data
    const float* depthDataFloat = depth_cam_->DepthData();
    const unsigned int imageSize = depth_width_ * depth_height_;
    
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

    // Use cached camera info (only update timestamp)
    depth_camera_info_.header = depth_msg_.header;
    
    // Publish depth image with camera info
    depth_pub_.publish(depth_msg_, depth_camera_info_);

    // Generate and publish aligned depth to color
    GenerateAlignedDepthToColor();
  }

  bool D435iGazeboPlugin::InitializeCameraParameters()
  {
    if (params_initialized_)
      return true;

    ROS_INFO("Initializing camera parameters from Gazebo simulation...");

    // Get intrinsics directly from Gazebo camera sensors
    if (!depth_cam_ || !color_cam_)
    {
      ROS_ERROR("Camera sensors not initialized!");
      return false;
    }

    // Calculate depth camera intrinsics from Gazebo camera properties
    depth_width_ = depth_cam_->ImageWidth();
    depth_height_ = depth_cam_->ImageHeight();
    double depth_hfov = depth_cam_->HFOV().Radian();
    
    depth_fx_ = 0.5 * depth_width_ / tan(0.5 * depth_hfov);
    depth_fy_ = depth_fx_; // Assuming square pixels
    depth_cx_ = depth_width_ * 0.5;
    depth_cy_ = depth_height_ * 0.5;
    
    ROS_INFO("Calculated depth camera intrinsics from Gazebo: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
             depth_fx_, depth_fy_, depth_cx_, depth_cy_);

    // Calculate color camera intrinsics from Gazebo camera properties
    color_width_ = color_cam_->ImageWidth();
    color_height_ = color_cam_->ImageHeight();
    double color_hfov = color_cam_->HFOV().Radian();
    
    color_fx_ = 0.5 * color_width_ / tan(0.5 * color_hfov);
    color_fy_ = color_fx_; // Assuming square pixels
    color_cx_ = color_width_ * 0.5;
    color_cy_ = color_height_ * 0.5;
    
    ROS_INFO("Calculated color camera intrinsics from Gazebo: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
             color_fx_, color_fy_, color_cx_, color_cy_);

    // Get extrinsics from TF (must be published by robot_state_publisher or static_transform_publisher)
    if (!tf_buffer_)
    {
      ROS_ERROR("TF buffer not initialized!");
      return false;
    }

    try
    {
      // Get transform from depth optical frame to color optical frame
      geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
          color_frame_id_, depth_frame_id_, ros::Time(0), ros::Duration(2.0));

      // Extract translation
      tx_ = transform_stamped.transform.translation.x;
      ty_ = transform_stamped.transform.translation.y;
      tz_ = transform_stamped.transform.translation.z;

      ROS_INFO("Loaded camera extrinsics from TF: tx=%.4f, ty=%.4f, tz=%.4f", tx_, ty_, tz_);

      // Warn if there's rotation (we only use translation)
      const auto& q = transform_stamped.transform.rotation;
      bool has_rotation = (fabs(q.x) > 1e-3 || fabs(q.y) > 1e-3 || 
                          fabs(q.z) > 1e-3 || fabs(q.w - 1.0) > 1e-3);
      if (has_rotation)
      {
        ROS_WARN("Rotation detected between depth and color frames (qx=%.4f, qy=%.4f, qz=%.4f, qw=%.4f). "
                 "Current implementation uses translation-only alignment.", q.x, q.y, q.z, q.w);
      }
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("Failed to get TF transform from %s to %s: %s",
                depth_frame_id_.c_str(), color_frame_id_.c_str(), ex.what());
      ROS_ERROR("Make sure robot_state_publisher is running and URDF is properly loaded!");
      return false;
    }

    // Create and cache CameraInfo messages (these are static)
    depth_camera_info_ = CreateCameraInfoMsg(depth_width_, depth_height_, 
                                             depth_fx_, depth_fy_, depth_cx_, depth_cy_,
                                             depth_frame_id_);
    
    color_camera_info_ = CreateCameraInfoMsg(color_width_, color_height_,
                                             color_fx_, color_fy_, color_cx_, color_cy_,
                                             color_frame_id_);
    
    aligned_camera_info_ = CreateCameraInfoMsg(color_width_, color_height_,
                                               color_fx_, color_fy_, color_cx_, color_cy_,
                                               color_frame_id_);

    params_initialized_ = true;
    ROS_INFO("Camera parameters initialized successfully from simulation!");
    return true;
  }

  void D435iGazeboPlugin::GenerateAlignedDepthToColor()
  {
    if (!color_cam_ || !depth_cam_ || !params_initialized_)
      return;

    // Get current time
    common::Time current_time = world_->SimTime();

    // Initialize aligned depth map with zeros
    std::fill(aligned_depth_map_.begin(), aligned_depth_map_.end(), 0);

    // Track the actual bounds of projected depth data for optimized hole filling
    unsigned int min_u = color_width_, max_u = 0;
    unsigned int min_v = color_height_, max_v = 0;

    // For each pixel in the depth image
    for (unsigned int v = 0; v < depth_height_; ++v)
    {
      const unsigned int row_offset = v * depth_width_;
      
      for (unsigned int u = 0; u < depth_width_; ++u)
      {
        const uint16_t depth_value = depth_map_[row_offset + u];

        if (depth_value == 0)
          continue; // Skip invalid depth

        // Convert depth value to meters
        const float depth_m = depth_value * DEPTH_SCALE_M;

        // Back-project depth pixel to 3D point in depth camera frame
        const float x_depth = (u - depth_cx_) * depth_m / depth_fx_;
        const float y_depth = (v - depth_cy_) * depth_m / depth_fy_;

        // Transform 3D point from depth frame to color frame (translation only)
        const float x_color = x_depth + tx_;
        const float y_color = y_depth + ty_;
        const float z_color = depth_m + tz_;

        // Project 3D point to color camera image plane
        if (z_color <= 0)
          continue; // Point is behind the color camera

        const int u_color_int = static_cast<int>((x_color * color_fx_ / z_color) + color_cx_ + 0.5);
        const int v_color_int = static_cast<int>((y_color * color_fy_ / z_color) + color_cy_ + 0.5);

        // Check if the projected point is within the color image bounds
        if (u_color_int >= 0 && u_color_int < static_cast<int>(color_width_) &&
            v_color_int >= 0 && v_color_int < static_cast<int>(color_height_))
        {
          const unsigned int aligned_idx = v_color_int * color_width_ + u_color_int;

          // Handle occlusion: keep the closest depth value
          if (aligned_depth_map_[aligned_idx] == 0 || depth_value < aligned_depth_map_[aligned_idx])
          {
            aligned_depth_map_[aligned_idx] = depth_value;
            
            // Update bounds for hole filling optimization
            if (u_color_int < static_cast<int>(min_u)) min_u = u_color_int;
            if (u_color_int > static_cast<int>(max_u)) max_u = u_color_int;
            if (v_color_int < static_cast<int>(min_v)) min_v = v_color_int;
            if (v_color_int > static_cast<int>(max_v)) max_v = v_color_int;
          }
        }
      }
    }

    // Optimized hole filling: only process the region with actual depth data
    // Add 1-pixel border for edge cases
    const unsigned int fill_min_v = (min_v > 1) ? min_v - 1 : 1;
    const unsigned int fill_max_v = (max_v < color_height_ - 2) ? max_v + 1 : color_height_ - 2;
    const unsigned int fill_min_u = (min_u > 1) ? min_u - 1 : 1;
    const unsigned int fill_max_u = (max_u < color_width_ - 2) ? max_u + 1 : color_width_ - 2;

    for (unsigned int v = fill_min_v; v <= fill_max_v; ++v)
    {
      const unsigned int row_offset = v * color_width_;
      
      for (unsigned int u = fill_min_u; u <= fill_max_u; ++u)
      {
        const unsigned int idx = row_offset + u;
        
        if (aligned_depth_map_[idx] == 0)
        {
          // Check 4-connected neighbors and average non-zero values
          const uint16_t top = aligned_depth_map_[idx - color_width_];
          const uint16_t bottom = aligned_depth_map_[idx + color_width_];
          const uint16_t left = aligned_depth_map_[idx - 1];
          const uint16_t right = aligned_depth_map_[idx + 1];
          
          uint32_t sum = 0;
          int count = 0;
          
          if (top) { sum += top; ++count; }
          if (bottom) { sum += bottom; ++count; }
          if (left) { sum += left; ++count; }
          if (right) { sum += right; ++count; }
          
          if (count > 0)
            aligned_depth_map_[idx] = static_cast<uint16_t>(sum / count);
        }
      }
    }

    // Create aligned depth image message
    aligned_depth_msg_.header.stamp.sec = current_time.sec;
    aligned_depth_msg_.header.stamp.nsec = current_time.nsec;
    aligned_depth_msg_.header.frame_id = color_frame_id_;

    sensor_msgs::fillImage(aligned_depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1,
                          color_height_, color_width_,
                          2 * color_width_,
                          reinterpret_cast<const void*>(aligned_depth_map_.data()));

    // Use cached camera info (only update timestamp)
    aligned_camera_info_.header = aligned_depth_msg_.header;
    
    // Publish aligned depth with color camera info
    aligned_depth_pub_.publish(aligned_depth_msg_, aligned_camera_info_);
  }

  sensor_msgs::CameraInfo D435iGazeboPlugin::CreateCameraInfoMsg(unsigned int width, unsigned int height,
                                                                 float fx, float fy, float cx, float cy,
                                                                 const std::string& frame_id)
  {
    sensor_msgs::CameraInfo info_msg;

    info_msg.header.frame_id = frame_id;
    info_msg.distortion_model = "plumb_bob";
    info_msg.height = height;
    info_msg.width = width;

    // Distortion coefficients (assuming no distortion for simulation)
    info_msg.D.resize(5, 0.0);

    // Intrinsic camera matrix K
    info_msg.K[0] = fx;  // fx
    info_msg.K[1] = 0.0;
    info_msg.K[2] = cx;  // cx
    info_msg.K[3] = 0.0;
    info_msg.K[4] = fy;  // fy
    info_msg.K[5] = cy;  // cy
    info_msg.K[6] = 0.0;
    info_msg.K[7] = 0.0;
    info_msg.K[8] = 1.0;

    // Rectification matrix R (identity for monocular camera)
    info_msg.R[0] = 1.0;
    info_msg.R[1] = 0.0;
    info_msg.R[2] = 0.0;
    info_msg.R[3] = 0.0;
    info_msg.R[4] = 1.0;
    info_msg.R[5] = 0.0;
    info_msg.R[6] = 0.0;
    info_msg.R[7] = 0.0;
    info_msg.R[8] = 1.0;

    // Projection matrix P
    info_msg.P[0] = fx;   // fx
    info_msg.P[1] = 0.0;
    info_msg.P[2] = cx;   // cx
    info_msg.P[3] = 0.0;  // Tx (0 for monocular)
    info_msg.P[4] = 0.0;
    info_msg.P[5] = fy;   // fy
    info_msg.P[6] = cy;   // cy
    info_msg.P[7] = 0.0;  // Ty (0 for monocular)
    info_msg.P[8] = 0.0;
    info_msg.P[9] = 0.0;
    info_msg.P[10] = 1.0;
    info_msg.P[11] = 0.0;

    return info_msg;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(gazebo::D435iGazeboPlugin)
