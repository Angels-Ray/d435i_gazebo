# d435i_gazebo

Intel RealSense D435i 深度相机的仿真资源和 Gazebo 传感器插件。

## 概述
- 提供 RealSense D435i 的 Gazebo 模型和 URDF/Xacro 描述文件。
- 实现自定义 Gazebo `ModelPlugin`，通过 ROS 发布彩色图像、深度图像和对齐深度图像以及校准的相机信息话题。
- 提供示例启动文件和 Gazebo 示例世界，用于快速验证虚拟传感器。

本软件包面向运行 Gazebo Classic 的 ROS 1 catkin 工作空间（已在 ROS Noetic + Gazebo 11 上测试）。只要列出的依赖项可用，其他 ROS 1 发行版也应该可以正常工作。

## 软件包结构
- `include/` 和 `src/`：`d435i_gazebo_plugin` 的 C++ 实现（`libd435i_gazebo_plugin.so`）。
- `urdf/`：基于 Xacro 的相机本体 URDF 文件（`d435i_robot.urdf.xacro`）。
- `meshes/`：URDF 引用的 STL 和材质资源。
- `launch/`：用于在 Gazebo 中生成相机的启动文件。
- `worlds/`：`d435i_gazebo.launch` 使用的演示世界。

## 依赖项
所有运行时和构建依赖项都在 `package.xml` 中声明。通过 `rosdep` 安装它们：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

主要 ROS 软件包依赖项：
- `gazebo_ros`, `gazebo_dev`
- `roscpp`
- `sensor_msgs`
- `image_transport`
- `camera_info_manager`
- `xacro`

## 构建
将此仓库克隆到 catkin 工作空间（例如，`~/catkin_ws/src`）并使用您首选的 catkin 工具构建：

```bash
cd ~/catkin_ws
catkin_make  # 或 catkin build
source devel/setup.bash
```

## 运行仿真
启动包含相机模型的演示世界：

```bash
roslaunch d435i_gazebo d435i_gazebo.launch
```

对于空世界部署，使用：

```bash
roslaunch d435i_gazebo d435i_simple.launch
```

两个启动文件都会生成 URDF 描述，启动 robot_state_publisher，并加载 Gazebo 插件。根据需要调整启动文件中的 `use_sim_time` 或生成位置参数。

## 发布的话题
所有话题都在 `<cameraName>` 命名空间下（默认为 `camera`）。插件通过 `image_transport::CameraPublisher` 发布，因此每个图像话题都自动配对一个相机信息话题。

| 话题 | 类型 | 描述 |
| --- | --- | --- |
| `<camera>/color/image_raw` | `sensor_msgs/Image` | Gazebo 彩色相机生成的 RGB 图像。 |
| `<camera>/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色流的内参校准信息。 |
| `<camera>/depth/image_raw` | `sensor_msgs/Image` (16UC1) | 深度图像（毫米单位，裁剪范围外置零）。 |
| `<camera>/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度流的内参校准信息。 |
| `<camera>/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` (16UC1) | 重投影到彩色光学框架的深度图像。 |
| `<camera>/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | 对齐深度图像使用彩色内参。 |

## 插件参数
当插件附加到机器人模型时，以下 SDF 元素配置插件：

| 参数 | 默认值 | 描述 |
| --- | --- | --- |
| `robotNamespace` | `""` | 用于发布者和服务的 ROS 命名空间。 |
| `cameraName` | `"camera"` | 话题和 TF 框架的基础名称。 |
| `colorFrameId` | `<cameraName>_color_optical_frame` | 彩色图像的 `frame_id`。 |
| `depthFrameId` | `<cameraName>_depth_optical_frame` | 深度图像的 `frame_id`。 |
| `updateRate` | `30.0` Hz | 插件主循环的目标更新频率。 |
| `prefix` | `""` | 用于定位 Gazebo 传感器的可选作用域名称前缀。 |

在自定义机器人描述中嵌入插件时，更新 URDF/Xacro 或 SDF 以覆盖这些默认值。

## TF 框架
URDF 通过 `robot_state_publisher` 发布以下 TF 框架（后缀取决于 `cameraName`）：
- `<cameraName>_link`
- `<cameraName>_color_optical_frame`
- `<cameraName>_depth_optical_frame`

在与下游感知管道集成时，使用 `rviz` 或 `tf2_tools` 验证变换。

## 贡献
欢迎提交问题和拉取请求。在提交更改之前，请运行 `catkin_lint` 和相关的 Gazebo/ROS 测试。

## 许可证
根据 Apache-2.0 许可证分发。有关完整的许可证文本和维护者信息，请参阅 `LICENSE`。