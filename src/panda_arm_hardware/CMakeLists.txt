cmake_minimum_required(VERSION 3.8)
project(panda_arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(angles REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# 订阅 std_msgs/String 形式的 /robot_description（重力补偿取模型用）
find_package(std_msgs REQUIRED)
# 重力补偿：pinocchio 提供 RNEA（computeGeneralizedGravity）。
# 注意用 pinocchio::pinocchio（= _default + _parsers），因为要从 URDF 字符串建模型。
find_package(pinocchio REQUIRED)

# 编译电机通信协议库（独立轻量库，供硬件插件与工具节点复用）
# 仅依赖 rclcpp，不引入 hardware_interface 等 ros2_control 依赖，
# 使 panda_arm_tools 等工具可以只链接协议层
add_library(
  motor_protocol
  SHARED
  src/motor_protocol.cpp
)

target_include_directories(
  motor_protocol
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  motor_protocol
  rclcpp
)

# 编译重力补偿库
#   · 独立库（不并进 panda_arm_controller），将来 MPC 节点可复用同一个模型；
#   · 只依赖 pinocchio + Eigen，【不依赖 ROS】—— 因此能脱离 ROS 单测；
#   · 只做静态重力项 g(q)，不做惯性项/科氏项（稳态不需要，见头文件说明）。
add_library(
  gravity_compensation
  SHARED
  src/gravity_compensation.cpp
)

target_include_directories(
  gravity_compensation
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# pinocchio::pinocchio 是 INTERFACE target，会自动带上
# pinocchio_default + pinocchio_parsers（以及 Eigen / Boost 的 include）。
# 不要在本文件里再 find_package(Boost)：pinocchio 的 config 设了
# Boost_NO_BOOST_CMAKE 才找到 Boost，自己再来一次可能撞上另一套 BoostConfig。
target_link_libraries(
  gravity_compensation
  PUBLIC
  pinocchio::pinocchio
)

# 编译硬件接口库
add_library(
  ${PROJECT_NAME}
  SHARED
  src/panda_arm_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  angles
)

target_link_libraries(
  ${PROJECT_NAME}
  motor_protocol
  pthread
)

# 编译自定义控制器库
add_library(
  panda_arm_controller
  SHARED
  src/panda_arm_controller.cpp
  src/trajectory.cpp
)

target_include_directories(
  panda_arm_controller
  PRIVATE
  include
)

ament_target_dependencies(
  panda_arm_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  trajectory_msgs
  realtime_tools
  lifecycle_msgs
  std_msgs
)

target_link_libraries(
  panda_arm_controller
  gravity_compensation
  pthread
)

# 编译 real_rviz_bridge 桥接节点
add_executable(real_rviz_bridge src/real_rviz_bridge.cpp)

ament_target_dependencies(real_rviz_bridge
  rclcpp
  sensor_msgs
  trajectory_msgs
)

# 导出插件
pluginlib_export_plugin_description_file(hardware_interface panda_arm_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface panda_arm_controller.xml)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库文件（插件动态库 + 协议库 + 重力补偿库）
install(
  TARGETS motor_protocol gravity_compensation ${PROJECT_NAME} panda_arm_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装可执行文件（独立节点）
install(
  TARGETS real_rviz_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  motor_protocol
  gravity_compensation
  ${PROJECT_NAME}
  panda_arm_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  # gravity_compensation 的 PUBLIC 依赖：下游若链 panda_arm_controller 也需要它
  pinocchio
)

ament_package()
