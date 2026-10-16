cmake_minimum_required(VERSION 3.8)
project(panda_arm_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(angles REQUIRED)
find_package(panda_arm_hardware REQUIRED)

# ==================== 公共库 ====================

# 电机读取库：封装 MotorProtocol，提供零力矩安全读取
add_library(motor_reader SHARED src/motor_reader.cpp)
target_include_directories(motor_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_reader
  rclcpp
)

# 只链接轻量协议库 motor_protocol（它仅依赖 rclcpp）。
# 注意：不能用 ament_target_dependencies(... panda_arm_hardware)，
# 那会连带引入 hardware_interface/controller_interface 等 ros2_control 依赖链
#（进一步需要 realtime_tools、angles 等未在工具包 find_package 的包）。
# 此处必须用 plain 签名（不带 PUBLIC/PRIVATE）：ament_target_dependencies
# 内部已用 plain 签名调用过 target_link_libraries，两者不可混用。
if(TARGET panda_arm_hardware::motor_protocol)
  target_link_libraries(motor_reader panda_arm_hardware::motor_protocol)
else()
  target_link_libraries(motor_reader motor_protocol)
endif()

# 配置管理库：读写 real_arm_config 的 YAML 参数
add_library(config_manager SHARED src/config_manager.cpp)
target_include_directories(config_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(config_manager
  rclcpp
  ament_index_cpp
)

# ==================== 工具节点 ====================

# 节点①：读取 7 个电机当前舵盘位置，终端打印一次后退出
add_executable(read_position_node src/read_position_node.cpp)
target_link_libraries(read_position_node motor_reader config_manager)

# 链接 angles（用于 normalize_angle 角度归一化）。
# 同样用 plain 签名，与上方 target_link_libraries 保持一致。
if(TARGET angles::angles)
  target_link_libraries(read_position_node angles::angles)
else()
  target_link_libraries(read_position_node angles)
endif()

# 节点②：零位置校准，把当前姿态写入 motor_params.yaml 的 zero_offsets
add_executable(zero_calibrator_node src/zero_calibrator_node.cpp)
target_link_libraries(zero_calibrator_node motor_reader config_manager)

# 链接 angles（normalize_angle）：把校准偏移量归一化到 ±180°，使 YAML 数值更小更直观
if(TARGET angles::angles)
  target_link_libraries(zero_calibrator_node angles::angles)
else()
  target_link_libraries(zero_calibrator_node angles)
endif()

# ==================== 安装 ====================

install(TARGETS
  motor_reader
  config_manager
  read_position_node
  zero_calibrator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Python 滑块节点（以「度」显示，限位自动从 URDF 读取）
# install(PROGRAMS) 会自动赋予可执行权限，无需手动 chmod
install(PROGRAMS
  scripts/slider_gui_deg.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
