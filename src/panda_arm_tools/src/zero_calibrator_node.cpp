/**
 * zero_calibrator_node — 零位置校准
 *
 * 把机械臂「当前姿态」标定为零位：读取各电机当前编码器读数，
 * 写入 motor_params.yaml 的 zero_offsets 字段。
 *
 * 硬件层约定：实际关节角度 = 编码器读数(outputPos) − zero_offset
 * 校准后，当前姿态的软件角度即为 0。
 *
 * 注意：单圈绝对编码器的物理位置与读数唯一对应，因此该偏移持久有效，
 * 断电重启后只要物理姿态未变，软件角度仍为 0（无需重新校准）。
 *
 * 用法：
 *   # 标定当前姿态为零位
 *   ros2 run panda_arm_tools zero_calibrator_node --ros-args -p serial_port:=/dev/ttyACM0
 *
 *   # 清除零位（全部归 0）
 *   ros2 run panda_arm_tools zero_calibrator_node --ros-args -p serial_port:=/dev/ttyACM0 -p clear:=true
 */

#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"

#include "panda_arm_tools/config_manager.hpp"
#include "panda_arm_tools/motor_reader.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("zero_calibrator_node");

  const std::string port = node->declare_parameter("serial_port", "/dev/ttyACM0");
  const bool clear = node->declare_parameter("clear", false);
  const double timeout = node->declare_parameter("timeout", 0.2);

  std::vector<double> new_offsets(7, 0.0);

  if (clear) {
    // ── 清除模式：偏移全部置 0，不需要串口 ──
    std::cout << "\n[清除零位] 将 zero_offsets 全部置为 0.0\n";
  } else {
    // ── 校准模式：读当前位置作为偏移 ──
    panda_arm_tools::MotorReader reader;
    if (!reader.open(port, timeout)) {
      std::cout << "[错误] 无法打开串口 " << port << "\n"
                << "       若 controller_manager 正在运行，请先 Ctrl+C 停掉 launch。\n";
      rclcpp::shutdown();
      return 1;
    }

    std::vector<int> ids = {1, 2, 3, 4, 5, 6, 7};
    auto states = reader.read_all(ids);

    std::cout << "\n=================== 零位置校准 ===================\n";
    std::cout << "读取当前姿态，将各电机当前位置设为零位偏移。\n\n";
    std::cout << std::fixed << std::setprecision(4);

    int ok_count = 0;
    for (size_t i = 0; i < states.size(); i++) {
      const auto & st = states[i];
      if (st.ok) {
        ok_count++;
        // 归一化到 ±180°，使写入 YAML 的偏移量是小数值（如 -0.819 而非 5.4641），
        // 一眼即可看出偏离零位的方向与大小。
        // 注意：这只是让配置更易读；真正修正「一圈偏移」的是硬件层的
        // normalize(output_pos - zero_offset)。
        new_offsets[i] = angles::normalize_angle(st.output_pos);
        std::cout << "  电机 " << st.motor_id << ": 当前位置 " << std::setw(9)
                  << st.output_pos << " rad → 零位偏移 " << std::setw(9)
                  << new_offsets[i] << " rad\n";
      } else {
        std::cout << "  电机 " << st.motor_id << ": 无响应 (" << st.error_msg
                  << ")，偏移保持 0.0\n";
      }
    }
    reader.close();

    if (ok_count == 0) {
      std::cout << "\n[错误] 没有任何电机响应，未写入配置。\n";
      rclcpp::shutdown();
      return 2;
    }
    std::cout << "\n共 " << ok_count << "/7 个电机参与校准。\n";
  }

  // ── 写入 YAML（src 持久化 + install 立即生效）──
  if (!panda_arm_tools::ConfigManager::set_zero_offsets(new_offsets)) {
    std::cout << "\n[错误] 写入 motor_params.yaml 失败。\n";
    rclcpp::shutdown();
    return 3;
  }

  std::cout << "\n[完成] zero_offsets 已写入 motor_params.yaml（src 与 install 两处）。\n";

  // 回读校验
  auto verify = panda_arm_tools::ConfigManager::get_zero_offsets();
  if (verify.size() == 7) {
    std::cout << "\n回读校验：\n";
    std::cout << std::fixed << std::setprecision(4);
    for (size_t i = 0; i < 7; i++) {
      std::cout << "  joint_" << (i + 1) << ": " << std::setw(9) << verify[i] << " rad\n";
    }
  }

  std::cout << "\n--------------------------------------------------------\n";
  std::cout << "生效方式：\n";
  std::cout << "  零位偏移经 xacro 展开进 URDF，由硬件层 on_init 读取，\n";
  std::cout << "  需重启 launch 生效：\n";
  std::cout << "    ros2 launch real_arm_config slider_control.launch.py \\\n";
  std::cout << "      serial_port:=" << port << "\n";
  std::cout << "  若之后执行过 colcon build，请一并确认配置已同步到 install。\n";
  std::cout << "--------------------------------------------------------\n\n";

  rclcpp::shutdown();
  return 0;
}
