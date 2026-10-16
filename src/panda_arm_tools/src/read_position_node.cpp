/**
 * read_position_node — 读取各电机当前舵盘位置
 *
 * 一次性执行：依次向 1~7 号电机下发零力矩指令取回反馈，
 * 在终端打印每个电机的舵盘位置后退出，不常驻、不 spin。
 *
 * 注意：串口为独占设备。若 controller_manager 正在运行（ros2_control_node
 * 已打开串口），本节点将无法打开串口。请先在原终端 Ctrl+C 停掉 launch。
 *
 * 用法：
 *   ros2 run panda_arm_tools read_position_node --ros-args -p serial_port:=/dev/ttyACM0
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

  auto node = rclcpp::Node::make_shared("read_position_node");

  const std::string port = node->declare_parameter("serial_port", "/dev/ttyACM0");
  const double timeout = node->declare_parameter("timeout", 0.2);

  panda_arm_tools::MotorReader reader;

  if (!reader.open(port, timeout)) {
    std::cout << "[错误] 无法打开串口 " << port << "\n"
              << "       若 controller_manager 正在运行，请先 Ctrl+C 停掉 launch，\n"
              << "       串口为独占设备，同一时刻只能被一个进程打开。\n";
    rclcpp::shutdown();
    return 1;
  }

  const double rad2deg = 180.0 / 3.14159265358979323846;
  std::vector<int> ids = {1, 2, 3, 4, 5, 6, 7};

  std::cout << "\n===================== 电机当前舵盘位置 =====================\n";
  // 读取当前零位偏移（motor_params.yaml 的 zero_offsets），用于换算校准后角度。
  // 校准后角度 = 原始 output_pos − zero_offset，即"相对直立零位"的软件角度。
  auto zero_offsets = panda_arm_tools::ConfigManager::get_zero_offsets();
  bool calibrated = false;
  for (double v : zero_offsets) {
    if (std::abs(v) > 1e-9) {
      calibrated = true;
      break;
    }
  }

  std::cout << "串口: " << port << "    波特率: 6000000\n";
  std::cout << "零位校准: "
            << (calibrated ? "已校准（校准后角度 = 原始 − zero_offset）"
                           : "未校准（zero_offsets 全为 0，故两列相同）")
            << "\n\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "  ID     原始(rad)      原始(deg)      校准后(rad)     校准后(deg)     外部编码(rad)      温度     电压    状态\n";
  std::cout << "  ------------------------------------------------------------------------------------------------\n";

  auto states = reader.read_all(ids);
  int ok_count = 0;

  for (size_t i = 0; i < states.size(); i++) {
    const auto & st = states[i];
    // 取对应关节的零位偏移；若配置缺失则按 0 处理
    const double offset = (i < zero_offsets.size()) ? zero_offsets[i] : 0.0;
    // 与硬件层保持一致：同样做归一化，消除「上电基准」导致的一圈偏移，
    // 使本列的「校准后角度」与 /joint_states 显示完全吻合。
    const double calib_pos = angles::normalize_angle(st.output_pos - offset);

    std::cout << "  " << st.motor_id << "   ";

    if (st.ok) {
      ok_count++;
      std::cout << std::setw(11) << st.output_pos << "  "
                << std::setw(11) << (st.output_pos * rad2deg) << "  "
                << std::setw(12) << calib_pos << "  "
                << std::setw(12) << (calib_pos * rad2deg) << "  "
                << std::setw(12) << st.ex_pos << "  "
                << std::setw(5) << st.temp << "  "
                << std::setw(6) << std::setprecision(1) << st.vol
                << std::setprecision(4) << "  ";

      if (st.merror != 0) {
        std::cout << "故障 0x" << std::hex << st.merror << std::dec;
      } else {
        std::cout << "正常";
      }
      std::cout << "\n";
    } else {
      std::cout << "         --            --            --            --            --         --     --   "
                << "无响应 (" << st.error_msg << ")\n";
    }
  }

  std::cout << "  ------------------------------------------------------------------------------------------------\n";
  std::cout << "  在线 " << ok_count << "/" << ids.size() << " 个电机\n";

  // 提示内部/外部编码器偏差（装臂前用来校验编码器一致性）
  bool printed_diff = false;
  for (const auto & st : states) {
    if (st.ok) {
      // 注意：output_pos 可累积、可为负（不循环），而 ex_pos 是 0~2π 循环。
      // 两者直接相减可能得到 ±2π 整数倍的「假偏差」（例如 output_pos=−1.0、
      // ex_pos=5.28 指向同一物理位置，直接相减却得 −6.28）。
      // 归一化到 ±π 后才能反映真实偏差。
      double diff = angles::normalize_angle(st.output_pos - st.ex_pos);
      if (std::abs(diff) > 0.001) {
        if (!printed_diff) {
          std::cout << "\n  [提示] 内部编码器(outputPos) 与 外部编码器(ExPos) 存在偏差：\n";
          printed_diff = true;
        }
        std::cout << "         电机 " << st.motor_id << ": 偏差 "
                  << std::setw(8) << diff << " rad ("
                  << std::setw(7) << (diff * rad2deg) << " deg)\n";
      }
    }
  }
  if (printed_diff) {
    std::cout << "         若偏差稳定，属零点标定差异，可通过 zero_calibrator 校准补偿。\n";
  }

  std::cout << "======================================================================\n\n";

  reader.close();
  rclcpp::shutdown();

  return (ok_count > 0) ? 0 : 2;
}
