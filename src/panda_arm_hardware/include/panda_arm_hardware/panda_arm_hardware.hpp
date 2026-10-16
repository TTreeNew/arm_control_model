#ifndef PANDA_ARM_HARDWARE__PANDA_ARM_HARDWARE_HPP_
#define PANDA_ARM_HARDWARE__PANDA_ARM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cstdint>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "panda_arm_hardware/motor_protocol.hpp"

namespace panda_arm_hardware
{

/**
 * ┌──────────────────────────────────────────────────────────────────────┐
 * │ ⚠️ 隐式依赖说明：继承自基类的 info_（务必先读这段）                      │
 * └──────────────────────────────────────────────────────────────────────┘
 *
 * 本类大量使用 info_，但它【并未在本文件中声明】——它是基类
 * hardware_interface::SystemInterface 的 protected 成员：
 *
 *     // system_interface.hpp
 *     protected:
 *       HardwareInfo info_;                 // URDF <ros2_control> 段的解析结果
 *       rclcpp_lifecycle::State lifecycle_state_;
 *
 * 三条必须遵守的约定：
 *
 * 1. on_init() 的【第一件事】必须是
 *        SystemInterface::on_init(info)
 *    因为 info_ 正是由父类的这个实现填充的（父类实现只有一句 info_ = hardware_info;）。
 *    不调它 → info_ 为空 → 后续所有解析与接口导出全部失效。
 *
 * 2. on_init() 之后，info_ 只读不写。
 *
 * 3. 切勿在本类中再次声明同名成员 `HardwareInfo info_;`。
 *    那会触发 C++ 的「名字隐藏」：形成两个独立对象，
 *    父类 on_init() 填基类那份，本类代码读到自己那份（始终为空），
 *    且【编译期不报错】，属于极难排查的静默失效。
 */
class PandaArmHardware : public hardware_interface::SystemInterface
{
public:
  PandaArmHardware();
  ~PandaArmHardware() override;

  // 核心生命周期函数
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 核心读写函数
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 导出接口（Humble版本必须实现）
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  // ── 通信协议对象 ──
  MotorProtocol protocol_;

  // ── 硬件参数 ──
  std::string serial_port_;
  std::array<uint8_t, 7> motor_ids_;
  std::array<double, 7> kp_;
  std::array<double, 7> kd_;
  // 零位置偏移 (rad)，实际关节角度 = 编码器读数(outputPos) - zero_offset_[i]
  // 由 zero_calibrator 工具校准后写入 motor_params.yaml，经 xacro 传入
  std::array<double, 7> zero_offset_;

  // ── 状态数据（硬件 -> 控制器） ──
  std::array<double, 7> joint_positions_;
  std::array<double, 7> joint_velocities_;
  std::array<double, 7> joint_efforts_;
  std::array<int8_t, 7> joint_temperatures_;
  std::array<float, 7> joint_voltages_;
  // 上次读到的【原始编码器位置】（未归一化、保留圈数信息）。
  // 用作下发基准：pos_cmd_motor = last_output_pos_[i] + pos_error。
  // 归一化后的 joint_positions_ 丢失了圈数，无法反推原始 output_pos，
  // 故必须单独保存原始值供下一周期下发使用。
  std::array<double, 7> last_output_pos_;

  // ── 命令数据（控制器 -> 硬件） ──
  std::array<double, 7> joint_position_commands_;
  std::array<double, 7> joint_velocity_commands_;
  std::array<double, 7> joint_effort_commands_;

  // ── 批量通信缓冲区 ──
  std::array<uint8_t, CONTROL_PACKET_SIZE * 7> tx_buffer_;
  std::array<uint8_t, FEEDBACK_PACKET_SIZE * 7> rx_buffer_;

  // ── 测试模式：实际连接的电机数量 ──
  size_t num_connected_motors_;

  // ── 辅助函数 ──
  bool validate_hardware_info(const hardware_interface::HardwareInfo & info);
};

}  // namespace panda_arm_hardware

#endif  // PANDA_ARM_HARDWARE__PANDA_ARM_HARDWARE_HPP_
