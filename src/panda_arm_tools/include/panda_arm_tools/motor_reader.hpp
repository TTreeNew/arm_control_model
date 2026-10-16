#ifndef PANDA_ARM_TOOLS__MOTOR_READER_HPP_
#define PANDA_ARM_TOOLS__MOTOR_READER_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "panda_arm_hardware/motor_protocol.hpp"

namespace panda_arm_tools
{

/// 单个电机的读取结果
struct MotorState
{
  bool ok = false;          ///< 是否读取成功
  int motor_id = 0;         ///< 电机 ID
  double output_pos = 0.0;  ///< 舵盘位置 rad（内部 17 位编码器推算，精度高）
  double ex_pos = 0.0;      ///< 舵盘位置 rad（外部 13 位编码器直测，接近机械真值）
  double output_spd = 0.0;  ///< 舵盘速度 rad/s
  double output_tor = 0.0;  ///< 舵盘力矩 N·m
  int temp = 0;             ///< 壳体温度 °C
  float vol = 0.0f;         ///< 电压 V
  uint32_t merror = 0;      ///< 故障码，0 = 正常
  std::string error_msg;    ///< 失败原因（ok=false 时有效）
};

/// 电机读取器：封装 MotorProtocol，提供「零力矩安全读取」
///
/// 读取时下发 mode=1(FOC 闭环)、timeout=0(禁用超时保护)、
/// 力矩/速度/位置/刚度/阻尼全 0 的指令。
/// 由控制律 τ = τ_ff + kp·(p_des−p) + kd·(ω_des−ω) 可知，
/// kp=kd=0 且 τ_ff=0 时输出力矩 τ=0，电机处于自由状态，
/// 既不会因收到命令而自行转动，又能取回完整反馈包。
class MotorReader
{
public:
  MotorReader();
  ~MotorReader();

  /// 打开串口
  bool open(const std::string & port, double timeout_sec = 0.2);

  void close();
  bool is_open() const;

  /// 读取单个电机当前状态（零力矩，电机不主动出力）
  MotorState read_one(int motor_id);

  /// 批量读取多个电机
  std::vector<MotorState> read_all(const std::vector<int> & motor_ids);

private:
  panda_arm_hardware::MotorProtocol protocol_;
};

}  // namespace panda_arm_tools

#endif  // PANDA_ARM_TOOLS__MOTOR_READER_HPP_
