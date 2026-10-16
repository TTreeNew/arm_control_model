#ifndef PANDA_ARM_HARDWARE__MOTOR_PROTOCOL_HPP_
#define PANDA_ARM_HARDWARE__MOTOR_PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <array>
#include <stdexcept>
#include <sys/select.h>

namespace panda_arm_hardware
{

// ==================== 协议常量 ====================
constexpr double RATIO = 70070.0 / 243.0;
constexpr double M_PI_VAL = 3.14159265358979323846;
constexpr int CONTROL_PACKET_SIZE = 20;
constexpr int FEEDBACK_PACKET_SIZE = 26;
constexpr int BAUD_RATE = 6000000;

// 广播ID
constexpr uint8_t BROADCAST_ID = 0x0F;

// ==================== 电机反馈数据结构体 ====================
struct MotorFeedback
{
  uint8_t motor_id;    // 舵机 ID: 0-14, 15 表示向所有舵机广播数据 (此时无返回)
  uint8_t mode;        // 工作模式: 0.锁定 1.FOC 闭环
  uint8_t timeout;     // 0.禁用超时保护 1.开启 (默认1s超时)
  int8_t temp;         // 壳体温度: -128~127°C
  uint8_t sensor;      // 绕组温度: 0-255°C
  float vol;           // 舵机端电压 0-127.5V ，4byte
  uint32_t merror;     // 舵机故障码, 0 = 无故障，4byte
  uint8_t mwarn;       // 舵机警告码, 0 = 无任何警告
  double ex_pos;       // 外部13位绝对编码器测得的舵盘位置 rad，8byte
  double output_tor;   // 舵盘力矩 N*m，8byte
  double output_spd;   // 舵盘速度 rad/s，8byte
  double output_pos;   // 内部17位绝对编码器测得的舵盘位置 rad，8byte
};

// ==================== 电机通信协议类 ====================
// 封装 J288/S288 电机串口通信协议，提供：
//   - termios2 串口打开/关闭/读写
//   - 控制包构建 (build_control_packet)
//   - 反馈包解析 (parse_feedback_packet)
//   - 请求-响应通信 (send_and_receive)
//   - 广播命令 (broadcast_command)
//   - 批量写入 (write_packets_batch)

class MotorProtocol
{
public:
  MotorProtocol();
  ~MotorProtocol();

  // ── 串口操作 ──
  bool open_serial(const std::string & port, int baudrate, double timeout_sec);
  void close_serial();

  // 设置「等待电机反馈包」的超时（秒）。
  // 实时控制循环（write）宜用较短值以快速失败、避免阻塞控制周期；
  // 调试工具（read_position / zero_calibrator）宜用较长值确保可靠读取。
  void set_recv_timeout(double sec) { recv_timeout_sec_ = sec; }
  ssize_t read_serial(uint8_t * buf, size_t len);
  ssize_t write_serial(const uint8_t * buf, size_t len);
  void flush_serial();
  bool is_open() const { return serial_fd_ >= 0; }
  int get_fd() const { return serial_fd_; }

  // ── 控制包构建 ──
  // 构建 20 字节控制包（含 CRC32），mode=0锁定, mode=1 FOC闭环
  std::array<uint8_t, CONTROL_PACKET_SIZE> build_control_packet(
    uint8_t motor_id, uint8_t mode, uint8_t timeout,
    double output_tor, double output_spd, double output_pos,
    double kp, double kd);

  // ── 反馈包解析 ──
  // 解析 26 字节反馈包，失败时抛出 std::runtime_error
  MotorFeedback parse_feedback_packet(const uint8_t * data);

  // ── 请求-响应通信 ──
  // 发送控制包并阻塞等待反馈包，失败时抛出 std::runtime_error
  MotorFeedback send_and_receive(
    uint8_t motor_id, uint8_t mode, uint8_t timeout,
    double output_tor, double output_spd, double output_pos,
    double kp, double kd);

  // ── 广播命令 ──
  // 向 ID=0x0F 广播命令，无反馈。
  // timeout: 1 = 开启超时保护（默认，正常运行时使用）
  //          0 = 禁用超时保护，同时【清除】电机已触发的保护标志
  //              （手册：超时保护触发后，需控制位发 0 才能清除）
  void broadcast_command(uint8_t mode, double tor, double spd, double pos,
                         double kp, double kd, uint8_t timeout = 1);

  // ── 批量写入（用于 realtime write 循环） ──
  // 将多个控制包拷贝到缓冲区中（调用方自行 send）
  void copy_packet_to_buffer(uint8_t * dst, const std::array<uint8_t, CONTROL_PACKET_SIZE> & packet);

private:
  int serial_fd_;

  // 等待电机反馈包的超时（秒），默认 3ms。
  // 实时控制下需远小于控制周期（200Hz → 5ms），避免单电机掉线长时间阻塞；
  // 调试工具可通过 set_recv_timeout() 放宽（如 0.2s）以保证读取可靠。
  double recv_timeout_sec_ = 0.003;

  // CRC32 查表计算
  static uint32_t crc32_lookup(const uint8_t * data, size_t len);
};

// ==================== 辅助函数：小端读写（内联） ====================

inline void write_le16(uint8_t * buf, int16_t val)
{
  buf[0] = static_cast<uint8_t>(val & 0xFF);
  buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

inline void write_le32(uint8_t * buf, int32_t val)
{
  buf[0] = static_cast<uint8_t>(val & 0xFF);
  buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
  buf[2] = static_cast<uint8_t>((val >> 16) & 0xFF);
  buf[3] = static_cast<uint8_t>((val >> 24) & 0xFF);
}

inline int16_t read_le16(const uint8_t * buf)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));
}

inline int32_t read_le32(const uint8_t * buf)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(buf[0]) |
    (static_cast<uint32_t>(buf[1]) << 8) |
    (static_cast<uint32_t>(buf[2]) << 16) |
    (static_cast<uint32_t>(buf[3]) << 24));
}

inline uint32_t read_le32u(const uint8_t * buf)
{
  return static_cast<uint32_t>(buf[0]) |
         (static_cast<uint32_t>(buf[1]) << 8) |
         (static_cast<uint32_t>(buf[2]) << 16) |
         (static_cast<uint32_t>(buf[3]) << 24);
}

}  // namespace panda_arm_hardware

#endif  // PANDA_ARM_HARDWARE__MOTOR_PROTOCOL_HPP_
