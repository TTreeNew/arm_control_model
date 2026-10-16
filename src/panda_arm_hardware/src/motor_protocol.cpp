#include "panda_arm_hardware/motor_protocol.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <cmath>
#include <algorithm>

// termios2 支持自定义波特率（如 6M）
#include <linux/termios.h>
#define winsize linux_winsize
#define termio linux_termio
#include <sys/ioctl.h>
#undef winsize
#undef termio

#include "rclcpp/rclcpp.hpp"

/* 电机api
   协议报文：<id>     <mode>      <timeout>   <tor>  <spd> <pos><Kp><Kd> 
           电机id  电机模式0=锁定 超时保护1开启 输出端力矩 速度  位置 刚度 阻尼
*/

namespace panda_arm_hardware
{

// ==================== CRC32 查表 ====================
static const uint32_t CRC32_TABLE[256] = {
  0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
  0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
  0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
  0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
  0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
  0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
  0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
  0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
  0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
  0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
  0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
  0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
  0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
  0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
  0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
  0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
  0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
  0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
  0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
  0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
  0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
  0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
  0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
  0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
  0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
  0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
  0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
  0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
  0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
  0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
  0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
  0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
};

// ==================== CRC32 实现 ====================

uint32_t MotorProtocol::crc32_lookup(const uint8_t * data, size_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  size_t i = 0;
  while (i + 3 < len) {
    uint8_t b0 = data[i + 3];
    uint8_t b1 = data[i + 2];
    uint8_t b2 = data[i + 1];
    uint8_t b3 = data[i];
    crc = CRC32_TABLE[(crc >> 24) ^ b0] ^ ((crc << 8) & 0xFFFFFFFF);
    crc = CRC32_TABLE[(crc >> 24) ^ b1] ^ ((crc << 8) & 0xFFFFFFFF);
    crc = CRC32_TABLE[(crc >> 24) ^ b2] ^ ((crc << 8) & 0xFFFFFFFF);
    crc = CRC32_TABLE[(crc >> 24) ^ b3] ^ ((crc << 8) & 0xFFFFFFFF);
    i += 4;
  }
  return crc;
}

// ==================== 构造 / 析构 ====================

MotorProtocol::MotorProtocol()
  : serial_fd_(-1)  //冒号后的是初始化列表
{
}

MotorProtocol::~MotorProtocol()
{
  close_serial();
}

// ==================== 串口操作 ====================

bool MotorProtocol::open_serial(const std::string & port, int baudrate, double timeout_sec)
{
  serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorProtocol"), "无法打开串口: %s (%s)", port.c_str(), strerror(errno));
    return false;
  }

  struct termios2 tio;
  std::memset(&tio, 0, sizeof(tio));

  if (ioctl(serial_fd_, TCGETS2, &tio) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorProtocol"), "TCGETS2 失败: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 清除标准波特率标志，启用自定义波特率
  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= (BOTHER | CLOCAL | CREAD);

  // 设置自定义波特率
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  // 8N1
  tio.c_cflag &= ~PARENB;   // 无校验
  tio.c_cflag &= ~CSTOPB;   // 1个停止位
  tio.c_cflag &= ~CSIZE;    // 清除数据位掩码
  tio.c_cflag |= CS8;       // 8个数据位

  // 无流控
  tio.c_cflag &= ~CRTSCTS;

  // 原始模式
  tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_iflag &= ~(INLCR | ICRNL | IGNCR);
  tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP);
  tio.c_oflag &= ~OPOST;

  // 读取超时设置（单位：0.1秒）
  // 使用 VMIN=FEEDBACK_PACKET_SIZE 确保阻塞读到完整反馈包
  tio.c_cc[VMIN]  = FEEDBACK_PACKET_SIZE;   // 26 字节
  tio.c_cc[VTIME] = static_cast<cc_t>(timeout_sec * 10.0 + 0.5);
  if (tio.c_cc[VTIME] < 1) tio.c_cc[VTIME] = 1;

  if (ioctl(serial_fd_, TCSETS2, &tio) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MotorProtocol"), "TCSETS2 失败: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // 验证波特率
  struct termios2 check_tio;
  std::memset(&check_tio, 0, sizeof(check_tio));
  if (ioctl(serial_fd_, TCGETS2, &check_tio) == 0) {
    RCLCPP_INFO(rclcpp::get_logger("MotorProtocol"), "串口 %s 已打开, 波特率: %d (请求: %d)",
                port.c_str(), check_tio.c_ospeed, baudrate);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("MotorProtocol"),
                "验证波特率失败 (TCGETS2): %s", strerror(errno));
  }

  // 清空缓冲区
  ioctl(serial_fd_, TCFLSH, TCIOFLUSH);

  return true;
}

void MotorProtocol::close_serial()
{
  if (serial_fd_ >= 0) {
    // 关闭前先等待内核发送缓冲区排空。
    // write() 仅把数据拷进内核缓冲即返回，若紧接着 close()，
    // 尚未真正发出的字节会被丢弃（USB 转串口尤其明显），
    // 导致退出时下发的「释放电机」指令根本没到达电机。
    //
    // ioctl(TCSBRK, 1) 等价于 tcdrain(fd)。此处沿用本文件的 ioctl 风格，
    // 避免引入 <termios.h> 与已包含的 <linux/termios.h> 发生 struct termios 重定义冲突。
    ::ioctl(serial_fd_, TCSBRK, 1);
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

ssize_t MotorProtocol::read_serial(uint8_t * buf, size_t len)
{
  return ::read(serial_fd_, buf, len);
}

ssize_t MotorProtocol::write_serial(const uint8_t * buf, size_t len)
{
  return ::write(serial_fd_, buf, len);
}

void MotorProtocol::flush_serial()
{
  if (serial_fd_ >= 0) {
    ioctl(serial_fd_, TCFLSH, TCIFLUSH);
  }
}

// ==================== 控制包构建 ====================
//"固定长度数组"容器<类型，长度>
std::array<uint8_t, CONTROL_PACKET_SIZE> MotorProtocol::build_control_packet(
  uint8_t motor_id, uint8_t mode, uint8_t timeout,
  double output_tor, double output_spd, double output_pos,
  double kp, double kd)
{
  std::array<uint8_t, CONTROL_PACKET_SIZE> packet;

  // 参数范围限制
  motor_id = std::max(0, std::min(15, static_cast<int>(motor_id))); //类型转换，把变量 motor_id 转换成 int 类型
  mode = std::max(0, std::min(1, static_cast<int>(mode)));
  timeout = timeout ? 1 : 0;
  kp = std::max(0.0, std::min(2128.523254, kp));
  kd = std::max(0.0, std::min(21.285233, kd));
  output_tor = std::max(-36.9093, std::min(36.908174, output_tor));
  output_spd = std::max(-278.90143, std::min(278.90143, output_spd));
  output_pos = std::max(-1428.018898, std::min(1428.018898, output_pos));

  // mode_byte
  uint8_t mode_byte = ((motor_id & 0x0F) |
                       ((mode & 0x07) << 4) |
                       ((timeout & 0x01) << 7));

  // 转换成转子端参数，电机最终读写的是转子端的参数 RATIO 是减速比
  int32_t k_pos_val = static_cast<int32_t>(std::round(kp / (RATIO * RATIO) * 1280000.0));
  int32_t k_spd_val = static_cast<int32_t>(std::round(kd / (RATIO * RATIO) * 128000000.0));
  int32_t pos_des_val = static_cast<int32_t>(std::round(output_pos * RATIO * 32768.0 / M_PI_VAL / 2.0));
  int32_t spd_des_val = static_cast<int32_t>(std::round(output_spd * RATIO * 2.560 / M_PI_VAL / 2.0));
  int32_t tor_des_val = static_cast<int32_t>(std::round(output_tor / RATIO * 256000.0));

  k_pos_val  = std::max(-32768, std::min(32767, k_pos_val));
  k_spd_val  = std::max(-32768, std::min(32767, k_spd_val));
  spd_des_val = std::max(-32768, std::min(32767, spd_des_val));
  tor_des_val = std::max(-32768, std::min(32767, tor_des_val));

  // 构建无 CRC 部分 (16 字节)
  packet[0] = 0xFE;
  packet[1] = 0xEE;
  packet[2] = mode_byte;
  packet[3] = 0x00;

  write_le16(&packet[4], static_cast<int16_t>(tor_des_val));
  write_le16(&packet[6], static_cast<int16_t>(spd_des_val));
  write_le32(&packet[8], pos_des_val);
  write_le16(&packet[12], static_cast<int16_t>(k_pos_val));
  write_le16(&packet[14], static_cast<int16_t>(k_spd_val));

  // CRC32
  uint32_t crc = crc32_lookup(packet.data(), 16);
  write_le32(&packet[16], static_cast<int32_t>(crc));

  return packet;
}

// ==================== 反馈包解析 ====================
//parse:解析
MotorFeedback MotorProtocol::parse_feedback_packet(const uint8_t * data)
{
  MotorFeedback feedback;

  if (data[0] != 0xFC || data[1] != 0xEE) {
    throw std::runtime_error("未收到反馈报文头");
  }

  // mode_byte
  uint8_t mode_byte = data[2];
  feedback.motor_id = mode_byte & 0x0F;
  feedback.mode = (mode_byte >> 4) & 0x07;
  feedback.timeout = (mode_byte >> 7) & 0x01;

  // CRC校验 (data[2:22], 20 bytes)
  uint32_t crc_received = read_le32u(data + 22);
  uint32_t crc_computed = crc32_lookup(data + 2, 20);
  if (crc_received != crc_computed) {
    throw std::runtime_error("CRC32 校验失败");
  }

  // 解析反馈字段 (data[3:22], 19 bytes)
  // 格式: b(1)+b(1)+b(1)+h(2)+h(2)+i(4)+I(4)+H(2)+B(1)+B(1)
  const uint8_t * fbk = data + 3;

  int8_t temp     = static_cast<int8_t>(fbk[0]);
  uint8_t sensor  = fbk[1];
  int8_t vol_byte = static_cast<int8_t>(fbk[2]);
  int16_t torque  = read_le16(fbk + 3);
  int16_t speed   = read_le16(fbk + 5);
  int32_t pos     = read_le32(fbk + 7);
  uint32_t merror = read_le32u(fbk + 11);

  // outpos_exflag: 2 字节（H 无符号）
  uint16_t outpos_exflag = static_cast<uint16_t>(fbk[15]) |
                           (static_cast<uint16_t>(fbk[16]) << 8);

  // 分离位域
  uint16_t out_pos = outpos_exflag & 0x1FFF;
  uint8_t ex_flag  = (outpos_exflag >> 13) & 0x07;

  // 转换为物理量
  feedback.temp       = temp;
  feedback.sensor     = sensor;
  feedback.vol        = static_cast<float>(vol_byte) / 2.0f;
  feedback.merror     = merror;
  feedback.mwarn      = ex_flag;
  feedback.ex_pos     = 2.0 * M_PI_VAL * static_cast<double>(out_pos) / 8192.0;
  feedback.output_spd = (static_cast<double>(speed) / 2.56) * 2.0 * M_PI_VAL / RATIO;
  feedback.output_tor = (static_cast<double>(torque) / 256000.0) * RATIO;
  feedback.output_pos = 2.0 * M_PI_VAL * static_cast<double>(pos) / 32768.0 / RATIO;

  return feedback;
}

// ==================== 请求-响应通信 ====================

MotorFeedback MotorProtocol::send_and_receive(
  uint8_t motor_id, uint8_t mode, uint8_t timeout,
  double output_tor, double output_spd, double output_pos,
  double kp, double kd)
{
  auto packet = build_control_packet(motor_id, mode, timeout,
                                     output_tor, output_spd, output_pos, kp, kd);
  flush_serial();

  if (write_serial(packet.data(), CONTROL_PACKET_SIZE) != CONTROL_PACKET_SIZE) {
    throw std::runtime_error("发送控制包失败");
  }

  uint8_t fb_data[FEEDBACK_PACKET_SIZE];

  // 使用 select() 做超时保护，因为 VMIN=26 在无数据时会永久阻塞。
  // 超时值由 recv_timeout_sec_ 决定（可通过 set_recv_timeout() 调整）：
  // 实时控制需远小于控制周期（200Hz → 5ms），调试工具可放宽以保证可靠读取。
  struct timeval tv;
  tv.tv_sec = static_cast<time_t>(recv_timeout_sec_);
  tv.tv_usec = static_cast<suseconds_t>(
    (recv_timeout_sec_ - static_cast<double>(tv.tv_sec)) * 1000000.0);

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(serial_fd_, &read_fds);

  int sel_ret = select(serial_fd_ + 1, &read_fds, nullptr, nullptr, &tv);
  if (sel_ret <= 0) {
    throw std::runtime_error("读取反馈超时(无数据)");
  }

  ssize_t total = 0;
  while (total < FEEDBACK_PACKET_SIZE) {
    ssize_t n = read_serial(fb_data + total, FEEDBACK_PACKET_SIZE - total);
    if (n <= 0) {
      throw std::runtime_error("读取反馈超时");
    }
    total += n;
  }

  const MotorFeedback feedback = parse_feedback_packet(fb_data);

  if (feedback.motor_id != motor_id) {
    throw std::runtime_error(
      "反馈包 motor_id 不匹配 (期望 " + std::to_string(static_cast<int>(motor_id)) +
      ", 实际 " + std::to_string(static_cast<int>(feedback.motor_id)) + ")");
  }

  return feedback;
}

// ==================== 广播命令 ====================

void MotorProtocol::broadcast_command(uint8_t mode, double tor, double spd, double pos,
                                      double kp, double kd, uint8_t timeout)
{
  if (serial_fd_ < 0) return;

  // timeout 由调用方指定，不再硬编码：
  //   1 = 开启超时保护（正常运行；停机广播后若系统异常，1 秒无指令则自动保护）
  //   0 = 禁用超时保护，并【清除】电机已触发的保护标志
  auto packet = build_control_packet(BROADCAST_ID, mode, timeout, tor, spd, pos, kp, kd);
  write_serial(packet.data(), CONTROL_PACKET_SIZE);
  // 广播命令没有反馈
}

// ==================== 批量写入辅助 ====================

void MotorProtocol::copy_packet_to_buffer(uint8_t * dst,
                                          const std::array<uint8_t, CONTROL_PACKET_SIZE> & packet)
{
  std::memcpy(dst, packet.data(), CONTROL_PACKET_SIZE);
}

}  // namespace panda_arm_hardware
