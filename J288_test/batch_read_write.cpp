#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
// 先 include <linux/termios.h> 获取 termios2/BOTHER 等定义
// 然后用宏防止 <sys/ioctl.h> 中重复定义 winsize/termio
#include <linux/termios.h>
#define winsize linux_winsize
#define termio linux_termio
#include <sys/ioctl.h>
#undef winsize
#undef termio
/*实测只能轮询，批量发送收不到消息，可能是因为电机收到就会朝总线发消息，半双工模式下批量
发布会导致电机和上位机同时使用总线 */


// ==================== 协议常量（与Python完全一致） ====================
const double RATIO = 70070.0 / 243.0;

constexpr int CONTROL_PACKET_SIZE  = 20;   // 控制包字节数
constexpr int FEEDBACK_PACKET_SIZE = 26;   // 反馈包字节数

// CRC32查表（与Python完全一致）
const uint32_t CRC32_TABLE[] = {
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

// ==================== 反馈数据结构体 ====================
struct Feedback {
    int motor_id;
    int mode;
    int timeout;
    int Temp;
    int sensor;
    double vol;
    uint32_t MError;
    int MWarn;
    double ExPos;
    double outputTor;
    double outputSpd;
    double outputPos;
};

// ==================== 函数声明 ====================
uint32_t crc32_lookup(const uint8_t* data, size_t length);
std::vector<uint8_t> build_control_packet(int motor_id, int mode, int timeout,
                                          double outputTor, double outputSpd, double outputPos,
                                          double Kp, double Kd);
bool parse_feedback_packet(const uint8_t* data, Feedback& fb);

// ==================== 串口通信类（纯Linux版） ====================
//
// 分层结构（严格两层，避免重复实现）：
//
//   最底层 : write_raw()  一次 write 推任意字节流
//            read_frame() 读满一帧 26 字节
//   ─────────────────────────────────────────────
//   单电机 : send()      构 1 个包 + 写出
//            receive()   读 1 帧 + 解析（可按 motor_id 过滤）
//            send_and_receive() = send() + receive()   ← 组合
//   ─────────────────────────────────────────────
//   批量   : send_batch()    构 n 个包 → 一次 write 全部推出
//            receive_batch()  循环读帧 → 按 motor_id 归属到各电机
//
class MotorProtocolSync {
public:
    MotorProtocolSync(const std::string& port, int baudrate, double timeout = 0.1);
    ~MotorProtocolSync();

    // ── 最底层 ──
    // 一次 write 推出整段字节流（内部处理部分写）
    bool write_raw(const uint8_t* data, size_t length);
    // 读满一帧 FEEDBACK_PACKET_SIZE 字节（不解析）；timeout 秒内未凑满返回 false。
    // bytes_got 非空时回填"本次实际读到了多少字节"（含超时前收到的残帧字节）。
    bool read_frame(uint8_t* frame, double timeout, size_t* bytes_got = nullptr);
    // 内核 RX 缓冲里现在有多少字节待取（等价于 FIONREAD）
    int bytes_available();
    // 丢弃内核 RX 缓冲（等价于 C++ 逐个版的 flush_serial / TCIFLUSH）
    void flush_input();

    // ── 单电机：send / receive / send_and_receive ──
    // 只发：构一个控制包写出去
    bool send(int motor_id, int mode, int timeout_flag,
              double outputTor, double outputSpd, double outputPos,
              double Kp, double Kd);
    // 只收：读一帧并解析；expect_id >= 0 时要求反馈的 motor_id 与之一致
    bool receive(Feedback& fb, int expect_id = -1);
    // 组合：send() + receive()
    bool send_and_receive(int motor_id, int mode, int timeout_flag,
                          double outputTor, double outputSpd, double outputPos,
                          double Kp, double Kd, Feedback& fb);

    // ── 批量：一次发全部 + 循环收 ──
    // 为 motor_ids 里每个电机各构一个包，拼成一段后【一次 write】推出
    bool send_batch(const std::vector<int>& motor_ids, int mode, int timeout_flag,
                    double outputTor, double outputSpd, double outputPos,
                    double Kp, double Kd);
    // 循环读帧并按 motor_id 归属；返回成功归属到的电机数。
    // raw_bytes 回填本轮总线上一共出现了多少字节（含凑不成一帧的碎片），
    //   用来区分 "一个字节都没有"(raw_bytes==0) 与 "有数据但帧不合法"(raw_bytes>0)。
    size_t receive_batch(std::vector<Feedback>& out,
                         const std::vector<int>& motor_ids,
                         size_t& crc_err, size_t& id_unknown,
                         size_t& raw_bytes);

    void close();

private:
    int fd;
    // 软件层等待预算（秒）。
    // ⚠️ 注意：串口被刻意配成 VMIN=0 / VTIME=0 的【完全非阻塞】模式，
    //    所有等待都由本类自己的 deadline 循环控制。
    //    原因：VTIME 的单位是 0.1s，最小只能到 0.1s，
    //    一旦出现"半帧"就会阻塞 100~200ms，远超控制周期（200Hz → 5ms）。
    double timeout_sec;

    void configure_serial(int baudrate);
    ssize_t read_bytes(uint8_t* buffer, size_t length);
    ssize_t write_bytes(const uint8_t* buffer, size_t length);
};

// ==================== 函数实现 ====================
uint32_t crc32_lookup(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    size_t i = 0;
    while (i + 3 < length) {
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

std::vector<uint8_t> build_control_packet(int motor_id, int mode, int timeout,
                                          double outputTor, double outputSpd, double outputPos,
                                          double Kp, double Kd) {
    // 参数范围限制（与Python完全一致）
    motor_id = std::clamp(motor_id, 0, 15);
    mode = std::clamp(mode, 0, 1);
    timeout = timeout ? 1 : 0;
    Kp = std::clamp(Kp, 0.0, 2128.523254);
    Kd = std::clamp(Kd, 0.0, 21.285233);
    outputTor = std::clamp(outputTor, -36.9093, 36.908174);
    outputSpd = std::clamp(outputSpd, -278.90143, 278.90143);
    outputPos = std::clamp(outputPos, -1428.018898, 1428.018898);

    // 构建mode_byte
    uint8_t mode_byte = ((motor_id & 0x0F) | ((mode & 0x07) << 4) | ((timeout & 0x01) << 7));

    // 参数转换（与Python公式完全一致）
    int16_t k_pos_val = static_cast<int16_t>(std::round(Kp / (RATIO * RATIO) * 1280000.0));
    int16_t k_spd_val = static_cast<int16_t>(std::round(Kd / (RATIO * RATIO) * 128000000.0));
    int32_t pos_des_val = static_cast<int32_t>(std::round(outputPos * RATIO * 32768.0 / M_PI / 2.0));
    int16_t spd_des_val = static_cast<int16_t>(std::round(outputSpd * RATIO * 2.560 / M_PI / 2.0));
    int16_t tor_des_val = static_cast<int16_t>(std::round(outputTor / RATIO * 256000.0));

    // 转换后参数范围限制
    k_pos_val = std::clamp<int16_t>(k_pos_val, -32768, 32767);
    k_spd_val = std::clamp<int16_t>(k_spd_val, -32768, 32767);
    spd_des_val = std::clamp<int16_t>(spd_des_val, -32768, 32767);
    tor_des_val = std::clamp<int16_t>(tor_des_val, -32768, 32767);
    pos_des_val = std::clamp<int32_t>(pos_des_val, -2147483648, 2147483647);

    // 手动打包为小端序（与Python struct.pack('<hhIhh')完全一致）
    uint8_t comd_bytes[12];
    comd_bytes[0] = static_cast<uint8_t>(tor_des_val & 0xFF);
    comd_bytes[1] = static_cast<uint8_t>((tor_des_val >> 8) & 0xFF);
    comd_bytes[2] = static_cast<uint8_t>(spd_des_val & 0xFF);
    comd_bytes[3] = static_cast<uint8_t>((spd_des_val >> 8) & 0xFF);
    comd_bytes[4] = static_cast<uint8_t>(pos_des_val & 0xFF);
    comd_bytes[5] = static_cast<uint8_t>((pos_des_val >> 8) & 0xFF);
    comd_bytes[6] = static_cast<uint8_t>((pos_des_val >> 16) & 0xFF);
    comd_bytes[7] = static_cast<uint8_t>((pos_des_val >> 24) & 0xFF);
    comd_bytes[8] = static_cast<uint8_t>(k_pos_val & 0xFF);
    comd_bytes[9] = static_cast<uint8_t>((k_pos_val >> 8) & 0xFF);
    comd_bytes[10] = static_cast<uint8_t>(k_spd_val & 0xFF);
    comd_bytes[11] = static_cast<uint8_t>((k_spd_val >> 8) & 0xFF);

    // 构建不带CRC的数据包
    std::vector<uint8_t> packet_without_crc;
    packet_without_crc.push_back(0xFE);
    packet_without_crc.push_back(0xEE);
    packet_without_crc.push_back(mode_byte);
    packet_without_crc.push_back(0x00);
    packet_without_crc.insert(packet_without_crc.end(), comd_bytes, comd_bytes + 12);

    // 计算CRC32
    uint32_t crc = crc32_lookup(packet_without_crc.data(), packet_without_crc.size());

    // 添加CRC（小端序）
    std::vector<uint8_t> full_packet = packet_without_crc;
    full_packet.push_back(static_cast<uint8_t>(crc & 0xFF));
    full_packet.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    full_packet.push_back(static_cast<uint8_t>((crc >> 16) & 0xFF));
    full_packet.push_back(static_cast<uint8_t>((crc >> 24) & 0xFF));

    return full_packet;
}

bool parse_feedback_packet(const uint8_t* data, Feedback& fb) {
    // 检查包头
    if (data[0] != 0xFC || data[1] != 0xEE) {
        return false;
    }

    // 解析mode_byte
    uint8_t mode_byte = data[2];
    fb.motor_id = mode_byte & 0x0F;
    fb.mode = (mode_byte >> 4) & 0x07;
    fb.timeout = (mode_byte >> 7) & 0x01;

    // 提取19字节反馈数据
    const uint8_t* fbk = data + 3;

    // 手动解析小端序数据（与Python struct.unpack('<bbb hh i I H BB')完全一致）
    int8_t temp = static_cast<int8_t>(fbk[0]);
    int8_t sensor = static_cast<int8_t>(fbk[1]);
    int8_t vol = static_cast<int8_t>(fbk[2]);

    int16_t torque = static_cast<int16_t>(fbk[3] | (fbk[4] << 8));
    int16_t speed = static_cast<int16_t>(fbk[5] | (fbk[6] << 8));

    int32_t pos = static_cast<int32_t>(fbk[7] | (fbk[8] << 8) | (fbk[9] << 16) | (fbk[10] << 24));

    uint32_t MError = static_cast<uint32_t>(fbk[11] | (fbk[12] << 8) | (fbk[13] << 16) | (fbk[14] << 24));

    uint16_t outpos_exflag = static_cast<uint16_t>(fbk[15] | (fbk[16] << 8));

    // 分离位域
    uint16_t OutPos = outpos_exflag & 0x1FFF;
    fb.MWarn = (outpos_exflag >> 13) & 0x07;

    // CRC校验
    uint32_t crc_received = static_cast<uint32_t>(data[22] | (data[23] << 8) | (data[24] << 16) | (data[25] << 24));
    uint32_t crc_computed = crc32_lookup(data + 2, 20);

    if (crc_received != crc_computed) {
        return false;
    }

    // 转换为物理量（与Python公式完全一致）
    fb.Temp = temp;
    fb.sensor = sensor;
    fb.vol = vol / 2.0;
    fb.MError = MError;
    fb.ExPos = 2 * M_PI * OutPos / 8192.0;
    fb.outputSpd = (speed / 2.56) * 2 * M_PI / RATIO;
    fb.outputTor = (torque / 256000.0) * RATIO;
    fb.outputPos = 2 * M_PI * pos / 32768.0 / RATIO;

    return true;
}

// ==================== 串口类实现（纯Linux版，使用termios2） ====================
MotorProtocolSync::MotorProtocolSync(const std::string& port, int baudrate, double timeout)
    : fd(-1), timeout_sec(timeout) {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        throw std::runtime_error("无法打开串口: " + port + "，错误: " + std::string(strerror(errno)));
    }
    fcntl(fd, F_SETFL, 0); // 清除非阻塞标志
    configure_serial(baudrate);
    flush_input();
}

void MotorProtocolSync::configure_serial(int baudrate) {
    struct termios2 tio;

    // 获取当前串口配置
    if (ioctl(fd, TCGETS2, &tio) != 0) {
        ::close(fd);
        throw std::runtime_error("获取串口属性失败: " + std::string(strerror(errno)));
    }

    // 清除传统波特率标志，启用自定义波特率模式
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;

    // 直接设置实际波特率数值（支持任意值，包括6000000）
    tio.c_ispeed = baudrate;
    tio.c_ospeed = baudrate;

    // 设置8N1
    tio.c_cflag &= ~PARENB;   // 无校验
    tio.c_cflag &= ~CSTOPB;   // 1个停止位
    tio.c_cflag &= ~CSIZE;    // 清除数据位掩码
    tio.c_cflag |= CS8;       // 8个数据位

    // 禁用流控制
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= (CLOCAL | CREAD);  // 忽略调制解调器状态，启用接收器

    // 原始模式（不进行任何输入输出处理）
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio.c_oflag &= ~OPOST;

    // ── 读取模式：VMIN=0 / VTIME=0 → 完全非阻塞 ──
    //   VMIN  = 最少读多少字节才返回（0 = 不要求）
    //   VTIME = 超时，单位 0.1s（0 = 不等）
    //   两者都为 0 时：read() 立即返回"当前可读的字节数"（可能为 0）。
    //   所有等待都由本类的 deadline 循环 + bytes_available() 自己控制。
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    // 应用新配置
    if (ioctl(fd, TCSETS2, &tio) != 0) {
        ::close(fd);
        throw std::runtime_error("设置串口属性失败: " + std::string(strerror(errno)));
    }

    // 验证波特率是否设置成功
    struct termios2 check_tio;
    if (ioctl(fd, TCGETS2, &check_tio) == 0) {
        if (check_tio.c_ospeed != (speed_t)baudrate) {
            ::close(fd);
            throw std::runtime_error("波特率设置失败！请求: " + std::to_string(baudrate)
                                    + "，实际: " + std::to_string(check_tio.c_ospeed));
        }
    }
}

ssize_t MotorProtocolSync::read_bytes(uint8_t* buffer, size_t length) {
    return read(fd, buffer, length);
}

ssize_t MotorProtocolSync::write_bytes(const uint8_t* buffer, size_t length) {
    return write(fd, buffer, length);
}

// ──────────────────── 最底层 ────────────────────

bool MotorProtocolSync::write_raw(const uint8_t* data, size_t length) {
    size_t sent = 0;
    while (sent < length) {
        const ssize_t n = write_bytes(data + sent, length - sent);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[错误] 写入串口失败: " << strerror(errno) << std::endl;
            return false;
        }
        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool MotorProtocolSync::read_frame(uint8_t* frame, double timeout, size_t* bytes_got) {
    size_t got = 0;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout);
    while (got < FEEDBACK_PACKET_SIZE) {
        if (std::chrono::steady_clock::now() > deadline) {
            if (bytes_got != nullptr) *bytes_got = got;   // 回填残帧字节数
            return false;   // 超时（可能是没响应，也可能是半帧）
        }
        const ssize_t n = read_bytes(frame + got, FEEDBACK_PACKET_SIZE - got);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (bytes_got != nullptr) *bytes_got = got;
            return false;
        }
        if (n == 0) {
            // VMIN=0/VTIME=0 下无数据时立即返回 0，自旋等待即可
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        got += static_cast<size_t>(n);
    }
    if (bytes_got != nullptr) *bytes_got = got;
    return true;
}

int MotorProtocolSync::bytes_available() {
    int n = 0;
    if (fd >= 0 && ioctl(fd, FIONREAD, &n) == 0) {
        return n;
    }
    return 0;
}

void MotorProtocolSync::flush_input() {
    if (fd >= 0) {
        ioctl(fd, TCFLSH, TCIFLUSH);   // 只清输入方向，不动 TX
    }
}

// ──────────────────── 单电机：send / receive / send_and_receive ────────────────────

bool MotorProtocolSync::send(int motor_id, int mode, int timeout_flag,
                             double outputTor, double outputSpd, double outputPos,
                             double Kp, double Kd) {
    const std::vector<uint8_t> cmd = build_control_packet(
        motor_id, mode, timeout_flag, outputTor, outputSpd, outputPos, Kp, Kd);
    return write_raw(cmd.data(), cmd.size());
}

bool MotorProtocolSync::receive(Feedback& fb, int expect_id) {
    uint8_t frame[FEEDBACK_PACKET_SIZE];
    if (!read_frame(frame, timeout_sec)) {
        return false;
    }
    if (!parse_feedback_packet(frame, fb)) {
        return false;
    }
    // expect_id >= 0 时要求是本电机的回复（与硬件层"校验反馈归属"一致）
    if (expect_id >= 0 && fb.motor_id != expect_id) {
        return false;
    }
    return true;
}

bool MotorProtocolSync::send_and_receive(int motor_id, int mode, int timeout_flag,
                                         double outputTor, double outputSpd, double outputPos,
                                         double Kp, double Kd, Feedback& fb) {
    // 组合：先发，再收。收的时候校验反馈确实是这个电机回的。
    if (!send(motor_id, mode, timeout_flag, outputTor, outputSpd, outputPos, Kp, Kd)) {
        return false;
    }
    return receive(fb, motor_id);
}

// ──────────────────── 批量：一次发全部 + 循环收 ────────────────────

bool MotorProtocolSync::send_batch(const std::vector<int>& motor_ids, int mode, int timeout_flag,
                                   double outputTor, double outputSpd, double outputPos,
                                   double Kp, double Kd) {
    // ① 先把 n 个包全部构好（纯内存操作）
    std::vector<uint8_t> buffer;
    buffer.reserve(static_cast<size_t>(CONTROL_PACKET_SIZE) * motor_ids.size());
    for (int id : motor_ids) {
        const std::vector<uint8_t> pkt = build_control_packet(
            id, mode, timeout_flag, outputTor, outputSpd, outputPos, Kp, Kd);
        buffer.insert(buffer.end(), pkt.begin(), pkt.end());
    }
    // ② 一次 write 全部推出
    return write_raw(buffer.data(), buffer.size());
}

size_t MotorProtocolSync::receive_batch(std::vector<Feedback>& out,
                                        const std::vector<int>& motor_ids,
                                        size_t& crc_err, size_t& id_unknown,
                                        size_t& raw_bytes) {
    // 总线静默多久就认为"不会再有帧了"（秒）
    constexpr double IDLE_BREAK_SEC = 0.005;

    out.clear();
    crc_err = 0;
    id_unknown = 0;
    raw_bytes = 0;

    std::vector<bool> filled(motor_ids.size(), false);
    size_t got = 0;

    const auto t_begin = std::chrono::steady_clock::now();
    auto t_last_rx = t_begin;

    while (got < motor_ids.size()) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - t_begin).count();
        const double idle = std::chrono::duration<double>(now - t_last_rx).count();

        if (elapsed > timeout_sec) break;              // 整轮预算用完
        if (got > 0 && idle > IDLE_BREAK_SEC) break;   // 已收到帧且总线静默 → 收完

        // 不足一帧就先等一下（避免 read_frame 里的自旋）
        if (bytes_available() < FEEDBACK_PACKET_SIZE) {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }

        uint8_t frame[FEEDBACK_PACKET_SIZE];
        size_t n_got = 0;
        if (!read_frame(frame, IDLE_BREAK_SEC, &n_got)) {
            raw_bytes += n_got;    // 超时前收到的残帧字节也要算进去
            break;
        }
        raw_bytes += n_got;
        t_last_rx = std::chrono::steady_clock::now();

        Feedback fb;
        if (!parse_feedback_packet(frame, fb)) {
            // 帧头 / CRC 失败 → 丢弃这一帧（字节数已计入 raw_bytes）
            ++crc_err;
            continue;
        }

        // ⭐ 按 motor_id 归属（帧到达顺序不保证等于发送顺序）
        bool matched = false;
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            if (!filled[i] && motor_ids[i] == fb.motor_id) {
                filled[i] = true;
                out.push_back(fb);
                ++got;
                matched = true;
                break;
            }
        }
        if (!matched) {
            ++id_unknown;   // 不属于本轮任何电机 / 同一电机的重复帧
        }
    }

    // 收尾：把滞留在内核缓冲里、不足一帧的碎片也算进 raw_bytes。
    // ⭐ 这一项很关键：没有它，"收到几个垃圾字节后被下一轮 flush 丢掉"
    //    会表现为 raw_bytes==0，与"总线上真的什么都没来"无法区分。
    raw_bytes += static_cast<size_t>(bytes_available());

    return got;
}

void MotorProtocolSync::close() {
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
}

MotorProtocolSync::~MotorProtocolSync() {
    close();
}

// ==================== 主程序 ====================
//
// 安全性：所有控制包都是 kp=kd=tor=spd=pos=0
//         控制律 τ = τ_ff + kp·Δp + kd·Δω = 0 → 电机零力矩自由，不会转动
//         ⚠️ 机械臂请先不装机或扶稳
static void print_usage(const char* prog) {
    std::cout
        << "J288/S288 批量读写可行性测试\n"
        << "\n用法: " << prog << " [模式] [选项]\n"
        << "      不带任何参数直接运行 → 进入交互式菜单\n"
        << "\n模式（可选其一，默认 batch）:\n"
        << "  batch      批量：一次全发 + 循环全收\n"
        << "  poll       基线：逐个发-收\n"
        << "  repl       交互式单条 send（原功能）\n"
        << "\n选项:\n"
        << "  --port DEV       串口设备（默认 /dev/ttyACM0）\n"
        << "  --rounds N       轮数（默认 200）\n"
        << "  --ids 1 2 ...    参与测试的电机 ID（默认 1 2 3 4 5 6 7）\n"
        << "  --rx-timeout MS  每轮接收预算，毫秒（默认 50）\n"
        << "  -h, --help       显示本帮助\n"
        << "\n示例:\n"
        << "  " << prog << "                                  # 交互式菜单\n"
        << "  " << prog << " batch --ids 2 3 --rounds 10 --port /dev/ttyACM0\n"
        << "  " << prog << " poll  --ids 2 3 --rounds 10 --port /dev/ttyACM0\n"
        << "  " << prog << " batch --ids 2 --rounds 10 --rx-timeout 10\n";
}

// 不带参数运行时进入交互式菜单
static void interactive_setup(std::string& port, std::string& mode,
                              int& rounds, std::vector<int>& motor_ids,
                              double& rx_timeout_sec) {
    std::string line;

    std::cout << "============= J288 批量读写测试 =============\n";
    std::cout << "选择模式:\n"
              << "  1) poll   基线：逐个发-收\n"
              << "  2) batch  批量：一次全发 + 循环全收\n"
              << "  3) repl   交互式单条 send\n"
              << "请选择 [1-3] (默认 2): ";
    if (std::getline(std::cin, line) && !line.empty()) {
        const int sel = std::atoi(line.c_str());
        mode = (sel == 1) ? "poll" : ((sel == 3) ? "repl" : "batch");
    } else {
        mode = "batch";
    }

    if (mode == "repl") {
        std::cout << "串口 (默认 " << port << "): ";
        if (std::getline(std::cin, line) && !line.empty()) {
            port = line;
        }
        return;
    }

    std::cout << "串口 (默认 " << port << "): ";
    if (std::getline(std::cin, line) && !line.empty()) {
        port = line;
    }

    std::cout << "轮数 (默认 " << rounds << "): ";
    if (std::getline(std::cin, line) && !line.empty()) {
        rounds = std::atoi(line.c_str());
    }

    std::cout << "每轮接收预算 ms (默认 " << static_cast<int>(rx_timeout_sec * 1000.0) << "): ";
    if (std::getline(std::cin, line) && !line.empty()) {
        rx_timeout_sec = std::atof(line.c_str()) / 1000.0;
    }

    std::cout << "电机ID 空格分隔 (默认 1 2 3 4 5 6 7): ";
    if (std::getline(std::cin, line) && !line.empty()) {
        std::istringstream iss(line);
        std::vector<int> tmp;
        int id = 0;
        while (iss >> id) {
            tmp.push_back(id);
        }
        if (!tmp.empty()) {
            motor_ids = tmp;
        }
    }
    std::cout << "---------------------------------------------\n";
}

static void run_repl(MotorProtocolSync& motor) {
    std::cout << "输入 'quit' 退出程序" << std::endl;
    std::cout << "命令格式: send <id> <mode> <timeout> <tor> <spd> <pos> <Kp> <Kd>" << std::endl;
    std::cout << "示例命令: send 2 1 1 0.0 0.0 0.0 0.0 0.0  (2号电机，零力矩自由)" << std::endl;

    std::string user_input;
    while (true) {
        std::cout << "> ";
        if (!std::getline(std::cin, user_input)) break;
        if (user_input == "quit") break;

        if (user_input.substr(0, 5) == "send ") {
            std::istringstream iss(user_input.substr(5));
            int motor_id, mode, timeout_flag;
            double tor, spd, pos, kp, kd;

            if (iss >> motor_id >> mode >> timeout_flag >> tor >> spd >> pos >> kp >> kd) {
                Feedback result;
                if (motor.send_and_receive(motor_id, mode, timeout_flag,
                                           tor, spd, pos, kp, kd, result)) {
                    std::cout << "[反馈] "
                              << "ID: " << result.motor_id
                              << ", 温度: " << result.Temp << "℃"
                              << ", 电压: " << result.vol << "V"
                              << ", 力矩: " << result.outputTor << "Nm"
                              << ", 速度: " << result.outputSpd << "rad/s"
                              << ", 位置: " << result.outputPos << "rad"
                              << ", 错误码: " << result.MError
                              << ", 警告码: " << result.MWarn << std::endl;
                } else {
                    std::cout << "[无响应] 电机 " << motor_id << " 未返回有效反馈" << std::endl;
                }
            } else {
                std::cout << "格式错误！正确格式: send <id> <mode> <timeout> <tor> <spd> <pos> <Kp> <Kd>" << std::endl;
            }
        } else {
            std::cout << "支持的命令: 'send ...' 或 'quit'" << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    // 默认值（可被命令行参数或交互式菜单覆盖）
    std::string port = "/dev/ttyACM0";
    const int BAUD_RATE = 6000000;
    std::string mode = "batch";
    int rounds = 200;
    double rx_timeout_sec = 0.05;   // 每轮接收预算（秒）
    std::vector<int> motor_ids;

    // ── 命令行解析 ──
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (a == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (a == "--rounds" && i + 1 < argc) {
            rounds = std::atoi(argv[++i]);
        } else if (a == "--rx-timeout" && i + 1 < argc) {
            rx_timeout_sec = std::atof(argv[++i]) / 1000.0;
        } else if (a == "--ids") {
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                motor_ids.push_back(std::atoi(argv[++i]));
            }
        } else if (a == "batch" || a == "poll" || a == "repl") {
            mode = a;
        } else if (!a.empty() && std::isdigit(static_cast<unsigned char>(a[0]))) {
            rounds = std::atoi(a.c_str());   // 兼容 "batch 300" 这种写法
        } else {
            std::cerr << "未知参数: " << a << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    // ── 不带参数 → 交互式菜单 ──
    if (argc == 1) {
        interactive_setup(port, mode, rounds, motor_ids, rx_timeout_sec);
    }

    if (motor_ids.empty()) {
        // 关节电机 ID：对应 real_arm_config/config/panda_arm.urdf.xacro 里
        // motor_id_joint1..7 的默认值 1~7
        motor_ids = {1, 2, 3, 4, 5, 6, 7};
    }
    if (rounds <= 0) {
        rounds = 1;
    }
    if (rx_timeout_sec <= 0.0) {
        rx_timeout_sec = 0.05;
    }

    // ── repl 模式 ──
    if (mode == "repl") {
        try {
            MotorProtocolSync motor(port, BAUD_RATE, 0.2);
            std::cout << "宇树J288/S288电机通信（纯Linux版）已启动，串口: " << port << std::endl;
            run_repl(motor);
        } catch (const std::exception& e) {
            std::cerr << "程序错误: " << e.what() << std::endl;
            return 1;
        }
        std::cout << "程序已正常退出" << std::endl;
        return 0;
    }

    // ── poll / batch 对照实验 ──
    try {
        // 串口本身是 VMIN=0/VTIME=0 非阻塞；timeout_sec 只是软件层等待预算
        MotorProtocolSync motor(port, BAUD_RATE, rx_timeout_sec);

        std::cout << "========== J288 批量读写可行性对照实验 ==========" << std::endl;
        std::cout << "模式: " << mode
                  << "   串口: " << port
                  << "   电机: ";
        for (size_t i = 0; i < motor_ids.size(); ++i) {
            std::cout << motor_ids[i] << (i + 1 < motor_ids.size() ? "," : "");
        }
        std::cout << "   轮数: " << rounds
                  << "   接收预算: " << static_cast<int>(rx_timeout_sec * 1000.0) << " ms"
                  << std::endl;
        std::cout << "指令: id, mode=1(FOC), timeout=1, tor=spd=pos=kp=kd=0"
                  << " → τ=0，电机零力矩自由" << std::endl;
        if (motor_ids.size() == 1) {
            std::cout << "⚠️ 只测了 1 个电机：batch 与 poll 完全等价，结果不能证明批量可行"
                      << std::endl;
        }
        std::cout << "--------------------------------------------------" << std::endl;

        size_t ok = 0, lost = 0, crc_err_total = 0, id_unknown_total = 0, raw_total = 0;
        const auto t0 = std::chrono::steady_clock::now();

        for (int r = 0; r < rounds; ++r) {
            if (mode == "poll") {
                // ── 基线：逐个发-收（与 hardware 现行 write() 一致，带 flush）──
                for (int id : motor_ids) {
                    motor.flush_input();   // 清掉上一轮残留，保证收到的是本轮回复
                    if (!motor.send(id, 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0)) {
                        ++lost;
                        continue;
                    }
                    Feedback fb;
                    if (motor.receive(fb, id) && fb.motor_id == id) {
                        ++ok;
                    } else {
                        ++lost;
                    }
                }
            } else {
                // ── 批量：一次全发 → 循环全收 ──
                motor.flush_input();
                if (!motor.send_batch(motor_ids, 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0)) {
                    lost += motor_ids.size();
                    continue;
                }
                std::vector<Feedback> fbs;
                size_t crc = 0, unk = 0, raw = 0;
                const size_t got = motor.receive_batch(fbs, motor_ids, crc, unk, raw);
                ok += got;
                lost += motor_ids.size() - got;
                crc_err_total += crc;
                id_unknown_total += unk;
                raw_total += raw;
            }
        }

        const double dt = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        const size_t total = static_cast<size_t>(rounds) * motor_ids.size();

        std::cout << "  成功率    " << ok << "/" << total
                  << " = " << (100.0 * ok / static_cast<double>(total)) << "%" << std::endl;
        std::cout << "  丢失      " << lost << std::endl;
        std::cout << "  CRC/帧头错 " << crc_err_total << std::endl;
        std::cout << "  非本轮ID  " << id_unknown_total << std::endl;
        if (mode == "batch") {
            // ⭐ 关键诊断量：总线上一共出现了多少字节（含凑不成帧的碎片）
            std::cout << "  总线原始字节 " << raw_total
                      << "  (期望 ≈ " << total * FEEDBACK_PACKET_SIZE << ")"
                      << (raw_total == 0 ? "   ← 一个字节都没收到"
                                         : "   ← 有字节，但凑不成合法帧")
                      << std::endl;
        }
        std::cout << "  总耗时    " << dt << " s → 每轮 " << (dt / rounds * 1000.0)
                  << " ms，摊到每电机 " << (dt / static_cast<double>(total) * 1000.0)
                  << " ms" << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;

        if (motor_ids.size() == 1) {
            std::cout << "  ⚠️ 只测了 1 个电机：batch 与 poll 完全等价，本结果不能证明批量可行。"
                      << "请用 --ids 2 3 这类多电机再测。" << std::endl;
        } else if (ok == total) {
            // ⚠️ 结论必须区分模式：poll 成功只说明"轮询可行"，与批量可行性无关
            if (mode == "poll") {
                std::cout << "  ✅ 轮询正常（基线）：逐个发-收 100% 成功。"
                          << "请再用 batch 同参数跑一遍做对比。" << std::endl;
            } else {
                std::cout << "  ✅ 批量全部成功：固件能处理背靠背多包" << std::endl;
            }
        } else if (mode == "poll") {
            std::cout << "  ⚠️ 轮询也丢帧：先排查端口/电机/供电，再谈批量" << std::endl;
        } else if (raw_total == 0) {
            std::cout << "  ❌ 总线上一个字节都没来：不是总线争用，更像"
                      << "「固件拒绝处理背靠背的多包」或「端口/电机选错」" << std::endl;
        } else if (crc_err_total > 0 || id_unknown_total > 0) {
            std::cout << "  ❌ 有数据但帧不合法：疑似半双工总线争用" << std::endl;
        } else {
            std::cout << "  ⚠️ 部分丢失：请用 poll 同参数跑一遍做对比" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "程序错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "程序已正常退出" << std::endl;
    return 0;
}
