#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <sstream>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <algorithm>
#include <memory>
#include <map>

#include <fcntl.h>
#include <unistd.h>
// 先 include <linux/termios.h> 获取 termios2/BOTHER 等定义
// 然后用宏防止 <sys/ioctl.h> 中重复定义 winsize/termio
#include <linux/termios.h>
#define winsize linux_winsize
#define termio linux_termio
#include <sys/ioctl.h>
#undef winsize
#undef termio
#include <linux/serial.h>
#include <sys/select.h>

// ==================== 协议常量 ====================
constexpr double RATIO = 70070.0 / 243.0;
constexpr double M_PI_VAL = 3.14159265358979323846;

// CRC32 查表
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

// CRC32 查表计算（与Python版逻辑完全一致）
uint32_t crc32_lookup(const uint8_t* data, size_t len) {
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

// 辅助：小端写入整数
inline void write_le16(uint8_t* buf, int16_t val) {
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
}

inline void write_le32(uint8_t* buf, int32_t val) {
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 24) & 0xFF;
}

inline int16_t read_le16(const uint8_t* buf) {
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

inline int32_t read_le32(const uint8_t* buf) {
    return (int32_t)((uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                     ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
}

inline uint16_t read_le16u(const uint8_t* buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

// ==================== 打包与解析函数 ====================

std::vector<uint8_t> build_control_packet(int motor_id, int mode, int timeout,
                                          double outputTor, double outputSpd,
                                          double outputPos, double Kp, double Kd) {
    motor_id = std::max(0, std::min(15, motor_id));
    mode     = std::max(0, std::min(1, mode));
    timeout  = timeout ? 1 : 0;
    Kp       = std::max(0.0, std::min(2128.523254, Kp));
    Kd       = std::max(0.0, std::min(21.285233, Kd));
    outputTor = std::max(-36.9093, std::min(36.908174, outputTor));
    outputSpd = std::max(-278.90143, std::min(278.90143, outputSpd));
    outputPos = std::max(-1428.018898, std::min(1428.018898, outputPos));

    uint8_t mode_byte = ((motor_id & 0x0F) |
                         ((mode & 0x07) << 4) |
                         ((timeout & 0x01) << 7));

    int32_t k_pos_val  = (int32_t)std::round(Kp / (RATIO * RATIO) * 1280000.0);
    int32_t k_spd_val  = (int32_t)std::round(Kd / (RATIO * RATIO) * 128000000.0);
    int32_t pos_des_val = (int32_t)std::round(outputPos * RATIO * 32768.0 / M_PI_VAL / 2.0);
    int32_t spd_des_val = (int32_t)std::round(outputSpd * RATIO * 2.560 / M_PI_VAL / 2.0);
    int32_t tor_des_val = (int32_t)std::round(outputTor / RATIO * 256000.0);

    k_pos_val  = std::max(-32768, std::min(32767, k_pos_val));
    k_spd_val  = std::max(-32768, std::min(32767, k_spd_val));
    spd_des_val = std::max(-32768, std::min(32767, spd_des_val));
    tor_des_val = std::max(-32768, std::min(32767, tor_des_val));
    pos_des_val = std::max((int64_t)-2147483648LL, std::min((int64_t)2147483647LL, (int64_t)pos_des_val));

    // struct.pack('<hhIhh' ...) → 小端: 2+2+4+2+2 = 12 bytes
    uint8_t comd_bytes[12];
    write_le16(comd_bytes + 0, (int16_t)tor_des_val);
    write_le16(comd_bytes + 2, (int16_t)spd_des_val);
    write_le32(comd_bytes + 4, (int32_t)pos_des_val);
    write_le16(comd_bytes + 8, (int16_t)k_pos_val);
    write_le16(comd_bytes + 10, (int16_t)k_spd_val);

    // packet_without_crc = b'\xFE\xEE' + mode_byte + b'\x00' + comd_bytes  → 16 bytes
    std::vector<uint8_t> packet_without_crc(16);
    packet_without_crc[0] = 0xFE;
    packet_without_crc[1] = 0xEE;
    packet_without_crc[2] = mode_byte;
    packet_without_crc[3] = 0x00;
    std::memcpy(packet_without_crc.data() + 4, comd_bytes, 12);

    uint32_t crc = crc32_lookup(packet_without_crc.data(), packet_without_crc.size());

    std::vector<uint8_t> full_packet(20);
    std::memcpy(full_packet.data(), packet_without_crc.data(), 16);
    write_le32(full_packet.data() + 16, (int32_t)crc);

    return full_packet;
}

// 反馈数据结构
struct FeedbackData {
    int motor_id;
    int mode;
    int timeout;
    int8_t Temp;
    int8_t sensor;
    double vol;
    uint16_t MError;
    int MWarn;       // ExFlag
    double ExPos;
    double outputTor;
    double outputSpd;
    double outputPos;
};

// 解析反馈包，失败返回 nullptr
std::shared_ptr<FeedbackData> parse_feedback_packet(const uint8_t* data, size_t len) {
    if (len != 26)
        return nullptr;
    if (data[0] != 0xFC || data[1] != 0xEE)
        return nullptr;

    uint8_t mode_byte = data[2];
    int motor_id = mode_byte & 0x0F;
    int mode     = (mode_byte >> 4) & 0x07;
    int timeout  = (mode_byte >> 7) & 0x01;

    // fbk = data[3:22] → 19 bytes
    const uint8_t* fbk = data + 3;

    // struct.unpack('<bbb hh i I H BB', fbk)
    // 格式: b(1) + b(1) + b(1) + h(2) + h(2) + i(4) + I(4) + H(2) + B(1) + B(1) = 19 bytes
    int8_t temp          = (int8_t)fbk[0];
    int8_t sensor        = (int8_t)fbk[1];
    int8_t vol_byte      = (int8_t)fbk[2];  // vol is a signed byte in unpack '<b'
    int16_t torque       = read_le16(fbk + 3);
    int16_t speed        = read_le16(fbk + 5);
    int32_t pos          = read_le32(fbk + 7);
    uint32_t MError      = read_le32(fbk + 11); // 'I' unsigned 4-byte
    uint16_t outpos_exflag = read_le16u(fbk + 15);
    uint8_t exsensor2    = fbk[17];
    uint8_t excom        = fbk[18];
    (void)exsensor2;
    (void)excom;

    // 分离位域
    int OutPos = outpos_exflag & 0x1FFF;       // 低13位
    int ExFlag = (outpos_exflag >> 13) & 0x07;  // 高3位

    // CRC 校验: data[2:22] 共 20 字节
    uint32_t crc_received = read_le32(data + 22);
    uint32_t crc_computed = crc32_lookup(data + 2, 20);
    if (crc_received != crc_computed)
        return nullptr;

    // 转换为物理量
    double vol_f      = (double)vol_byte / 2.0;
    double ExPos      = 2.0 * M_PI_VAL * OutPos / 8192.0;
    double outputSpd  = ((double)speed / 2.56) * 2.0 * M_PI_VAL / RATIO;
    double outputTor  = ((double)torque / 256000.0) * RATIO;
    double outputPos  = 2.0 * M_PI_VAL * pos / 32768.0 / RATIO;

    auto fb = std::make_shared<FeedbackData>();
    fb->motor_id  = motor_id;
    fb->mode      = mode;
    fb->timeout   = timeout;
    fb->Temp      = temp;
    fb->sensor    = sensor;
    fb->vol       = vol_f;
    fb->MError    = MError;
    fb->MWarn     = ExFlag;
    fb->ExPos     = ExPos;
    fb->outputTor = outputTor;
    fb->outputSpd = outputSpd;
    fb->outputPos = outputPos;

    return fb;
}

// ==================== 串口通信类（请求-响应模式） ====================

class MotorProtocolSync {
public:
    MotorProtocolSync(const std::string& port, int baudrate = 6000000, double timeout_sec = 0.1)
        : fd_(-1), timeout_sec_(timeout_sec) {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0)
            throw std::runtime_error("无法打开串口: " + port);

        // 使用 termios2 设置自定义波特率（支持非标准波特率如 6M）
        struct termios2 tty2;
        std::memset(&tty2, 0, sizeof(tty2));
        if (ioctl(fd_, TCGETS2, &tty2) != 0)
            throw std::runtime_error("TCGETS2 失败");

        tty2.c_cflag &= ~CBAUD;
        tty2.c_cflag |= (BOTHER | CLOCAL | CREAD);
        tty2.c_ospeed = baudrate;
        tty2.c_ispeed = baudrate;

        // 8N1, 无流控, 原始模式
        tty2.c_cflag &= ~PARENB;
        tty2.c_cflag &= ~CSTOPB;
        tty2.c_cflag &= ~CSIZE;
        tty2.c_cflag |= CS8;
        tty2.c_cflag &= ~CRTSCTS;
        tty2.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty2.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty2.c_iflag &= ~(INLCR | ICRNL | IGNCR);
        tty2.c_oflag &= ~OPOST;

        // 设置超时
        tty2.c_cc[VMIN]  = 26;   // 最少读取 26 字节
        tty2.c_cc[VTIME] = (uint8_t)(timeout_sec * 10.0 + 0.5); // 0.1s → 1
        if (tty2.c_cc[VTIME] < 1) tty2.c_cc[VTIME] = 1;

        if (ioctl(fd_, TCSETS2, &tty2) != 0)
            throw std::runtime_error("TCSETS2 失败");

        // 清空缓冲区
        ioctl(fd_, TCFLSH, TCIOFLUSH);
    }

    ~MotorProtocolSync() {
        close();
    }

    std::shared_ptr<FeedbackData> send_and_receive(int motor_id, int mode, int timeout_flag,
                                                    double outputTor, double outputSpd,
                                                    double outputPos, double Kp, double Kd) {
        auto cmd = build_control_packet(motor_id, mode, timeout_flag,
                                        outputTor, outputSpd, outputPos, Kp, Kd);
        ssize_t written = write(fd_, cmd.data(), cmd.size());
        if (written != (ssize_t)cmd.size()) {
            std::cerr << "[错误] 发送数据失败" << std::endl;
            return nullptr;
        }

        // 读取 26 字节反馈
        uint8_t feedback[26];
        ssize_t total = 0;
        while (total < 26) {
            ssize_t n = read(fd_, feedback + total, 26 - total);
            if (n <= 0) {
                std::cerr << "[错误] 未收到完整反馈包（收到 " << total << " 字节）" << std::endl;
                return nullptr;
            }
            total += n;
        }

        auto parsed = parse_feedback_packet(feedback, 26);
        if (!parsed) {
            std::cerr << "[错误] 反馈包校验失败或格式错误" << std::endl;
            return nullptr;
        }
        return parsed;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    int fd_;
    double timeout_sec_;
};

// ==================== 主程序 ====================

const std::string SERIAL_PORT = "/dev/ttyACM0";

int main() {
    std::unique_ptr<MotorProtocolSync> motor;
    try {
        motor = std::make_unique<MotorProtocolSync>(SERIAL_PORT, 6000000, 0.2);
        std::cout << "电机通信（请求-响应模式）已启动。输入 'quit' 退出。" << std::endl;
        std::cout << "格式: send <id> <mode> <timeout> <tor> <spd> <pos> <Kp> <Kd>" << std::endl;
        std::cout << "示例: send 0 1 0 0.0 0.5 0.0 0.0 1.0" << std::endl;

        std::string line;
        while (true) {
            std::cout << "> " << std::flush;
            if (!std::getline(std::cin, line))
                break;
            if (line.empty())
                continue;

            // 去除首尾空格
            size_t start = line.find_first_not_of(" \t");
            if (start == std::string::npos)
                continue;
            size_t end = line.find_last_not_of(" \t");
            line = line.substr(start, end - start + 1);

            if (line == "quit")
                break;

            if (line.rfind("send ", 0) == 0) {
                std::string args = line.substr(5);
                std::istringstream iss(args);
                int motor_id, mode, timeout_flag;
                double tor, spd, pos, kp, kd;

                if (iss >> motor_id >> mode >> timeout_flag >> tor >> spd >> pos >> kp >> kd) {
                    auto result = motor->send_and_receive(motor_id, mode, timeout_flag,
                                                          tor, spd, pos, kp, kd);
                    if (result) {
                        std::cout << "[反馈] "
                                  << "motor_id=" << result->motor_id
                                  << ", mode=" << result->mode
                                  << ", timeout=" << result->timeout
                                  << ", Temp=" << (int)result->Temp
                                  << ", sensor=" << (int)result->sensor
                                  << ", vol=" << result->vol
                                  << ", MError=" << result->MError
                                  << ", MWarn=" << result->MWarn
                                  << ", ExPos=" << result->ExPos
                                  << ", outputTor=" << result->outputTor
                                  << ", outputSpd=" << result->outputSpd
                                  << ", outputPos=" << result->outputPos
                                  << std::endl;
                    }
                } else {
                    std::cout << "格式: send <id> <mode> <timeout> <tor> <spd> <pos> <Kp> <Kd>" << std::endl;
                }
            } else {
                std::cout << "支持命令: 'send ...' 或 'quit'" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "串口错误: " << e.what() << std::endl;
    }

    std::cout << "程序退出" << std::endl;
    return 0;
}
