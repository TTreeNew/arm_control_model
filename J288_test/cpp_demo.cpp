#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <sstream>
#include <chrono>
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

// ==================== 协议常量（与Python完全一致） ====================
const double RATIO = 70070.0 / 243.0;

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
class MotorProtocolSync {
public:
    MotorProtocolSync(const std::string& port, int baudrate, double timeout = 0.1);
    ~MotorProtocolSync();

    bool send_and_receive(int motor_id, int mode, int timeout_flag,
                          double outputTor, double outputSpd, double outputPos,
                          double Kp, double Kd, Feedback& fb);

    void close();

private:
    int fd;
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
    : timeout_sec(timeout) {
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        throw std::runtime_error("无法打开串口: " + port + "，错误: " + std::string(strerror(errno)));
    }
    fcntl(fd, F_SETFL, 0); // 清除非阻塞标志
    configure_serial(baudrate);
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

    // 读取超时设置（单位：0.1秒）
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = static_cast<cc_t>(timeout_sec * 10);

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

bool MotorProtocolSync::send_and_receive(int motor_id, int mode, int timeout_flag,
                                         double outputTor, double outputSpd, double outputPos,
                                         double Kp, double Kd, Feedback& fb) {
    // 构建控制包
    std::vector<uint8_t> cmd = build_control_packet(motor_id, mode, timeout_flag,
                                                    outputTor, outputSpd, outputPos, Kp, Kd);

    // 发送控制包
    ssize_t bytesWritten = write_bytes(cmd.data(), cmd.size());
    if (bytesWritten != cmd.size()) {
        std::cerr << "[错误] 发送失败，仅发送了 " << bytesWritten << " 字节" << std::endl;
        return false;
    }

    // 循环读取26字节反馈（处理分包情况）
    uint8_t feedback[26];
    ssize_t bytesRead = 0;
    size_t totalRead = 0;
    auto start = std::chrono::steady_clock::now();

    while (totalRead < 26) {
        bytesRead = read_bytes(feedback + totalRead, 26 - totalRead);
        if (bytesRead < 0) {
            std::cerr << "[错误] 读取串口失败: " << strerror(errno) << std::endl;
            return false;
        }
        totalRead += bytesRead;

        // 超时检查
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed > timeout_sec) {
            std::cerr << "[错误] 未收到完整反馈包（收到 " << totalRead << " 字节）" << std::endl;
            return false;
        }
    }

    // 解析反馈
    return parse_feedback_packet(feedback, fb);
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
int main() {
    // 修改为实际串口（使用 ls /dev/ttyACM* /dev/ttyUSB*）
    const std::string SERIAL_PORT = "/dev/ttyACM0";
    const int BAUD_RATE = 6000000;

    try {
        MotorProtocolSync motor(SERIAL_PORT, BAUD_RATE, 0.2);
        std::cout << "宇树J288/S288电机通信（纯Linux版）已启动" << std::endl;
        std::cout << "输入 'quit' 退出程序" << std::endl;
        std::cout << "命令格式: send <id> <mode> <timeout> <tor> <spd> <pos> <Kp> <Kd>" << std::endl;
        std::cout << "示例命令: send 0 1 0 0.0 0.5 0.0 0.0 1.0" << std::endl;

        std::string user_input;
        while (true) {
            std::cout << "> ";
            std::getline(std::cin, user_input);

            if (user_input == "quit") {
                break;
            }

            if (user_input.substr(0, 5) == "send ") {
                std::istringstream iss(user_input.substr(5));
                int motor_id, mode, timeout_flag;
                double tor, spd, pos, kp, kd;

                if (iss >> motor_id >> mode >> timeout_flag >> tor >> spd >> pos >> kp >> kd) {
                    Feedback result;
                    if (motor.send_and_receive(motor_id, mode, timeout_flag, tor, spd, pos, kp, kd, result)) {
                        std::cout << "[反馈] "
                                  << "ID: " << result.motor_id
                                  << ", 温度: " << result.Temp << "℃"
                                  << ", 电压: " << result.vol << "V"
                                  << ", 力矩: " << result.outputTor << "Nm"
                                  << ", 速度: " << result.outputSpd << "rad/s"
                                  << ", 位置: " << result.outputPos << "rad"
                                  << ", 错误码: " << result.MError
                                  << ", 警告码: " << result.MWarn << std::endl;
                    }
                } else {
                    std::cout << "格式错误！正确格式: send <id> <mode> <timeout> <tor> <spd> <pos> <Kp> <Kd>" << std::endl;
                }
            } else {
                std::cout << "支持的命令: 'send ...' 或 'quit'" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "程序错误: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "程序已正常退出" << std::endl;
    return 0;
}
