#include "panda_arm_hardware/panda_arm_hardware.hpp"

#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <algorithm>

// 角度归一化（normalize_angle）：消除编码器上电基准导致的 ±360° 偏移
#include <angles/angles.h>

#include "pluginlib/class_list_macros.hpp"

namespace panda_arm_hardware
{

// ==================== PandaArmHardware 实现 ====================

PandaArmHardware::PandaArmHardware()
  : num_connected_motors_(0)
{
  joint_positions_.fill(0.0);
  joint_velocities_.fill(0.0);
  joint_efforts_.fill(0.0);
  joint_temperatures_.fill(0);
  joint_voltages_.fill(0.0f);
  joint_position_commands_.fill(0.0);
  joint_velocity_commands_.fill(0.0);
  joint_effort_commands_.fill(0.0);
  motor_ids_.fill(0);
  // kp/kd 的真正来源是 on_init() 从 URDF 的 <param> 读入
  // （motor_params.yaml → xacro → URDF → HardwareInfo）。
  // 这里填值只是防御性初始化，保证 on_init 之前不会有未初始化的值被读到；
  // 刻意填 0.0 而不是 3.0/0.02：
  //   ① 那两个非 0 值永远到不了电机 —— 缺参数时 params.at("kp") 会抛异常、
  //      on_init 返回 ERROR、硬件整个加载失败，不存在"悄悄用默认值"的路径；
  //   ② 万一真有漏配，默认 0 是最安全的（电机自由，只剩减速器阻尼），
  //      而 3.0 会带刚度顶住 —— 上电瞬间猛地弹一下，比自由更危险。
  kp_.fill(0.0);
  kd_.fill(0.0);
  zero_offset_.fill(0.0);
  last_output_pos_.fill(0.0);
}

PandaArmHardware::~PandaArmHardware()
{
  // 兜底释放：on_deactivate() 是生命周期回调，Ctrl+C 强退时 controller_manager
  // 不会主动触发它，因此析构函数是最可靠的最后一道防线，
  // 确保任何退出路径都把电机置为零力矩自由状态。
  if (protocol_.is_open()) {
    try {
      // mode=1(FOC) + timeout=0(清除超时保护) + τ_ff/spd/pos/kp/kd 全 0
      // → τ = 0 + 0·Δp + 0·Δω = 0，电机自由（仅剩减速器自身阻尼）
      protocol_.broadcast_command(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
    } catch (const std::exception &) {
      // 析构阶段不再向外抛异常
    }
    protocol_.close_serial();
  }
}

// ==================== ros2_control 生命周期 ====================
//理解：控制器作为插件被加载进ros2_control_node，和info在同一个进程里，他们在同一个作用域下，因此控制器天然就能看见info。
hardware_interface::CallbackReturn PandaArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "初始化硬件接口: %s", info_.name.c_str());

  // validate_hardware_info 是无副作用的纯函数，显式传参便于单独测试
  if (!validate_hardware_info(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    // 以下统一从成员 info_ 读取，与 export_*_interfaces() 保持同一个数据源
    serial_port_ = info_.hardware_parameters.at("serial_port");

    // 读取默认速度，YAML 未配置时使用头文件中定义的默认值 0.5，（iterator（迭代器））
    //.first = 键 .second = 值  
    //必需的参数用 .at()，可选即有默认值的参数用 .find()

    for (size_t i = 0; i < info_.joints.size(); i++) {
      const auto & params = info_.joints[i].parameters;
      motor_ids_[i] = static_cast<uint8_t>(std::stoi(params.at("motor_id")));
      kp_[i] = std::stod(params.at("kp"));
      kd_[i] = std::stod(params.at("kd"));

      // 零位置偏移：未配置时默认为 0（即不做偏移）
      auto zo_it = params.find("zero_offset");
      zero_offset_[i] = (zo_it != params.end()) ? std::stod(zo_it->second) : 0.0;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("PandaArmHardware"), "Failed to parse hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Hardware initialized successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PandaArmHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Activating hardware...");

  // 使用 MotorProtocol 打开串口（支持 6M 波特率）
  if (!protocol_.open_serial(serial_port_, BAUD_RATE, 0.2)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 逐个初始化电机，失败的跳过不报错（测试模式）
  num_connected_motors_ = 0;
  for (size_t i = 0; i < 7; i++) {
    try {
      // 零力矩查询指令：mode=1(FOC闭环)，但 τ_ff / 速度 / 位置 / kp / kd 全 0
      // → 控制律 τ = 0 + 0×Δp + 0×Δω = 0，完全不影响电机，仅取回真实位置。
      //
      // timeout=1：开启超时保护（1 秒未收到新指令则电机自动保护停机）。
      // 正常激活后 write() 以 200Hz 持续发指令，不会误触发；
      // 但若激活后系统异常（未进入 write 循环），电机 1 秒后自动保护，
      // 避免长期停留在零力矩自由状态却无保护。
      MotorFeedback feedback = protocol_.send_and_receive(
        motor_ids_[i], 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0);

      // 与 write() 保持一致：减零位偏移并归一化，统一使用「软件角度」。
      // 归一化消除「编码器上电时圈数清零」导致的 ±360° 恒定偏移，
      // 使初始软件角度落在 ±180° 内（如显示 -100° 而非 260°）。
      joint_positions_[i] = angles::normalize_angle(feedback.output_pos - zero_offset_[i]);
      // 同时保存【原始】编码器位置，作为后续下发的基准（保留圈数信息）
      last_output_pos_[i] = feedback.output_pos;
      joint_velocities_[i] = feedback.output_spd;
      joint_efforts_[i] = feedback.output_tor;
      joint_temperatures_[i] = feedback.temp;
      joint_voltages_[i] = feedback.vol;  //电压

      // 命令初始化为当前实际位置（软件角度），防止上电猛拽；
      // write() 下发时会自动加回 zero_offset 转成编码器绝对角度
      joint_position_commands_[i] = joint_positions_[i];

      num_connected_motors_++;
      RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "电机 %d 初始化完成, 位置: %.4f rad",
                  motor_ids_[i], joint_positions_[i]);
    } catch (const std::exception & e) {
      // 测试模式：仅警告，不中断激活
      RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"),
                  "电机 %d 未响应，跳过 (关节 %zu 将上报0值): %s",
                  motor_ids_[i], i, e.what());
      // 未连接的关节保持 0 值
    }
  }

  if (num_connected_motors_ == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PandaArmHardware"), "没有任何电机响应！");
    protocol_.close_serial();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"),
              "Hardware activated successfully, %zu/7 motors connected",
              num_connected_motors_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PandaArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Deactivating hardware...");

  if (protocol_.is_open()) {
    try {
      // 零力矩释放：mode=1(FOC 闭环) + timeout=0(清除超时保护) + 力矩/速度/位置/kp/kd 全 0
      // → 控制律 τ = 0 + 0×Δp + 0×Δω = 0，电机进入自由状态（仅剩减速器自身阻尼）
      //
      // ⚠️ mode 必须是 1：mode=0 是「锁定」模式，走内部位置环、根本不读 kp/kd，
      //    会把电机锁死 —— 比正常运行时还硬，与"退出时释放"的意图完全相反。
      //    同理 kp/kd 必须为 0，否则 mode=1 下仍会按位置误差持续出力矩。
      protocol_.broadcast_command(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"), "Failed to send release command: %s", e.what());
    }
    protocol_.close_serial();
  }

  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Hardware deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==================== read/write ====================

hardware_interface::return_type PandaArmHardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  // 电机没有独立的"读"操作，发一帧回一帧，所以把读写放在一个状态完成
  // 反馈包已在 write() 中通过 send_and_receive 逐个读取
  // read() 只需要返回当前状态即可
  if (!protocol_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PandaArmHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!protocol_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  // 逐个电机通信：发一个控制包 → 收一个反馈包（与 on_activate 中一致的模式）
  // 批量发送在高速串口下可能导致电机解析异常或反馈时序混乱
  //
  // 本循环是【纯执行器】：把命令接口原样送进电机。
  //   p_des = joint_position_commands_   （控制器 / 桥接已完成限速 + 插值）
  //   v_des = joint_velocity_commands_
  //   τ_ff  = joint_effort_commands_
  //   kp/kd = kp_[i] / kd_[i]
  for (size_t i = 0; i < num_connected_motors_; i++) {
    const double pos_error = joint_position_commands_[i] - joint_positions_[i];
    const double vel_cmd   = joint_velocity_commands_[i];
    const double eff_cmd   = joint_effort_commands_[i];
    const double use_kp    = kp_[i];
    const double use_kd    = kd_[i];

    // DEBUG
    // write 的频率是 200Hz（ros2_controllers.yaml 里 update_rate: 200）
    // 每隔 50 次打印一次（约 0.25s）
    //   cmd_pos  目标位置 (rad)          cur_pos  当前实际位置 (rad)
    //   err      位置误差 = cmd_pos - cur_pos (rad)
    //   vel      下发的速度指令 (rad/s)
    //   tor      电机上报的实际力矩 (Nm，关节侧)
    //            └ 静止时 ≈ 抗重力力矩；逼近 1.5 说明已饱和
    //   kp / kd  本帧实际使用的刚度 / 阻尼
    static int wr_count = 0;
    if (++wr_count % 50 == 1) {
      RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"),
        // kp/kd 用 %.3f 而不是 %.1f：kd 常见值是 0.03，用 %.1f 会打成 "0.0"，
        // 看起来像"阻尼为 0"，容易把人引去查一个不存在的问题。
        "write[%zu]: cmd_pos=%.4f cur_pos=%.4f err=%.4f vel=%.4f tor=%.3f kp=%.3f kd=%.3f",
        i, joint_position_commands_[i], joint_positions_[i], pos_error, vel_cmd,
        joint_efforts_[i], use_kp, use_kd);
    }


    try {
      // 下发 = 【上一帧原始编码器位置】+ 误差，而非直接下发绝对目标。
      //
      // 为什么不能直接下发 (命令 + zero_offset)：
      //   归一化后的软件角度丢失了圈数，若下发绝对目标，电机内部 PD 计算的
      //   (p_des - p_cur) 可能反向绕一大圈（例如算出 -350° 而非 +10°），导致撞限位。
      //
      // 正确做法（增量式下发）：
      //   误差:  pos_error      = cmd_software - joint_positions_（软件空间，弧内路径）
      //   下发:  pos_cmd_motor  = last_output_pos_[i] + pos_error（编码器空间）
      // 由于误差在软件空间与编码器空间数值相同（zero_offset 相减时抵消），
      // 这样可确保电机沿弧内可达路径走，不会反向绕圈。
      double pos_cmd_motor = last_output_pos_[i] + pos_error;

      // timeout=1：开启超时保护。
      // write() 以 200Hz(5ms) 持续下发，远小于 1 秒阈值，正常工作不会触发；
      // 仅当系统卡死 / 节点异常退出（>1 秒无指令）时电机才自动保护停机。
      MotorFeedback feedback = protocol_.send_and_receive(
        motor_ids_[i], 1, 1,
        eff_cmd, vel_cmd, pos_cmd_motor,
        use_kp, use_kd);

      // 减零位偏移并归一化 → 相对零位的软件角度。
      // 归一化保证 joint_positions_ 始终在 ±180° 内，
      // 工作范围 ±160° 完整落在其中，供误差计算与上层(/joint_states)显示使用。
      joint_positions_[i]   = angles::normalize_angle(feedback.output_pos - zero_offset_[i]);
      // 保存【原始】编码器位置，作为下一周期的下发基准（保留圈数信息）
      last_output_pos_[i]   = feedback.output_pos;
      joint_velocities_[i]  = feedback.output_spd;
      joint_efforts_[i]     = feedback.output_tor;
      joint_temperatures_[i] = feedback.temp;
      joint_voltages_[i]    = feedback.vol;
      
      if (feedback.merror != 0 || feedback.mwarn != 0) {  //merror是舵机故障码
        RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"), "Motor %d error: 0x%08X, warning: 0x%02X",
                    motor_ids_[i], feedback.merror, feedback.mwarn);
      }
    } catch (const std::exception & e) {
      static int err_count = 0;
      if (++err_count % 50 == 1) {
        RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"),
          "电机 %d 通信失败: %s (保持上次状态)", motor_ids_[i], e.what());
      }
      // 通信失败时保留上次状态，继续处理下一个电机
    }
  }

  return hardware_interface::return_type::OK;
}

// ==================== 导出接口 ====================
// 三种接口【硬编码】，不读 URDF 的 <command_interface> 声明。
// 原因：MIT 协议控制帧固定包含 位置/速度/力矩/刚度/阻尼 五个量，
//       硬件物理上永远需要这三个命令缓冲区，与 URDF 声明无关。
//       URDF 声明由 validate_hardware_info() 校验，不一致会在 on_init 报错。

// 从export开始数据经历了多层取地址和引用。（地址->引用->地址）
// ① hardware用取地址运算符 & 造出指向硬件数据的指针 double*，构造 CommandInterface 对象。
// ② ResourceManager 用 CommandInterface& 构造 LoanedCommandInterface 对象，LoanedCommandInterface 成员保存CommandInterface 的引用。
//    控制器激活时， ResourceManager 把 LoanedCommandInterface 交给控制器。
// ③ 由于 std::vector 容器不能直接放引用，所以通过 reference_wrapper<LoanedCommandInterface> 构造类对象，
//    内部保存指向 LoanedCommandInterface 的指针。
//    joint_command_interface_[4] / joint_state_interface_[4]装的是 reference_wrapper 构造的类的对象
std::vector<hardware_interface::StateInterface> PandaArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    state_interfaces.emplace_back(joint.name, "position", &joint_positions_[i]);
    state_interfaces.emplace_back(joint.name, "velocity", &joint_velocities_[i]);
    state_interfaces.emplace_back(joint.name, "effort", &joint_efforts_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PandaArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    command_interfaces.emplace_back(joint.name, "position", &joint_position_commands_[i]); //&是取地址
    command_interfaces.emplace_back(joint.name, "velocity", &joint_velocity_commands_[i]);
    command_interfaces.emplace_back(joint.name, "effort", &joint_effort_commands_[i]);
  }

  return command_interfaces;
}

bool PandaArmHardware::validate_hardware_info(const hardware_interface::HardwareInfo & info)
{
  if (info.joints.size() != 7) {
    RCLCPP_ERROR(rclcpp::get_logger("PandaArmHardware"), "期望7个关节, 收到 %zu", info.joints.size());
    return false;
  }

  for (const auto & joint : info.joints) {
    bool has_position_command = false;
    for (const auto & cmd_if : joint.command_interfaces) {
      if (cmd_if.name == "position") {
        has_position_command = true;
        break;
      }
    }
    if (!has_position_command) {
      RCLCPP_ERROR(rclcpp::get_logger("PandaArmHardware"), "Joint %s missing position command interface", joint.name.c_str());
      return false;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;
    bool has_effort_state = false;
    for (const auto & state_if : joint.state_interfaces) {
      if (state_if.name == "position") has_position_state = true;
      if (state_if.name == "velocity") has_velocity_state = true;
      if (state_if.name == "effort") has_effort_state = true;
    }
    if (!has_position_state || !has_velocity_state || !has_effort_state) {
      RCLCPP_ERROR(rclcpp::get_logger("PandaArmHardware"), "Joint %s missing required state interfaces", joint.name.c_str());
      return false;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Hardware info validation passed");
  return true;
}

}  // namespace panda_arm_hardware

// 插件导出宏
PLUGINLIB_EXPORT_CLASS(
  panda_arm_hardware::PandaArmHardware,
  hardware_interface::SystemInterface
)
