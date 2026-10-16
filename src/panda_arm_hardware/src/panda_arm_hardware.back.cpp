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
  kp_.fill(3.0);
  kd_.fill(0.02);
  zero_offset_.fill(0.0);
  last_output_pos_.fill(0.0);
}

PandaArmHardware::~PandaArmHardware()
{
  if (protocol_.is_open()) {
      protocol_.broadcast_command(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0);//释放关节
    protocol_.close_serial();
  }
}

// ==================== ros2_control 生命周期 ====================
//理解：控制器作为插件被加载进ros2_control_node，和info在同一个进程里，他们在同一个作用域下，因此控制器天然就能看见info。
hardware_interface::CallbackReturn PandaArmHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{//触发info向info_的拷贝
  hardware_interface::SystemInterface::on_init(info);

  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "初始化硬件接口: %s", info_.name.c_str());

    serial_port_ = info_.hardware_parameters.at("serial_port");
    // 读取默认速度，YAML 未配置时使用头文件中定义的默认值 0.5，（iterator（迭代器））
    //.first = 键 .second = 值  
    auto it = info_.hardware_parameters.find("default_velocity");
    if (it != info_.hardware_parameters.end()) {
      default_velocity_ = std::stod(it->second);
    }

    for (size_t i = 0; i < info_.joints.size(); i++) {
      const auto & params = info_.joints[i].parameters;
      motor_ids_[i] = static_cast<uint8_t>(std::stoi(params.at("motor_id")));
      kp_[i] = std::stod(params.at("kp"));
      kd_[i] = std::stod(params.at("kd"));

      // 零位置偏移：未配置时默认为 0（即不做偏移）
      auto zo_it = params.find("zero_offset");
      zero_offset_[i] = (zo_it != params.end()) ? std::stod(zo_it->second) : 0.0;
    }

  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Hardware 初始化成功");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PandaArmHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Activating hardware...");

  // 使用 MotorProtocol 打开串口（支持 6M 波特率），最多等待0.2秒超时
  if (!protocol_.open_serial(serial_port_, BAUD_RATE, 0.2)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 初始化电机
  num_connected_motors_ = 0;
  for (size_t i = 0; i < 7; i++) {
    try {
      // 格式: send <id> <mode> <timeout>     <tor>     <spd> <pos> <Kp> <Kd> 
      //           id    模式   超时保护   （输出端）力矩   速度   位置  刚度  阻尼
      MotorFeedback feedback = protocol_.send_and_receive(
        motor_ids_[i], 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
      // 归一化，统一使用软件角度。
      joint_positions_[i] = angles::normalize_angle(feedback.output_pos - zero_offset_[i]);
      last_output_pos_[i] = feedback.output_pos; //原始编码器位置
      joint_velocities_[i] = feedback.output_spd;
      joint_efforts_[i] = feedback.output_tor;
      joint_temperatures_[i] = feedback.temp;
      joint_voltages_[i] = feedback.vol;  //电压

      joint_position_commands_[i] = joint_positions_[i];
      num_connected_motors_++;
      RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "电机 %d 初始化完成, 位置: %.4f rad",
                  motor_ids_[i], joint_positions_[i]);
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"),
                  "电机 %d 未响应，跳过 (关节 %zu 将上报0值): %s",
                  motor_ids_[i], i, e.what());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"),
              "Hardware 激活成功, %zu/7 motors 已连接",
              num_connected_motors_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PandaArmHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"), "Deactivating hardware...");

  if (protocol_.is_open()) {
    try {
      // 释放关节
      protocol_.broadcast_command(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
    } catch (const std::exception & e) {
      RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"), "on_deactivate无法释放关节: %s", e.what());
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
  /*电机协议没有仲裁机制且是半双工，只能轮询发一个读一个。
  如果读写分别放在read和write，也只能更新7次完成一轮轮询。
  所以读写全放write里
*/
}

hardware_interface::return_type PandaArmHardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!protocol_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }
  constexpr double VEL_EPS = 1e-6; // 程序不会使用真正的0，而是极小的数。  constexpr 修饰 编译期常量
  constexpr double MOVE_THRESHOLD = 0.02;   // rad，进入运动
  constexpr double STOP_THRESHOLD = 0.002;  // rad，退出运动（小于进入阈值，形成滞回）
  constexpr double SLOW_ZONE = 0.05;        // rad，仅在最后 0.05 rad（约2.9°）内减速
  constexpr double MIN_VEL = 0.02;          // rad/s，接近目标时的低速，防过冲
  // 到位保留的保持刚度
  constexpr double HOLD_KP = 1.0;
  constexpr double HOLD_KD = 0.2;

  // 逐个电机通信
  for (size_t i = 0; i < num_connected_motors_; i++) {
    // 如果控制器未发布速度指令（|vel_cmd| <= VEL_EPS）且位置有偏差，补上默认速度
    double vel_cmd = joint_velocity_commands_[i];
    double pos_error = joint_position_commands_[i] - joint_positions_[i];
    double abs_err = std::abs(pos_error);
    bool moving = false;   // 接下来是否运动
    bool vel_from_fallback = false;  // 速度是否来自补位（非主动指令）

    if (std::abs(vel_cmd) <= VEL_EPS) {
      // 滞回判定：已在运动则误差小于 STOP_THRESHOLD 才停，静止则误差大于 MOVE_THRESHOLD 才动
      bool should_move = was_moving_[i] ? (abs_err > STOP_THRESHOLD)   //没有给速度的情况下是否运动
                                        : (abs_err > MOVE_THRESHOLD);
      if (should_move) {
        // 速度随误差大小衰减：误差大→接近满速，误差小→接近最小速度，避免固定高速过冲
        double speed = default_velocity_;
        if (abs_err < SLOW_ZONE) {
          speed = std::max(MIN_VEL, default_velocity_ * abs_err / SLOW_ZONE);
        }
        vel_cmd = std::copysign(speed, pos_error);
        moving = true;
        vel_from_fallback = true;  // 标记此速度来自补位逻辑
      }
      // should_move 为 false 时 moving 保持 false，正确进入保持态
    } else {
      moving = true;
    }
    was_moving_[i] = moving;

    double eff_cmd = joint_effort_commands_[i];
    double use_kp = kp_[i];
    double use_kd = kd_[i];

    // 如果目标位置和当前位置几乎一致且没有主动速度指令，
    // 则进入保持模式：力矩清零，但保留较小的保持刚度（不归零），避免完全失刚导致振荡
    if (!moving) {
      eff_cmd = 0.0;
      use_kp = HOLD_KP;
      use_kd = HOLD_KD;
    }

    // DEBUG
    // write的频率是200Hz(ros2_controllers.yaml里update_rate: 200)
    // 每隔50次打印一次（0.25s打印一次）
    //   cmd_pos  目标位置 (rad)
    //   cur_pos  当前实际位置 (rad)
    //   err      位置误差 = cmd_pos - cur_pos (rad)
    //   vel      下发速度 (rad/s)：控制器给的，或硬件补位的
    //   tor      电机上报的实际力矩 (Nm，关节侧)
    //            └ 静止时 ≈ 抗重力力矩；逼近 1.5 说明饱和
    //   kp / kd  本帧实际使用的刚度 / 阻尼
    //   MOVING   运动态（用 kp_/kd_）   IDLE   保持态（用 HOLD_KP/HOLD_KD）
    static int wr_count = 0;
    if (++wr_count % 50 == 1) {
      RCLCPP_INFO(rclcpp::get_logger("PandaArmHardware"),
        "write[%zu]: cmd_pos=%.4f cur_pos=%.4f err=%.4f vel=%.4f tor=%.3f kp=%.1f kd=%.1f %s",
        i, joint_position_commands_[i], joint_positions_[i], pos_error, vel_cmd,
        joint_efforts_[i], use_kp, use_kd,
        moving ? "MOVING" : "IDLE");  //idle：空闲的;闲置的
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
        if (vel_from_fallback) {
          // 通信失败且仍在按旧误差补速度指令：位置未更新，持续发补位指令无意义
          RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"),
            "电机 %d 通信失败: %s (位置未更新, 仍按旧误差补速度指令)", motor_ids_[i], e.what());
        } else {
          RCLCPP_WARN(rclcpp::get_logger("PandaArmHardware"),
            "电机 %d 通信失败: %s (保持上次状态)", motor_ids_[i], e.what());
        }
      }
      // 通信失败时保留上次状态，继续处理下一个电机
    }
  }

  return hardware_interface::return_type::OK;
}

// ==================== 导出接口到框架 ====================
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
