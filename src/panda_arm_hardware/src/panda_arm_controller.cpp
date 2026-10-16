// Panda Arm Controller — 支持 position + velocity + effort 三路命令接口的 MIT 模式控制器
//
// 关键改动 vs joint_trajectory_controller:
//   1. 去除 effort 轨迹的拒绝检查 — 允许 trajectory waypoints 携带 effort（前馈力矩）
//   2. effort 命令直接写轨迹采样值 state_desired_.effort（不走 PID 闭环）
//   3. 去除 use_closed_loop_pid_adapter_ / PID / tolerances / *_parameters.hpp 等复杂依赖
//   4. 去除 FollowJointTrajectory Action Server —— 本项目不用 MoveIt2，只用话题接口
//   5.去除 acceleration 接口 和默认检查，官方（humble版本）的joint_trajectory_controller
// 只允许position, velocity两两组合。本项目删去硬性检查，缺失时静默降级。可以同时使用position, velocity，effect

#include "panda_arm_hardware/panda_arm_controller.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace panda_arm_controller
{

// ──────────────────── 生命周期 ────────────────────

PandaArmController::PandaArmController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn PandaArmController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}
// ────────两个interface_configuration──────── 用从on_configure获得的变量做字符串拼接
controller_interface::InterfaceConfiguration
PandaArmController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (dof_ == 0)
  {
    return conf;
  }

  conf.names.reserve(dof_ * command_interface_types_.size()); //这个*是乘法
  for (const auto & joint_name : command_joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
PandaArmController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (dof_ == 0)
  {
    return conf;
  }

  conf.names.reserve(dof_ * state_interface_types_.size());
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

//————──从ros2_controllers.yaml 读取参数
controller_interface::CallbackReturn PandaArmController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // ── 读取 joints 参数 
  joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
  dof_ = joint_names_.size();

  // ── 读取 command_joints 参数 （yaml没有command_joints参数，"非空"才是特例）
  command_joint_names_ = auto_declare<std::vector<std::string>>("command_joints", {});
  if (command_joint_names_.empty())
  {
    command_joint_names_ = joint_names_;
  }

  // ── 读取 command_interfaces 参数 
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", {});
  
  // ── 读取 state_interfaces 参数 ──
  state_interface_types_ =
    auto_declare<std::vector<std::string>>("state_interfaces", {});

  // ── 检查 command_interface 类型，返回迭代器（这里就是返回地址，指针）── 
  auto contains = [](const std::vector<std::string> & list, const std::string & type) {
    return std::find(list.begin(), list.end(), type) != list.end();
  };
  has_position_command_interface_ =
    contains(command_interface_types_, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains(command_interface_types_, hardware_interface::HW_IF_VELOCITY);
  has_effort_command_interface_ =
    contains(command_interface_types_, hardware_interface::HW_IF_EFFORT);

  has_position_state_interface_ =
    contains(state_interface_types_, hardware_interface::HW_IF_POSITION);
  has_velocity_state_interface_ =
    contains(state_interface_types_, hardware_interface::HW_IF_VELOCITY);
  has_effort_state_interface_ =
    contains(state_interface_types_, hardware_interface::HW_IF_EFFORT);

  RCLCPP_INFO(
    logger,
    "Command: [pos=%d, vel=%d, eff=%d]  State: [pos=%d, vel=%d, eff=%d]",
    has_position_command_interface_, has_velocity_command_interface_,
    has_effort_command_interface_,
    has_position_state_interface_, has_velocity_state_interface_,
    has_effort_state_interface_);

  // ── 预分配 interface 存储 ──
  for (auto & itf : joint_command_interface_)
  {
    itf.reserve(dof_);
  }
  for (auto & itf : joint_state_interface_)
  {
    itf.reserve(dof_);
  }

  // ── 可选参数 ──
  allow_partial_joints_goal_ =
    auto_declare<bool>("allow_partial_joints_goal", false);//是否允许轨迹只包含部分关节
  open_loop_control_ =
    auto_declare<bool>("open_loop_control", false); //false表示插值时把实际位置当作轨迹起点

  // ── hold_position_msg ──
  init_hold_position_msg();

  // ── 话题订阅 ──
  joint_command_subscriber_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg) {
      topic_callback(msg);
    });

  // ── 重力补偿：读参数 + 订阅 robot_description ──
  robot_description_topic_ =
    auto_declare<std::string>("robot_description_topic", "/robot_description");

  gravity_scale_ = auto_declare<double>("gravity_scale", 0.0);
  gravity_sign_ = auto_declare<std::vector<double>>("gravity_sign", {});
  if (!gravity_sign_.empty() && gravity_sign_.size() != dof_)
  {
    RCLCPP_ERROR(
      logger, "'gravity_sign' size (%zu) != 'joints' size (%zu).",
      gravity_sign_.size(), dof_);
    return controller_interface::CallbackReturn::ERROR;
  }
  tau_ff_.assign(dof_, 0.0);

  auto robot_description_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  robot_description_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    robot_description_topic_, robot_description_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      on_robot_description(msg);
    });

  if (gravity_scale_ == 0.0)
  {
    RCLCPP_INFO(
      logger, "重力补偿: gravity_scale=0 → 关闭（effort 命令恒为轨迹自带值）");
  }
  else
  {
    RCLCPP_INFO(
      logger, "重力补偿: gravity_scale=%.3f，等待 %s 以建立模型 ...",
      gravity_scale_, robot_description_topic_.c_str());//std::string是对象，内部才是char*，.c_str()会返回 const char*
  }

  // ── MIT 律的 PD 项：用于仿真，如果是实物计算结果是0 ──
  //
  //   为了模仿电机，控制器在这里根据kpkd算力矩，Gazebo 只提供 effort 命令接口
  // 实物【不要写这两个参数】→ 取默认全 0 → update() 里那两项恒为 0，
  //   effort 只写重力前馈，PD 仍由电机板完成，行为与今天完全一致。
  kp_ = auto_declare<std::vector<double>>("kp", std::vector<double>(dof_, 0.0));
  kd_ = auto_declare<std::vector<double>>("kd", std::vector<double>(dof_, 0.0));
  effort_limit_ = auto_declare<std::vector<double>>(
    "effort_limit", std::vector<double>(dof_, std::numeric_limits<double>::infinity()));

  if (kp_.size() != dof_ || kd_.size() != dof_ || effort_limit_.size() != dof_)
  {
    RCLCPP_ERROR(
      logger,
      "'kp'/'kd'/'effort_limit' 的长度必须都等于 'joints' 的长度 (%zu)，实际为 %zu/%zu/%zu",
      dof_, kp_.size(), kd_.size(), effort_limit_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // pd_by_controller_ 真假表示PD是否需要控制器计算
  pd_by_controller_ = has_effort_command_interface_ && !has_position_command_interface_;

  const bool any_gain =
    std::any_of(kp_.begin(), kp_.end(), [](double v) { return v != 0.0; }) ||
    std::any_of(kd_.begin(), kd_.end(), [](double v) { return v != 0.0; });
  if (has_position_command_interface_ && any_gain)
  {
    RCLCPP_WARN(
      logger,
      "检测到 position 命令接口（PD 由下游电机板负责）→ 本控制器【忽略】kp/kd，"
      "以免与板子的 kp/kd 叠加导致振荡。若要让控制器算 PD，"
      "请把 command_interfaces 收成只剩 effort。");
  }
  RCLCPP_INFO(
    logger, "MIT 律: PD 由%s计算；kp[0]=%.2f kd[0]=%.3f effort_limit[0]=%.2f（其余关节同理）",
    pd_by_controller_ ? "本控制器" : "下游（电机板）",
    kp_[0], kd_[0], effort_limit_[0]);

  // ── 预分配 JointTrajectoryPoint ──
  auto resize_point = [&](JointTrajectoryPoint & p) {
    p.positions.resize(dof_, 0.0);
    if (has_velocity_state_interface_ || has_velocity_command_interface_)
    {
      p.velocities.resize(dof_, 0.0);
    }
    if (has_effort_state_interface_ || has_effort_command_interface_)
    {
      p.effort.resize(dof_, 0.0);
    }
  };
  resize_point(state_current_); //当前实际状态（插值起点）
  resize_point(state_desired_); //期望状态
  resize_point(last_commanded_state_); //上一次下发的命令，open_loop_control_=true时用

  RCLCPP_INFO(logger, "PandaArmController configured with %zu joints.", dof_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PandaArmController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // ── 按顺序收集 command interfaces ──
  // 顺序: position, velocity, effort
  const std::vector<std::string> cmd_order = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT
  };

  // 防御：槽位数必须容得下 cmd_order 。
  // 若将来给 cmd_order 加了项却忘了同步 NUM_INTERFACE_SLOTS，
  // 这里给出明确报错，而不是下面 joint_command_interface_[idx] 越界写坏内存。
  if (cmd_order.size() > NUM_INTERFACE_SLOTS)
  {
    RCLCPP_ERROR(logger, "cmd_order (%zu) 超过 joint_command_interface_ 槽位数 (%zu)",
                 cmd_order.size(), NUM_INTERFACE_SLOTS);
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t idx = 0; idx < cmd_order.size(); ++idx)
  {
    const auto & itf_name = cmd_order[idx];
    if (!std::any_of(command_interface_types_.begin(), command_interface_types_.end(),
                     [&](const auto & s) { return s == itf_name; }))
    {
      continue;  // 这个接口类型不在配置中，跳过这次的大for
    }

    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, command_joint_names_, itf_name,
          joint_command_interface_[idx]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.",
        dof_, itf_name.c_str(), joint_command_interface_[idx].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // ── 按顺序收集 state interfaces ──
  const std::vector<std::string> state_order = {
    hardware_interface::HW_IF_POSITION,
    hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_EFFORT
  };

  if (state_order.size() > NUM_INTERFACE_SLOTS)
  {
    RCLCPP_ERROR(logger, "state_order (%zu) 超过 joint_state_interface_ 槽位数 (%zu)",
                 state_order.size(), NUM_INTERFACE_SLOTS);
    return controller_interface::CallbackReturn::ERROR;
  }

  for (size_t idx = 0; idx < state_order.size(); ++idx)
  {
    const auto & itf_name = state_order[idx];
    if (!std::any_of(state_interface_types_.begin(), state_interface_types_.end(),
                     [&](const auto & s) { return s == itf_name; }))
    {
      continue;
    }

    if (!controller_interface::get_ordered_interfaces(
          state_interfaces_, joint_names_, itf_name,
          joint_state_interface_[idx]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' state interfaces, got %zu.",
        dof_, itf_name.c_str(), joint_state_interface_[idx].size());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // ── 初始化轨迹对象 ──
  traj_ptr_ = std::make_shared<Trajectory>();
  traj_msg_buffer_.writeFromNonRT(std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());
  subscriber_is_active_ = true;

  // ── 从硬件读取当前状态 ──
  read_state_from_state_interfaces(state_current_);
  last_commanded_state_ = state_current_;

  // ── 启动时保持当前位置 ──（set_hold_position() 内部已把 rt_is_holding_ 置 true）
  add_new_trajectory_msg(set_hold_position());

  RCLCPP_INFO(logger, "控制器on_activate完成");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PandaArmController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // 把命令清零
  for (size_t i = 0; i < dof_; ++i)
  {
    if (has_position_command_interface_ && !joint_command_interface_[IDX_POSITION].empty())
    {
      joint_command_interface_[IDX_POSITION][i].get().set_value(
        joint_command_interface_[IDX_POSITION][i].get().get_value());//位置保持不变
    }
    if (has_velocity_command_interface_ && !joint_command_interface_[IDX_VELOCITY].empty())
    {
      joint_command_interface_[IDX_VELOCITY][i].get().set_value(0.0);
    }
    if (has_effort_command_interface_ && !joint_command_interface_[IDX_EFFORT].empty())
    {
      joint_command_interface_[IDX_EFFORT][i].get().set_value(0.0);
    }
  }

  for (auto & itf : joint_command_interface_) itf.clear();
  for (auto & itf : joint_state_interface_) itf.clear();

  release_interfaces();
  subscriber_is_active_ = false;
  traj_ptr_.reset();

  RCLCPP_INFO(logger, "控制器on_deactivated完成");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PandaArmController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PandaArmController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool PandaArmController::reset()
{
  subscriber_is_active_ = false;
  joint_command_subscriber_.reset();
  // 只释放订阅，不清 gravity_：模型是只读的、且建立一次成本高，
  // 重新 configure 时新订阅会再次收到 transient_local 的 robot_description。
  robot_description_sub_.reset();
  traj_ptr_.reset();
  return true;
}

// ──────────────────── update() ────────────────────
//理论上只能写实时安全代码，因为任何不可预测耗时的操作都可能会让控制周期抖动
controller_interface::return_type PandaArmController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /* period */) // MIT 模式不需要 period
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }

  // ── 写接口的 lambda ──
  auto assign_interface_from_point =
    [&](auto & joint_interface, const std::vector<double> & traj_pt) {
      for (size_t i = 0; i < dof_; ++i)
      {
        joint_interface[i].get().set_value(traj_pt[i]);
      }
    };  

  // ── 读取当前状态 ──
  read_state_from_state_interfaces(state_current_); //把 joint_state_interface_ 写入 state_current_

  // ── 重力前馈 τ_ff = gravity_scale × gravity_sign ⊙ g(q) ──
  //
  // 只补静态重力项：稳态 (q̇ = q̈ = 0) 下 M q̈ + C q̇ + g = τ 退化为 g(q) = τ，
  // 「下垂 / 抬不动 / 饱和」只由 g(q) 决定；惯性项/科氏项只影响瞬态，
  // 在当前的限速（2 rad/s）下约占重力的 10~25%。详见 gravity_compensation.hpp。
  //
  // 未就绪（scale=0、URDF 没到、解析失败）→ gravity_ok=false → effort 只写轨迹自带值。
  const bool gravity_ok =
    gravity_.ready() && gravity_.compute(state_current_.positions, tau_ff_); //重力补偿值在这里计算
  if (!gravity_ok && gravity_scale_ > 0.0)
  {
    warn_gravity_not_ready();
  }

  // ── 检查是否有新的轨迹（点）消息 ──  traj_ptr_是指向 Trajectory 类的共享指针
  // current_external_msg是当前执行的轨迹点，new_external_msg是下一刻（如果有）的轨迹点
  auto current_external_msg = traj_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_buffer_.readFromRT(); //new_external_msg 指向的还是指针
  if (current_external_msg != *new_external_msg)  //比较的是地址变了没有（有新轨迹到达）
  {
    fill_partial_goal(*new_external_msg); //如果轨迹只包含部分关节，用当前位置补全其余关节
    sort_to_local_joint_order(*new_external_msg);//把关节顺序重排成控制器内部的固定顺序
    traj_ptr_->update(*new_external_msg);//把这条轨迹传入采样函数，重置采样状态
    // DEBUG
    static int upd_count = 0;
    if (++upd_count % 50 == 1) {
      RCLCPP_INFO(get_node()->get_logger(),
        "update: new traj, pos[0]=%.3f", (*new_external_msg)->points[0].positions[0]);
    }
  }

  // ── 如果有活跃轨迹，采样并写命令 ──
  if (has_active_trajectory())
  {
    if (!traj_ptr_->is_sampled_already())
    {
      if (open_loop_control_)
      {
        traj_ptr_->set_point_before_trajectory_msg(time, last_commanded_state_);
      }
      else
      {
        traj_ptr_->set_point_before_trajectory_msg(time, state_current_);
      }
    }

    TrajectoryPointConstIter start_itr, end_itr;
    const bool valid_point = traj_ptr_->sample(  //sample()是trajectory.cpp的方法，内部调用插值算法，把结果写进state_desired_
      time, state_desired_, start_itr, end_itr);

    if (valid_point)
    {
      // DEBUG
      static int samp_count = 0;
      if (++samp_count % 50 == 1) {
        RCLCPP_INFO(get_node()->get_logger(),
          "sample: cur[0]=%.3f des[0]=%.3f", state_current_.positions[0], state_desired_.positions[0]);
      }

      // ── 写 command interfaces ──
      // ★ 关键：position / velocity / effort 全部直接写采样值
      if (has_position_command_interface_)
      {
        assign_interface_from_point(
          joint_command_interface_[IDX_POSITION], state_desired_.positions);
      }
      if (has_velocity_command_interface_)
      {
        assign_interface_from_point(
          joint_command_interface_[IDX_VELOCITY], state_desired_.velocities);
      }
      // ★ effort = 重力前馈 g(q) + 轨迹自带的前馈 + PD 项（可选），最后限幅。
      //
      //    PD ：只有 pd_by_controller_ 为真才会计算，且有双层防护：
      //     控制器的kpkd默认值为0，走实物的时候控制器配置文件没有kpkd，使用默认值0，
      //     实物即便通过pd_by_controller_验证，算出来的还是0

      if (has_effort_command_interface_)
      {
        for (size_t i = 0; i < dof_; ++i)
        {
          // yaml 配置了effort接口但轨迹点没带 effort时就会触发
          const double traj_eff =
            state_desired_.effort.empty() ? 0.0 : state_desired_.effort[i];

          double pd = 0.0;
          if (pd_by_controller_)
          {
            // 与电机板同一条律：τ = τ_ff + kp(p_des − p) + kd(v_des − v)
            // 反馈量取【实测】值（state_current_），不是期望值。
            const double p_err =
              state_desired_.positions.empty()
                ? 0.0
                : state_desired_.positions[i] - state_current_.positions[i];
            const double v_err =
              (state_desired_.velocities.empty() || state_current_.velocities.empty())
                ? 0.0
                : state_desired_.velocities[i] - state_current_.velocities[i];
            pd = kp_[i] * p_err + kd_[i] * v_err;
          }

          // 总力矩限幅 ：仿真里靠它复现电机 1.5 Nm 堵转上限（否则仿真比实物更有劲，
          // joint_2 饱和的现象复现不出来）；
          // 实物 effort_limit_ 为 inf ，等于不限，电机自己有上限
          double tau = (gravity_ok ? tau_ff_[i] : 0.0) + traj_eff + pd;
          tau = std::clamp(tau, -effort_limit_[i], effort_limit_[i]);
          joint_command_interface_[IDX_EFFORT][i].get().set_value(tau);
        }
      }

      last_commanded_state_ = state_desired_;
    }

    // ── 轨迹走完 → 切到"保持当前实际位置" ──
    // 为什么不用"重复末点"（上游 joint_trajectory_controller 的语义）：
    //   本控制器的下游是 MIT 模式的 J288。末点的【非零 velocity】会被
    //   PandaArmHardware::write() 当成主动速度指令持续下发，绕过"按误差补速度 /
    //   到位即停"的兜底逻辑；末点的【非零 effort】在到位后也会被硬件强制清零
    //   （eff_cmd = 0.0），保留没有意义。
    //   改用【当前实际位置】当目标：pos_error ≈ 0 → 硬件立即进入保持态
    //   （HOLD_KP/HOLD_KD），不会产生额外的"赶路"动作。
    //
    // 用 rt_is_holding_ 做一次性守卫：轨迹走完后 sample() 每个周期都会返回
    //   end_itr == end()，不加守卫就会每周期都调 writeFromNonRT
    //   （实时线程里加锁）。set_hold_position() 内部会把 rt_is_holding_ 置 true，
    //   新轨迹到达时由 topic_callback 置回 false，从而"每条轨迹只触发一次"。
    if (!rt_is_holding_ && (end_itr == traj_ptr_->end() || !valid_point))
    {
      traj_msg_buffer_.writeFromNonRT(set_hold_position());
    }
  }

  return controller_interface::return_type::OK;
}

// ──────────────────── 状态读写 ────────────────────

void PandaArmController::read_state_from_state_interfaces(JointTrajectoryPoint & state)
{
  auto assign_point = [&](std::vector<double> & traj_pt, const auto & joint_itf) {
    for (size_t i = 0; i < dof_; ++i)
    {
      traj_pt[i] = joint_itf[i].get().get_value();
    }
  };

  if (has_position_state_interface_ && !joint_state_interface_[IDX_POSITION].empty())
  {
    assign_point(state.positions, joint_state_interface_[IDX_POSITION]);
  }
  if (has_velocity_state_interface_ && !joint_state_interface_[IDX_VELOCITY].empty())
  {
    assign_point(state.velocities, joint_state_interface_[IDX_VELOCITY]);
  }
  else
  {
    state.velocities.clear();
  }
  if (has_effort_state_interface_ && !joint_state_interface_[IDX_EFFORT].empty())
  {
    assign_point(state.effort, joint_state_interface_[IDX_EFFORT]);
  }
  else
  {
    state.effort.clear();
  }
}

// ──────────────────── 轨迹管理 ────────────────────

bool PandaArmController::has_active_trajectory() const
{
  return traj_ptr_ != nullptr && traj_ptr_->has_trajectory_msg();
}

void PandaArmController::add_new_trajectory_msg(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg)
{
  traj_msg_buffer_.writeFromNonRT(traj_msg);
}

// ──────────────────── 轨迹验证 ────────────────────
// ★★★ 关键改动：去掉了 effort 字段的拒绝逻辑

bool PandaArmController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  auto logger = get_node()->get_logger();

  if (!allow_partial_joints_goal_ && trajectory.joint_names.size() != dof_) // partial:部分的
  {
    RCLCPP_ERROR(
      logger, "Joints on incoming trajectory don't match controller joints.");
    return false;
  }

  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(logger, "Empty joint names on incoming trajectory.");
    return false;
  }

  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(logger, "Empty trajectory received.");
    return false;
  }

  // 检查起点时间
  const auto traj_start = static_cast<rclcpp::Time>(trajectory.header.stamp);
  if (traj_start.seconds() != 0.0)
  {
    auto traj_end = traj_start + trajectory.points.back().time_from_start;
    if (traj_end < get_node()->now())  //因为此时轨迹点还未开始执行，traj_end 在未来
    {
      RCLCPP_ERROR(logger, "Received trajectory that ends in the past.");
      return false;
    }
  }

  // 检查关节名
  for (const auto & name : trajectory.joint_names)
  {
    if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end())
    {
      RCLCPP_ERROR(logger, "Joint '%s' not in controller.", name.c_str());
      return false;
    }
  }

  // 检查时间单调递增，每次触发回调从头检查一次轨迹点，非实时性能预算充足所以可以从头检查
  rclcpp::Duration prev_time(0, 0); //保存"上一个点的时间"
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    const auto & pt = trajectory.points[i];
    rclcpp::Duration pt_time = pt.time_from_start;

    if (i > 0 && pt_time <= prev_time)
    {
      RCLCPP_ERROR(logger, "Trajectory time not strictly increasing at point %zu.", i);
      return false;
    }
    prev_time = pt_time;

    const size_t jc = trajectory.joint_names.size();

    // 必须有 position
    if (pt.positions.size() != jc)
    {
      RCLCPP_ERROR(logger, "Mismatch: positions size %zu vs joints %zu at point %zu.",
                   pt.positions.size(), jc, i);
      return false;
    }

    // velocity / acceleration / effort 如果提供了，关节数必须匹配
    if (!pt.velocities.empty() && pt.velocities.size() != jc)
    {
      RCLCPP_ERROR(logger, "Mismatch: velocities size %zu at point %zu.",
                   pt.velocities.size(), i);
      return false;
    }
    if (!pt.accelerations.empty() && pt.accelerations.size() != jc)
    {
      RCLCPP_ERROR(logger, "Mismatch: accelerations size %zu at point %zu.",
                   pt.accelerations.size(), i);
      return false;
    }
    if (!pt.effort.empty() && pt.effort.size() != jc)
    {
      RCLCPP_ERROR(logger, "Mismatch: effort size %zu at point %zu.",
                   pt.effort.size(), i);
      return false;
    }

    // ★★★ 关键改动：不再拒绝 effort 字段！原版在此处 return false
  }

  return true;
}

// ──────────────────── 保持位置 ────────────────────

void PandaArmController::init_hold_position_msg() //初始化位置保持工具 on_config里调用
{
  hold_position_msg_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  hold_position_msg_ptr_->header.stamp =
    rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  hold_position_msg_ptr_->joint_names = joint_names_;
  hold_position_msg_ptr_->points.resize(1);
  hold_position_msg_ptr_->points[0].positions.resize(dof_, 0.0);
  hold_position_msg_ptr_->points[0].velocities.resize(dof_, 0.0);
  hold_position_msg_ptr_->points[0].effort.resize(dof_, 0.0);
  // accelerations 保持 size == 0（语义："不提供加速度"），但先把容量开好。
  // 目的：避免 Trajectory::sample() 末尾分支里的
  //       output_state.accelerations.resize(...) 在【实时线程】里发生堆分配
  //       （本对象是长期复用的）。
  // reserve 只改 capacity 、不改 size，所以语义不变。
  hold_position_msg_ptr_->points[0].accelerations.clear();
  hold_position_msg_ptr_->points[0].accelerations.reserve(dof_);
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
PandaArmController::set_hold_position()
{
  auto & pt = hold_position_msg_ptr_->points[0];

  pt.positions = state_current_.positions;
  // 用 assign 而非 for + 下标：既简洁，又能自愈 size 不一致
  // （原写法假定 velocities/effort 的 size 一定 >= dof_，否则越界）
  pt.velocities.assign(dof_, 0.0);
  pt.effort.assign(dof_, 0.0);
  // 清掉可能残留的加速度：本对象长期复用，若之前被整体赋值过带加速度的末点，
  // 这里不清理就会残留脏值。
  pt.accelerations.clear();

  rt_is_holding_ = true;
  return hold_position_msg_ptr_;
}

// ──────────────────── 补全 & 排序 ────────────────────

void PandaArmController::fill_partial_goal(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) const
{
  if (dof_ == traj_msg->joint_names.size())
  {
    return;
  }

  for (size_t i = 0; i < dof_; ++i)
  {
    if (std::find(traj_msg->joint_names.begin(), traj_msg->joint_names.end(),
                  joint_names_[i]) != traj_msg->joint_names.end())  //找到则返回指向找到元素的迭代器
    {
      continue;  // 已存在
    }

    traj_msg->joint_names.push_back(joint_names_[i]);
    for (auto & pt : traj_msg->points)
    {
      if (!pt.positions.empty())
      {
        pt.positions.push_back(state_current_.positions[i]);
      }
      if (!pt.velocities.empty())
      {
        pt.velocities.push_back(0.0);
      }
      if (!pt.accelerations.empty())
      {
        pt.accelerations.push_back(0.0);
      }
      if (!pt.effort.empty())
      {
        pt.effort.push_back(0.0);
      }
    }
  }
}

void PandaArmController::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg)
{
  std::vector<size_t> map = mapping(traj_msg->joint_names, joint_names_);

  // ── 恒等映射早退 ──
  // 轨迹的关节顺序与控制器一致时，下面的重排只是无意义的自我拷贝（out[i] = in[i]）：
  // 既白费 CPU，又会在实时线程里对每个点的 4 个字段各做一次堆分配（4N 次 malloc/free）。
  // 注：关节名不匹配时 mapping() 返回空 vector，此时 map.size() 与 joint_names.size() 不等，
  //     不会被误判为恒等，仍会走下面的重排逻辑（保持原有行为）。
  bool identity = (map.size() == traj_msg->joint_names.size());
  for (size_t i = 0; identity && i < map.size(); ++i)
  {
    if (map[i] != i)
    {
      identity = false;
    }
  }
  if (identity)
  {
    return;
  }

  auto remap = [&](const std::vector<double> & in) -> std::vector<double> {
    if (in.empty()) return in;
    std::vector<double> out(dof_, 0.0);
    for (size_t i = 0; i < map.size(); ++i)
    {
      out[map[i]] = in[i];
    }
    return out;
  };

  for (auto & pt : traj_msg->points)
  {
    pt.positions = remap(pt.positions);
    pt.velocities = remap(pt.velocities);
    pt.accelerations = remap(pt.accelerations);
    pt.effort = remap(pt.effort);
  }
}

// ──────────────────── 话题回调 ────────────────────

void PandaArmController::topic_callback(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
{
  if (!validate_trajectory_msg(*msg))
  {
    return;
  }

  if (subscriber_is_active_)
  {
    add_new_trajectory_msg(msg);
    rt_is_holding_ = false;
    // DEBUG
    static int cb_count = 0;
    if (++cb_count % 50 == 1) {
      RCLCPP_INFO(get_node()->get_logger(),
        "topic_callback: accepted msg, pos[0]=%.3f", msg->points[0].positions[0]);
    }
  }
}

// ──────────────────── 重力补偿 ────────────────────

void PandaArmController::on_robot_description(const std_msgs::msg::String::SharedPtr msg)
{
  auto logger = get_node()->get_logger();

  // 【非实时线程】。下面要解析 URDF、建 pinocchio 模型，都会分配内存，
  // 绝对不能放在 update() 里做 —— 这是本功能唯一"重"的地方，且只做一次。
  if (gravity_.ready())
  {
    if (msg->data != last_urdf_)
    {
      // 不重建：update() 可能正在实时线程里读模型，重建会造成数据竞争。
      // 正常运行不会发生（robot_state_publisher 只发一次 robot_description）。
      RCLCPP_WARN(
        logger, "收到与已加载模型不同的 %s，忽略（需重启节点才能应用新模型）",
        robot_description_topic_.c_str());
    }
    return;
  }

  panda_arm_hardware::GravityParams params;
  params.scale = gravity_scale_;
  params.sign = gravity_sign_;

  if (!gravity_.configure(msg->data, joint_names_, params))
  {
    RCLCPP_ERROR(
      logger,
      "重力模型建立失败 → 重力补偿保持关闭。请检查 URDF 里是否存在 %zu 个"
      " 单自由度关节、且名字与 ros2_controllers.yaml 的 'joints' 完全一致。",
      dof_);
    return;
  }

  last_urdf_ = msg->data;
  RCLCPP_INFO(
    logger, "重力模型已加载: nq=%d nv=%d, 关节=%zu, gravity_scale=%.3f",
    gravity_.nq(), gravity_.nv(), dof_, gravity_scale_);
}

void PandaArmController::warn_gravity_not_ready()
{
  // update() 跑在 200Hz 的实时线程上，日志本身会分配内存，所以必须限频。
  // 不用 RCLCPP_WARN_THROTTLE：那套宏内部有静态状态，实时线程里不合适。
  if (++warn_gravity_counter_ % 400 != 1)
  {
    return;
  }
  RCLCPP_WARN(
    get_node()->get_logger(),
    "gravity_scale=%.3f 但重力模型尚未就绪（未收到 %s）→ 本周期 effort 只写轨迹值",
    gravity_scale_, robot_description_topic_.c_str());
}

}  // namespace panda_arm_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  panda_arm_controller::PandaArmController, controller_interface::ControllerInterface)
