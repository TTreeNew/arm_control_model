// Panda Arm Controller — 支持 position + velocity + effort 三路命令接口的 MIT 模式控制器
//
// 基于 joint_trajectory_controller 精简而来，关键改动：
//   1. 允许轨迹携带 effort（前馈力矩）字段
//   2. effort = 重力前馈 g(q) + 轨迹自带的 effort，不做 PID 闭环
//   3. 去除 use_closed_loop_pid_adapter_ / tolerances / *_parameters.hpp 等复杂依赖
//   4. 去除 FollowJointTrajectory Action Server —— 本项目不用 MoveIt2，
//      轨迹只通过 ~/joint_trajectory 话题下发
//
// 重力补偿（第 2 条的 g(q)）：
//   订阅 /robot_description（transient_local）取机器人模型，用 pinocchio 的
//   computeGeneralizedGravity 按【实测】位形算出静态重力力矩，作为前馈写进
//   effort 命令接口。只补 g(q)，不补惯性项 M(q)q̈ / 科氏项 C(q,q̇)q̇ ——
//   后两项只在运动时有值，不影响稳态。实现见 gravity_compensation.hpp。

#ifndef PANDA_ARM_CONTROLLER__PANDA_ARM_CONTROLLER_HPP_
#define PANDA_ARM_CONTROLLER__PANDA_ARM_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "panda_arm_hardware/gravity_compensation.hpp"
#include "panda_arm_hardware/trajectory.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace panda_arm_controller
{

using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

class PandaArmController : public controller_interface::ControllerInterface
{
public:
  PandaArmController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ─── 轨迹验证 ───
  bool validate_trajectory_msg(const trajectory_msgs::msg::JointTrajectory & trajectory) const;

  // ─── 轨迹替换 / 插入 ───
  void add_new_trajectory_msg(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg);
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> set_hold_position();
  void init_hold_position_msg();

  // ─── 排序 / 补全 ───
  void fill_partial_goal(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg) const;
  void sort_to_local_joint_order(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> traj_msg);

  // ─── 话题回调 ───
  void topic_callback(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg);

  // ─── 重力补偿 ───
  // /robot_description 回调：非实时线程，在这里解析 URDF 建模型
  void on_robot_description(const std_msgs::msg::String::SharedPtr msg);
  // 模型未就绪但 gravity_scale > 0 时的限频告警（避免实时线程里刷日志）
  void warn_gravity_not_ready();

  // ─── 状态读写 ───
  void read_state_from_state_interfaces(JointTrajectoryPoint & state);

  bool has_active_trajectory() const;
  bool reset();

  // ─── 参数 ───
  size_t dof_ = 0;
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  bool has_position_command_interface_ = false;
  bool has_velocity_command_interface_ = false;
  bool has_effort_command_interface_ = false;
  bool has_position_state_interface_ = false;
  bool has_velocity_state_interface_ = false;
  bool has_effort_state_interface_ = false;

  // ─── 轨迹对象 ───
  std::shared_ptr<Trajectory> traj_ptr_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>>
    traj_msg_buffer_;

  // ─── 状态 & 命令 ───
  JointTrajectoryPoint state_current_;
  JointTrajectoryPoint state_desired_;
  JointTrajectoryPoint last_commanded_state_;

  // 命令/状态接口的槽位数。
  // ⚠️ on_activate() 按 cmd_order / state_order 的【顺序】把接口填入 [0..N-1]：
  //    槽位数必须 >= 对应 order 的长度，且下面 IDX_* 的取值必须与该顺序一一对应。
  //    若调整了 order 的顺序而不同步改 IDX_*，会【静默错位】（编译期不报错）。
  //    加速度不在其中：J288 协议无该字段，硬件也未导出该接口。
  static constexpr size_t NUM_INTERFACE_SLOTS = 3;

  // 命令接口索引（槽位含义见 NUM_INTERFACE_SLOTS 的注释）
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_command_interface_[NUM_INTERFACE_SLOTS];

  // 状态接口索引
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_state_interface_[NUM_INTERFACE_SLOTS];

  static constexpr size_t IDX_POSITION = 0;
  static constexpr size_t IDX_VELOCITY = 1;
  static constexpr size_t IDX_EFFORT = 2;

  // ─── 话题订阅 ───
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
  bool subscriber_is_active_ = false;

  // ─── 保持位置消息 ───
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> hold_position_msg_ptr_;
  // 是否正处于"保持"态（update() 用它做"轨迹走完只切一次"的一次性守卫）。
  // 跨线程：topic_callback() 在 executor 线程置 false，update() 在实时线程读取并置 true，
  // 因此必须是 atomic（否则是数据竞争）。
  std::atomic<bool> rt_is_holding_ = false;

  // ─── 重力补偿 ───
  // 前馈力矩 τ_ff = gravity_scale × gravity_sign ⊙ g(q)，只补静态重力项。
  // 需要时也只读 state_current_.positions（硬件导出的 position state interface），
  // 不订阅 /joint_states。
  panda_arm_hardware::GravityCompensation gravity_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  std::vector<double> tau_ff_;          // 与 joint_names_ 同序，on_configure 预分配
  std::string robot_description_topic_ = "/robot_description";
  std::string last_urdf_;               // 已建模型所用的 URDF，用于识别"内容变了"
  double gravity_scale_ = 0.0;
  std::vector<double> gravity_sign_;    // 空 = 全部 +1
  int warn_gravity_counter_ = 0;

  // ─── MIT 律的 PD 项 ───
  // 由谁算：见 pd_by_controller_（on_configure 里判定一次）。
  //   实物：ros2_controllers.yaml 【不写】kp/kd → 取默认全 0 → update() 里那两项
  //         恒等于 0，effort 只写重力前馈，PD 仍由电机板完成
  //         （motor_params.yaml → URDF <param> → PandaArmHardware），
  //         行为与"没有这段代码"完全一致。
  //   仿真：Gazebo 里没有电机板，只有 effort 命令接口，没人算 PD
  //         → 由本控制器自己闭环，数值从 sim_controllers.yaml 给。
  // ⚠️ 双算会振荡：controller 的 kp/kd 与板子的 kp/kd 必须只有一个非零。
  std::vector<double> kp_;
  std::vector<double> kd_;

  // 总力矩限幅 (Nm)。
  //   仿真填 1.5 —— 复现电机堵转上限（见 motor_params.yaml 顶部注释里的天花板），
  //   否则仿真里的机械臂会比实物"更有劲"，joint_2 饱和的现象复现不出来。
  //   实物留默认 inf —— 板子内部有自己的保护，不在这里重复限制。
  std::vector<double> effort_limit_;

  // PD 由谁算：true = 本控制器；false = 下游（电机板）。
  // 判据是【硬件有没有 position 命令接口】，这条判据不是猜的：
  //   "有 position 命令接口"本身就意味着"下游需要 p_des 才能做 MIT 闭环"
  //   → 两条信息是同一件事的两面，所以不可能出现"两边都算"。
  bool pd_by_controller_ = false;

  // ─── 辅助 ───
  bool allow_partial_joints_goal_ = false;
  bool open_loop_control_ = false;
};

}  // namespace panda_arm_controller

#endif  // PANDA_ARM_CONTROLLER__PANDA_ARM_CONTROLLER_HPP_
