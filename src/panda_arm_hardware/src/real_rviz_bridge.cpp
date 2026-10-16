/**
 * real_rviz_bridge — Rviz2 滑块 → 真实机械臂桥接节点
 *
 * 功能：
 *   1. 订阅 /gui_joint_states（joint_state_publisher_gui 或 slider_gui_deg 发布，
 *      关节名 joint_1~joint_7）
 *   2. 将 JointState 转换为 JointTrajectory，发布给 PandaArmController
 *   3. 速率限制：并不是直接对速度裁剪，而是通过限制每帧步长来约束最大速度 
 *   4. 发出的轨迹点带position, velocity
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class RealRvizBridge : public rclcpp::Node
{
public:
  RealRvizBridge()
  : Node("real_rviz_bridge")
  {
    // ── 参数声明 ──
    this->declare_parameter<std::vector<std::string>>("joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"});
    this->declare_parameter<double>("publish_rate", 50.0);           // Hz
    this->declare_parameter<double>("max_velocity", 1.5);            // rad/s, 关节角速度上限
    this->declare_parameter<double>("step_duration", 0.1);           // 轨迹点时长（秒）
    this->declare_parameter<bool>("enabled", true);                  // 是否启用桥接

    joint_names_ = this->get_parameter("joint_names").as_string_array();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    step_duration_ = this->get_parameter("step_duration").as_double();
    enabled_ = this->get_parameter("enabled").as_bool();

    dof_ = joint_names_.size();
    // 上游（滑块）写入的原始目标：只由 joint_state_callback 写，限幅器绝不修改它。
    // 这样「还剩多少没走」这个误差不会丢 —— 上游即使只发一次消息，
    // 本节点也能继续逐帧逼近到目标（旧实现会把误差改写掉，只能靠上游重发）。
    slider_target_.resize(dof_, 0.0);
    // 限幅器状态：实际下发的目标，每帧朝 slider_target_ 推进一步
    cmd_target_.resize(dof_, 0.0);
    // 本帧各关节实际推进量（rad），用于换算成速度前馈
    advanced_.resize(dof_, 0.0);

    // ── 订阅 /gui_joint_states（滑块命令）──
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/gui_joint_states", rclcpp::SensorDataQoS(),
      std::bind(&RealRvizBridge::joint_state_callback, this, std::placeholders::_1));

    // ── 订阅 /joint_states（真实反馈），用于初始化目标基准 ──
    // 关键：slider_target_ / cmd_target_ 初值为 0，而电机实际（软件）角度通常不是 0
    //（单圈编码器每次上电圈数清零，软件角度是个新值）。
    // 若不以真实反馈初始化，首次拖动时目标（≈0）与电机实际位置相差很大，
    // 硬件层误差会远超 SLOW_ZONE(0.05) → 满速冲过去（表现为"猛冲一下"）。
    actual_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&RealRvizBridge::actual_state_callback, this, std::placeholders::_1));

    // ── 发布 JointTrajectory 给控制器 ──
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/panda_arm_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());

    // ── 定时器：以固定频率发送轨迹 ──
    // 这个周期同时也是速率限制的时间基准（见 timer_callback）
    auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      period,
      std::bind(&RealRvizBridge::timer_callback, this));

    // ── 参数动态更新回调 ──
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&RealRvizBridge::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "RealRvizBridge started: %zu joints, %.1f Hz, max_vel=%.2f rad/s",
      dof_, publish_rate_, max_velocity_);
    RCLCPP_INFO(this->get_logger(),
      "速率限制: max_velocity=%.2f rad/s ÷ publish_rate=%.1f Hz → 每帧最大 %.4f rad",
      max_velocity_, publish_rate_, max_velocity_ / publish_rate_);
    RCLCPP_INFO(this->get_logger(),
      "轨迹点: 自洽 (position, velocity)，time_from_start = %.4f s (= 1/publish_rate)",
      publish_rate_ > 0.0 ? 1.0 / publish_rate_ : 0.0);
  }

private:
  // ── 订阅回调：从 gui_joint_states 提取目标位置（原始值，不限幅）──
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!enabled_) return;

    // 按关节名匹配（处理乱序消息）
    for (size_t i = 0; i < dof_; i++) {
      for (size_t j = 0; j < msg->name.size(); j++) {
        if (msg->name[j] == joint_names_[i]) {
          if (j < msg->position.size()) {
            slider_target_[i] = msg->position[j];
          }
          break;
        }
      }
    }
  }

  // ── 真实反馈回调：用电机实际位置初始化目标基准（仅首次）──
  void actual_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (initialized_) {
      return;
    }

    // 按关节名匹配（处理乱序消息）
    size_t matched = 0;
    for (size_t i = 0; i < dof_; i++) {
      for (size_t j = 0; j < msg->name.size(); j++) {
        if (msg->name[j] == joint_names_[i]) {
          if (j < msg->position.size()) {
            slider_target_[i] = msg->position[j];
            cmd_target_[i] = msg->position[j];
            matched++;
          }
          break;
        }
      }
    }

    // 所有关节都匹配到才认为初始化完成，避免部分关节仍用 0 基准
    if (matched == dof_) {
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(),
        "已用真实反馈(/joint_states)初始化目标基准，避免首次拖动猛冲");
    }
  }

  // ── 定时器回调：速率限制 + 发送轨迹 ──
  void timer_callback()
  {
    if (!enabled_) return;

    // 未用真实反馈初始化前不发布轨迹：
    // 此时目标仍是初值 0，与电机实际位置不符，发布会导致电机满速猛冲。
    if (!initialized_) {
      return;
    }

    // ── 速率限制（坡度限制器）──
    // 时间基准必须是本节点【自己的发送周期】1/publish_rate（= 定时器周期），
    // 而不是 step_duration：
    //   step_duration  = 轨迹点应在多久内到达（给插值用的时间戳）
    //   1/publish_rate = 每帧时长 ← 限幅真正需要的是这个
    // 因为每帧最多推进 max_velocity × tick_period，所以
    // 有效速度上限恒等于 max_velocity (rad/s)，与上游发布频率无关。
    const double tick_period = 1.0 / publish_rate_;
    const double max_delta = max_velocity_ * tick_period;

    // 每帧朝上游原始目标 slider_target_ 逼近一步；被改的是 cmd_target_，
    // slider_target_ 保持上游原值，剩余误差因此不会丢失。
    // changed 的含义 =「尚未收敛，需要继续发」。
    // advanced_[i] = 本帧实际推进量（rad），下一步用它换算速度前馈。
    bool changed = false;
    for (size_t i = 0; i < dof_; i++) {
      const double delta = slider_target_[i] - cmd_target_[i];
      if (std::abs(delta) <= 1e-9) {
        advanced_[i] = 0.0;                    // 已到位：本帧不动
        continue;
      }
      changed = true;
      // 巡航段 |delta| > max_delta → 走满 max_delta（= max_velocity × 周期）；
      // 接近段 |delta| <= max_delta → 一步到位，速率随误差自然衰减。
      const double step = (std::abs(delta) > max_delta)
                            ? std::copysign(max_delta, delta)
                            : delta;
      cmd_target_[i] += step;
      advanced_[i] = step;
    }
    if (!changed) return;

    // ── 构造 JointTrajectory ──
    auto traj_msg = trajectory_msgs::msg::JointTrajectory();
    //  显式置 .stamp = 0，含义是起点由控制器按"收到消息的时刻"自己锚定
    //    （Trajectory::sample 里 trajectory_start_time_ = sample_time）。
    //    刻意不填 this->now() 的三个理由：
    //      1. validate_trajectory_msg 会跳过"轨迹已过期"检查（stamp == 0 时不做时间比较）；
    //      2. 插值区间精确等于 point.time_from_start，不被网络/回调延迟侵蚀
    //         （若填 now()，区间变成 time_from_start 减去延迟；延迟超过它时会直接跳变到目标点）；
    //      3. 起点由控制器自己的时钟决定，换到仿真时间(use_sim_time)时本节点无需改动。
    traj_msg.header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    traj_msg.joint_names = joint_names_;

    // ── 轨迹点：(position, velocity) 自洽 ──
    // 本帧从 A 走到了 A+Δ，速率天然就是 Δ / 周期：
    //   positions  = cmd_target_        （本帧的新位置）
    //   velocities = advanced_ / 周期   （与之自洽的速率）
    // 两者是同一事实的两种表达 → 控制器那段的插值退化成直线，
    // 不再出现「填 0 时被强加的：到点必须减速到 0」的假动作。
    // 速度随误差自然成形：巡航段 = max_velocity，接近段衰减，到位为 0。
    // 因为 |advanced_| <= max_delta，所以 |velocities| <= max_velocity 恒成立。
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = cmd_target_;
    point.velocities.resize(dof_);
    for (size_t i = 0; i < dof_; ++i) {
      point.velocities[i] = advanced_[i] / tick_period;
    }
    point.effort.resize(dof_, 0.0);
    // 时间戳 = 发布周期：这一段就是「以该速率走完这一帧」。
    //（不再用 step_duration：它与发布周期错配会让段内产生多余的加减速。）
    point.time_from_start = rclcpp::Duration::from_seconds(tick_period);
    traj_msg.points.push_back(point);

    trajectory_pub_->publish(traj_msg);
  }

  // ── 参数动态更新 ──  （绑在回调函数里，ros2 param set设置参数时自动触发）
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      if (param.get_name() == "enabled") {
        enabled_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Bridge %s", enabled_ ? "ENABLED" : "DISABLED");
      } else if (param.get_name() == "max_velocity") {
        max_velocity_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "max_velocity = %.3f rad/s", max_velocity_);
      } else if (param.get_name() == "step_duration") {
        // 已废弃：轨迹点时间戳现在固定为发布周期（1/publish_rate），
        // step_duration 不再参与任何计算。保留声明只为兼容旧参数文件。
        RCLCPP_WARN(this->get_logger(),
          "step_duration=%.3f 已废弃、不再生效（轨迹点时长 = 1/publish_rate）。",
          param.as_double());
      }
    }
    return result;
  }

  // ── 成员变量 ──
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  // 订阅真实反馈 /joint_states，用于把目标基准初始化为电机实际位置
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr actual_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_;
  size_t dof_ = 0;

  double publish_rate_ = 50.0;
  double max_velocity_ = 1.5;
  double step_duration_ = 0.1;
  bool enabled_ = true;

  std::vector<double> slider_target_;   // 上游（滑块）写入的原始目标，限幅器不修改
  std::vector<double> cmd_target_;      // 限幅器状态：实际下发的目标
  std::vector<double> advanced_;        // 本帧各关节实际推进量（rad），用于速度前馈
  // 是否已用真实反馈初始化目标基准。
  // 未初始化时禁止发布轨迹：否则会以初值 0 为基准、与电机实际位置产生大误差 → 猛冲。
  bool initialized_ = false;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealRvizBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
