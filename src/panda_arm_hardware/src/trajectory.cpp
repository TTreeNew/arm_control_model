// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "panda_arm_hardware/trajectory.hpp"

#include <cmath>

#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace panda_arm_controller
{
// 创建"轨迹执行引擎"的空壳：此时 trajectory_msg_ 为空，
// has_trajectory_msg() 返回 false，控制器的 update() 会因此跳过采样。
Trajectory::Trajectory()
: trajectory_start_time_(0),
  time_before_traj_msg_(0)
{
} 
// 保存当前关节状态为"轨迹之前的点"
void Trajectory::set_point_before_trajectory_msg(
  const rclcpp::Time & current_time,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_point)
{
  time_before_traj_msg_ = current_time;
  state_before_traj_msg_ = current_point;

  if (current_point.velocities.empty() && !trajectory_msg_->points[0].velocities.empty())
  {
    state_before_traj_msg_.velocities.resize(trajectory_msg_->points[0].velocities.size(), 0.0);
  }
  if (current_point.accelerations.empty() && !trajectory_msg_->points[0].accelerations.empty())
  {
    state_before_traj_msg_.accelerations.resize(
      trajectory_msg_->points[0].accelerations.size(), 0.0);
  }
}

void Trajectory::update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory)
{
  trajectory_msg_ = joint_trajectory;
  trajectory_start_time_ = static_cast<rclcpp::Time>(joint_trajectory->header.stamp);
  sampled_already_ = false;
  last_sample_idx_ = 0;
}
//找到采样点对应的轨迹片段并采样
bool Trajectory::sample(
  const rclcpp::Time & sample_time,
  trajectory_msgs::msg::JointTrajectoryPoint & output_state,  //跨周期复用的成员变量，调用的时候填的是state_desired_
  TrajectoryPointConstIter & start_segment_itr,
  TrajectoryPointConstIter & end_segment_itr)
{
  THROW_ON_NULLPTR(trajectory_msg_)

  if (trajectory_msg_->points.empty())
  {
    start_segment_itr = end();
    end_segment_itr = end();
    return false;
  }

  if (!sampled_already_)
  {
    if (trajectory_start_time_.seconds() == 0.0)
    {
      trajectory_start_time_ = sample_time;
    }
    sampled_already_ = true;
  }

  if (sample_time < time_before_traj_msg_)
  {
    return false;
  }

  output_state = trajectory_msgs::msg::JointTrajectoryPoint();

  auto & first_point_in_msg = trajectory_msg_->points[0];
  const rclcpp::Time first_point_timestamp =
    trajectory_start_time_ + first_point_in_msg.time_from_start;

  if (sample_time < first_point_timestamp)
  {
    interpolate_between_points(
      time_before_traj_msg_, state_before_traj_msg_,
      first_point_timestamp, first_point_in_msg,
      sample_time, output_state);
    start_segment_itr = begin();
    end_segment_itr = begin();
    return true;
  }

  const auto last_idx = trajectory_msg_->points.size() - 1;
  for (size_t i = last_sample_idx_; i < last_idx; ++i)
  {
    auto & point = trajectory_msg_->points[i];
    auto & next_point = trajectory_msg_->points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;

    if (sample_time >= t0 && sample_time < t1)
    {
      interpolate_between_points(t0, point, t1, next_point, sample_time, output_state);
      start_segment_itr = begin() + static_cast<long>(i);
      end_segment_itr = begin() + static_cast<long>(i + 1);
      output_state.time_from_start = next_point.time_from_start;
      last_sample_idx_ = i;
      return true;
    }
  }

  // 轨迹执行完毕
  start_segment_itr = --end(); //end的前一个点
  end_segment_itr = end();
  last_sample_idx_ = last_idx;
  output_state = (*start_segment_itr);

  if (output_state.velocities.empty())
  {
    output_state.velocities.resize(output_state.positions.size(), 0.0);
  }
  if (output_state.accelerations.empty())
  {
    output_state.accelerations.resize(output_state.positions.size(), 0.0);
  }
  return true;
}

void Trajectory::interpolate_between_points(
  const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
  const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
  const rclcpp::Time & sample_time,
  trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  rclcpp::Duration duration_so_far = sample_time - time_a;
  rclcpp::Duration duration_btwn_points = time_b - time_a;

  const size_t dim = state_a.positions.size();
  output.positions.resize(dim, 0.0);
  output.velocities.resize(dim, 0.0);
  output.accelerations.resize(dim, 0.0);

  auto generate_powers = [](int n, double x, double * powers) {
    powers[0] = 1.0;
    for (int i = 1; i <= n; ++i)
    {
      powers[i] = powers[i - 1] * x;
    }
  };

  bool has_velocity = !state_a.velocities.empty() && !state_b.velocities.empty();
  bool has_accel = !state_a.accelerations.empty() && !state_b.accelerations.empty();

  if (duration_so_far.seconds() < 0.0)
  {
    duration_so_far = rclcpp::Duration::from_seconds(0.0);
    has_velocity = has_accel = false;
  }
  if (duration_so_far.seconds() > duration_btwn_points.seconds())
  {
    duration_so_far = duration_btwn_points;
    has_velocity = has_accel = false;
  }

  double t[6];
  generate_powers(5, duration_so_far.seconds(), t);

  if (!has_velocity && !has_accel) //线性插值，只有位置
  {
    // linear interpolation
    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double end_pos = state_b.positions[i];
      double coefficients[2] = {0.0, 0.0};
      coefficients[0] = start_pos;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[1] = (end_pos - start_pos) / duration_btwn_points.seconds();
      }
      output.positions[i] = t[0] * coefficients[0] + t[1] * coefficients[1];
      output.velocities[i] = t[0] * coefficients[1];
    }
  }
  else if (has_velocity && !has_accel) //三次插值，位置和速度
  {
    // cubic interpolation
    double T[4];
    generate_powers(3, duration_btwn_points.seconds(), T);
    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];
      double coefficients[4] = {0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[2] =
          (-3.0 * start_pos + 3.0 * end_pos - 2.0 * start_vel * T[1] - end_vel * T[1]) / T[2];
        coefficients[3] =
          (2.0 * start_pos - 2.0 * end_pos + start_vel * T[1] + end_vel * T[1]) / T[3];
      }
      output.positions[i] =
        t[0] * coefficients[0] + t[1] * coefficients[1] +
        t[2] * coefficients[2] + t[3] * coefficients[3];
      output.velocities[i] =
        t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
        t[2] * 3.0 * coefficients[3];
      output.accelerations[i] =
        t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3];
    }
  }
  else if (has_velocity && has_accel) //五次插值，有位置和速度和加速度
  {
    // quintic interpolation
    double T[6];
    generate_powers(5, duration_btwn_points.seconds(), T);
    for (size_t i = 0; i < dim; ++i)
    {
      double start_pos = state_a.positions[i];
      double start_vel = state_a.velocities[i];
      double start_acc = state_a.accelerations[i];
      double end_pos = state_b.positions[i];
      double end_vel = state_b.velocities[i];
      double end_acc = state_b.accelerations[i];
      double coefficients[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      coefficients[2] = 0.5 * start_acc;
      if (duration_btwn_points.seconds() != 0.0)
      {
        coefficients[3] =
          (-20.0 * start_pos + 20.0 * end_pos - 3.0 * start_acc * T[2] + end_acc * T[2] -
           12.0 * start_vel * T[1] - 8.0 * end_vel * T[1]) / (2.0 * T[3]);
        coefficients[4] =
          (30.0 * start_pos - 30.0 * end_pos + 3.0 * start_acc * T[2] - 2.0 * end_acc * T[2] +
           16.0 * start_vel * T[1] + 14.0 * end_vel * T[1]) / (2.0 * T[4]);
        coefficients[5] =
          (-12.0 * start_pos + 12.0 * end_pos - start_acc * T[2] + end_acc * T[2] -
           6.0 * start_vel * T[1] - 6.0 * end_vel * T[1]) / (2.0 * T[5]);
      }
      output.positions[i] =
        t[0] * coefficients[0] + t[1] * coefficients[1] + t[2] * coefficients[2] +
        t[3] * coefficients[3] + t[4] * coefficients[4] + t[5] * coefficients[5];
      output.velocities[i] =
        t[0] * coefficients[1] + t[1] * 2.0 * coefficients[2] +
        t[2] * 3.0 * coefficients[3] + t[3] * 4.0 * coefficients[4] +
        t[4] * 5.0 * coefficients[5];
      output.accelerations[i] =
        t[0] * 2.0 * coefficients[2] + t[1] * 6.0 * coefficients[3] +
        t[2] * 12.0 * coefficients[4] + t[3] * 20.0 * coefficients[5];
    }
  }
}
TrajectoryPointConstIter Trajectory::begin() const
{
  THROW_ON_NULLPTR(trajectory_msg_)
  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter Trajectory::end() const
{
  THROW_ON_NULLPTR(trajectory_msg_)
  return trajectory_msg_->points.end();
}

bool Trajectory::has_trajectory_msg() const
{
  return trajectory_msg_.get() != nullptr;
}

}  // namespace panda_arm_controller
