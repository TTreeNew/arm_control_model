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

#ifndef PANDA_ARM_CONTROLLER__TRAJECTORY_HPP_
#define PANDA_ARM_CONTROLLER__TRAJECTORY_HPP_

#include <algorithm>  // 下面 mapping() 用 std::find（原先靠传递包含侥幸编译）
#include <memory>
#include <vector>

#include "rclcpp/time.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace panda_arm_controller
{
using TrajectoryPointIter = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::iterator;
using TrajectoryPointConstIter =
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  /// 唯一构造：创建"轨迹执行引擎"的空壳（trajectory_msg_ 为空）。
  /// 装一条轨迹是两段式：
  ///   update(msg)                          // 装入轨迹 + 锚定起点时间
  ///   set_point_before_trajectory_msg(...) // 补"轨迹之前的点"（平滑接续用）
  /// 刻意不提供"构造时带轨迹 / 带前点"的重载：控制器长期复用同一对象，
  /// 换轨迹只调 update()，实时路径上不做任何对象构造。
  Trajectory();

  /**
   * 记录"轨迹之前的点"：新轨迹开始采样前，用当前实际（或上次下发的）状态作为插值起点，
   * 使采样从现有位置平滑接续到轨迹第一个点。
   */
  void set_point_before_trajectory_msg(
    const rclcpp::Time & current_time,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_point);

  void update(std::shared_ptr<trajectory_msgs::msg::JointTrajectory> joint_trajectory);

  /// 采样 Find the segment (made up of 2 points) and its expected state from the containing trajectory.
  /// 插值阶数由两个端点提供了哪些字段决定：
  ///   只有 positions -> 线性；另有 velocities -> 三次；再有 accelerations -> 五次。
  bool sample(
    const rclcpp::Time & sample_time,
    trajectory_msgs::msg::JointTrajectoryPoint & output_state,
    TrajectoryPointConstIter & start_segment_itr, TrajectoryPointConstIter & end_segment_itr);
//工具函数，被sample调用
  void interpolate_between_points(
    const rclcpp::Time & time_a, const trajectory_msgs::msg::JointTrajectoryPoint & state_a,
    const rclcpp::Time & time_b, const trajectory_msgs::msg::JointTrajectoryPoint & state_b,
    const rclcpp::Time & sample_time, trajectory_msgs::msg::JointTrajectoryPoint & output);

  TrajectoryPointConstIter begin() const;//const 成员函数,表示调用这个函数不会修改对象的任何成员变量
  TrajectoryPointConstIter end() const;
  bool has_trajectory_msg() const;//轨迹是否存在

  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> get_trajectory_msg() const
  {
    return trajectory_msg_;
  }

  bool is_sampled_already() const { return sampled_already_; }

private:
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg_;
  rclcpp::Time trajectory_start_time_;

  rclcpp::Time time_before_traj_msg_;
  trajectory_msgs::msg::JointTrajectoryPoint state_before_traj_msg_;

  bool sampled_already_ = false;
  size_t last_sample_idx_ = 0;
};

//建立 t1 中每个元素在 t2 中的位置映射： mapping_vector[i] = 「t1 的第 i 个元素，在 t2 里排第几」
template <class T>
inline std::vector<size_t> mapping(const T & t1, const T & t2)
{
  if (t1.size() > t2.size())
  {
    return std::vector<size_t>();
  }
  std::vector<size_t> mapping_vector(t1.size());
  for (auto t1_it = t1.begin(); t1_it != t1.end(); ++t1_it)
  {
    auto t2_it = std::find(t2.begin(), t2.end(), *t1_it);
    if (t2.end() == t2_it)
    {
      return std::vector<size_t>();
    }
    else
    {
      const size_t t1_dist = static_cast<size_t>(std::distance(t1.begin(), t1_it));
      const size_t t2_dist = static_cast<size_t>(std::distance(t2.begin(), t2_it));
      mapping_vector[t1_dist] = t2_dist;
    }
  }
  return mapping_vector;
}

}  // namespace panda_arm_controller

#endif  // PANDA_ARM_CONTROLLER__TRAJECTORY_HPP_
