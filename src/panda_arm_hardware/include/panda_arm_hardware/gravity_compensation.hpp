// GravityCompensation —— 重力前馈 τ_ff = g(q)
//
// 原理（详见下方）：
//   关节空间动力学   M(q)q̈ + C(q,q̇)q̇ + g(q) = τ
//   静止时 q̇ = q̈ = 0，方程退化为   g(q) = τ
//   所以「稳态下垂 / 抬不动 / 饱和」这类问题只由 g(q) 决定，把 g(q) 当作前馈力矩
//   下发即可抵消重力，让电机的 kp 从"扛重力的主力"退回"修误差的配角"。
//
// 只做重力项，不做惯性项 M(q)q̈ 与科氏项 C(q,q̇)q̇：
//   · 后两项只在【运动】时有值，不影响稳态；当前的限速 2 rad/s 下惯性项
//     约占重力的 10~25%，补它的收益远小于补重力；
//   · 前馈惯性项还需要一个可信的参考加速度，而当前参考是限速斜坡
//     （速度阶跃、加速度尖峰），硬做差分只会引入冲击。
//   若将来做高速轨迹跟踪，再在 compute() 旁边加一个 computeMomentumFeedforward()
//   即可 —— 同一个 pinocchio Model 一行就能给出全部三项：
//     pinocchio::crba()              -> data.M
//     pinocchio::nonLinearEffects()  -> data.nle = C·v + g
//
// 设计约定：
//   · 本类【不依赖 ROS】：只收 URDF 字符串 + 参数结构体。因此可以脱离 ROS 单测，
//     将来 MPC 节点也能直接复用同一份模型。
//   · configure() 会解析 URDF 并建立 pinocchio 模型 → 会分配内存，
//     只能在【非实时】阶段（如控制器的 on_configure）调用。
//   · compute() 是【实时安全】的：无堆分配、无锁、无系统调用。
//   · 线程约定：configure() 在非实时线程、compute() 在实时线程。
//     两者用 acquire/release 配对同步，保证实时线程不会看到半成品模型。
//     模型只建一次，之后不再重建（重建会与实时线程的数据读取竞争）。

#ifndef PANDA_ARM_HARDWARE__GRAVITY_COMPENSATION_HPP_
#define PANDA_ARM_HARDWARE__GRAVITY_COMPENSATION_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/multibody/model.hpp"

namespace panda_arm_hardware
{

/// 重力补偿参数（与 ROS 解耦的纯数据结构，由调用方从参数服务器读好后传入）
struct GravityParams
{
  /// 缩放系数：0 = 关闭（安全默认），标定后调到 1.0 附近。
  /// URDF 的质量通常不含线缆/工装/负载，实测常偏小 20~50%，所以允许 > 1。
  double scale = 0.0;

  /// 逐关节符号（+1 / -1），顺序与传入的 joint_names 一致。
  /// URDF 的关节轴来自 CAD 建模坐标系，电机正方向来自装配实测，个别关节可能相反；
  /// 若某关节一加补偿就往一边猛冲，把对应位翻成 -1 即可，不必改 URDF。
  /// 留空 = 全部 +1。
  std::vector<double> sign;
};

class GravityCompensation
{
public:
  GravityCompensation() = default;

  /// 解析 URDF、建立模型与"控制器关节顺序 → pinocchio 关节"的映射。
  ///
  /// 做三重校验：自由度数目一致、每个关节名在 URDF 里存在、每个都恰好 1 自由度。
  /// 任一项不过 → 返回 false 且保持【未就绪】(compute() 会返回 false)，
  /// 调用方应据此把前馈力矩写 0，而不是猜一个值。
  ///
  /// 已经就绪时直接返回 true（不重建）。
  ///
  /// \param urdf         展开后的 URDF 字符串（如 /robot_description 话题内容）
  /// \param joint_names  控制器的关节顺序（本类结果按同样顺序输出）
  /// \param params       缩放与符号
  bool configure(
    const std::string & urdf,
    const std::vector<std::string> & joint_names,
    const GravityParams & params);

  /// 是否已就绪（可安全调用 compute() 并拿到有效结果）
  bool ready() const {return ready_.load(std::memory_order_acquire);}

  /// 关节数（configure() 成功后有效）
  size_t dof() const {return dof_;}

  /// 模型自由度，仅用于日志/自检（两者都应等于 dof()）
  int nq() const {return model_ ? model_->nq : 0;}
  int nv() const {return model_ ? model_->nv : 0;}

  /// 实时安全：计算当前位形下的重力前馈力矩。
  ///
  /// \param q        实测关节角（rad），按 configure() 传入的 joint_names 顺序
  /// \param tau_out  输出，与 q 同序，单位 Nm（调用方宜按 dof() 预分配，避免实时分配）
  /// \return false = 未就绪 或 q 长度不符；此时【不修改】tau_out
  bool compute(const std::vector<double> & q, std::vector<double> & tau_out);

private:
  // 用指针保存模型/数据：pinocchio::Data 内部持有指向 Model 的指针，
  // 且两者都可能很大；用指针可以明确"只在 configure() 里构造一次"。
  std::unique_ptr<pinocchio::Model> model_;
  std::unique_ptr<pinocchio::Data> data_;

  std::vector<int> q_idx_;  // 控制器关节 i → pinocchio 配置下标 idx_q
  std::vector<int> v_idx_;  // 控制器关节 i → pinocchio 速度下标 idx_v

  Eigen::VectorXd q_pin_;    // 长度 nq，configure() 里预分配
  Eigen::VectorXd tau_pin_;  // 长度 nv，compute() 的落点（data_.g 的副本，便于调试）

  std::vector<double> sign_;
  double scale_ = 1.0;
  size_t dof_ = 0;

  std::atomic<bool> ready_{false};
};

}  // namespace panda_arm_hardware

#endif  // PANDA_ARM_HARDWARE__GRAVITY_COMPENSATION_HPP_
