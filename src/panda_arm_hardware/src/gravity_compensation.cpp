#include "panda_arm_hardware/gravity_compensation.hpp"

#include <exception>
#include <utility>

#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

//重力补偿文件，用pinocchi计算重力补偿
namespace panda_arm_hardware
{

bool GravityCompensation::configure(
  const std::string & urdf,
  const std::vector<std::string> & joint_names,
  const GravityParams & params)
{
  // 已就绪就不重建：compute() 可能正跑在实时线程里读 model_/data_，
  // 重建会造成数据竞争（正常场景也不会走到这里）。
  if (ready_.load(std::memory_order_acquire))
  {
    return true;
  }

  if (urdf.empty() || joint_names.empty())
  {
    return false;
  }
  if (!params.sign.empty() && params.sign.size() != joint_names.size())
  {
    return false;
  }

  // ── ① 解析 URDF → pinocchio 模型（会分配内存，所以本函数必须非实时调用）──
  auto model = std::make_unique<pinocchio::Model>();
  // 构建 Pinocchio 动力学模型
  pinocchio::urdf::buildModelFromXML(urdf, *model);

  // ── ② 显式设重力向量（ Z-up） ──
  model->gravity.linear(Eigen::Vector3d(0.0, 0.0, -9.81));

  // ── ③ 三重校验：自由度数目、关节名存在、每个关节恰好 1 自由度 ──
  const size_t dof = joint_names.size();
  if (static_cast<size_t>(model->nq) != dof || static_cast<size_t>(model->nv) != dof)
  {
    return false;
  }

  // 按【名字】建立映射
  std::vector<int> q_idx(dof, -1);
  std::vector<int> v_idx(dof, -1);
  for (size_t i = 0; i < dof; ++i)
  {
    const auto joint_id = model->getJointId(joint_names[i]);
    // 必须是单自由度关节（注意 continuous 关节会占 nq=2，会让下标映射失效）
    const auto & joint_model = model->joints[joint_id];
    if (joint_model.nq() != 1 || joint_model.nv() != 1)
    {
      return false;
    }
    q_idx[i] = joint_model.idx_q();
    v_idx[i] = joint_model.idx_v();
  }

  // ── ④ 全部校验通过后才写成员（顺序：模型 → Data → 下标/缓冲 → 标志）──
  model_ = std::move(model);
  data_ = std::make_unique<pinocchio::Data>(*model_);
  q_idx_ = std::move(q_idx);
  v_idx_ = std::move(v_idx);
  q_pin_ = Eigen::VectorXd::Zero(model_->nq);
  tau_pin_ = Eigen::VectorXd::Zero(model_->nv);

  scale_ = params.scale;
  sign_.assign(dof, 1.0);
  if (!params.sign.empty())
  {
    sign_ = params.sign;
  }
  dof_ = dof;

  // release：保证上面所有写入对实时线程可见（它与 compute() 的 acquire 配对）
  ready_.store(true, std::memory_order_release);
  return true;
}

bool GravityCompensation::compute(
  const std::vector<double> & q,
  std::vector<double> & tau_out)
{
  if (!ready_.load(std::memory_order_acquire))
  {
    return false;
  }
  if (q.size() != dof_)
  {
    return false;
  }

  // 实测位形按名字映射后的下标填入配置向量
  for (size_t i = 0; i < dof_; ++i)
  {
    q_pin_[q_idx_[i]] = q[i];
  }

  // 静止假设（v = 0, a = 0）下的重力广义力矩 → data.g，等价于 rnea(model, data, q, 0, 0)
  pinocchio::computeGeneralizedGravity(*model_, *data_, q_pin_);

  // 正常情况下调用方已按 dof() 预分配，这里只做兜底（最多发生一次）
  if (tau_out.size() != dof_)
  {
    tau_out.resize(dof_, 0.0);
  }
  for (size_t i = 0; i < dof_; ++i)
  {
    tau_pin_[v_idx_[i]] = data_->g[v_idx_[i]];
    tau_out[i] = scale_ * sign_[i] * data_->g[v_idx_[i]];
  }
  return true;
}

}  // namespace panda_arm_hardware
