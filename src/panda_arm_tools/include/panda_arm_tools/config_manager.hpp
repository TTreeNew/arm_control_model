#ifndef PANDA_ARM_TOOLS__CONFIG_MANAGER_HPP_
#define PANDA_ARM_TOOLS__CONFIG_MANAGER_HPP_

#include <string>
#include <vector>

namespace panda_arm_tools
{

/// 配置管理器：读写 real_arm_config 包中的 YAML 参数文件
///
/// 设计说明（重要）：
/// 采用「文本行处理」而非通用 YAML 库（如 yaml-cpp）写回，只替换目标行的
/// 数值，因此行尾注释、空行、其余内容全部原样保留（手写 YAML 不会被重新格式化）。
class ConfigManager
{
public:
  /// 定位配置文件。
  /// src 路径用于持久化（重新 colcon build 不会被覆盖）；
  /// install 路径用于立即生效（节点实际读取的位置）。
  static bool resolve_paths(
    std::string & motor_params_src,
    std::string & bridge_params_src,
    std::string & motor_params_install,
    std::string & bridge_params_install);

  /// 设置 7 个关节的零位偏移（rad）
  static bool set_zero_offsets(const std::vector<double> & offsets);

  /// 读取当前零位偏移（用于显示），失败返回空
  static std::vector<double> get_zero_offsets();

private:
  /// 在指定段(section)内替换 key 的值。
  /// section 为空串时表示「顶层 key」（行首无缩进）。
  /// 保留锚点标记（如 &kp_value）与行尾注释。
  static bool replace_key_in_section(
    const std::string & file,
    const std::string & section,
    const std::string & key,
    const std::string & new_value);

  /// 对 src 与 install 两处配置都执行同一替换，保证持久化 + 立即生效
  static bool replace_both(
    const std::string & src_file,
    const std::string & install_file,
    const std::string & section,
    const std::string & key,
    const std::string & new_value);

  /// 从指定段中读取 key 的数值
  static double get_numeric_in_section(
    const std::string & file,
    const std::string & section,
    const std::string & key);
};

}  // namespace panda_arm_tools

#endif  // PANDA_ARM_TOOLS__CONFIG_MANAGER_HPP_
