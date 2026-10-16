#include "panda_arm_tools/config_manager.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace panda_arm_tools
{

namespace
{

/// 判断某行（忽略前导空白后）是否以 "key:" 开头
bool line_starts_with_key(const std::string & line, const std::string & key)
{
  size_t p = line.find_first_not_of(" \t");
  if (p == std::string::npos) {
    return false;
  }
  std::string rest = line.substr(p);
  return rest.rfind(key + ":", 0) == 0;
}

}  // namespace

bool ConfigManager::resolve_paths(
  std::string & motor_params_src,
  std::string & bridge_params_src,
  std::string & motor_params_install,
  std::string & bridge_params_install)
{
  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory("real_arm_config");
  } catch (const std::exception &) {
    return false;
  }

  motor_params_install = share + "/config/motor_params.yaml";
  bridge_params_install = share + "/config/bridge_params.yaml";

  // 由 install 路径反推 src 路径：
  //   <ws>/install/real_arm_config/share/real_arm_config
  //   → 上溯 4 层到 <ws>，再进入 src/real_arm_config/config
  namespace fs = std::filesystem;
  fs::path p(share);
  fs::path ws = p.parent_path()    // <ws>/install/real_arm_config/share
                  .parent_path()   // <ws>/install/real_arm_config
                  .parent_path()   // <ws>/install
                  .parent_path();  // <ws>

  motor_params_src = (ws / "src" / "real_arm_config" / "config" / "motor_params.yaml").string();
  bridge_params_src = (ws / "src" / "real_arm_config" / "config" / "bridge_params.yaml").string();

  return true;
}

bool ConfigManager::replace_key_in_section(
  const std::string & file,
  const std::string & section,
  const std::string & key,
  const std::string & new_value)
{
  std::ifstream in(file);
  if (!in) {
    return false;
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) {
    lines.push_back(line);
  }
  in.close();

  // ── 定位目标行 ──
  // 段(section)可以是任意层级：先找到 "section:" 所在行及其缩进，
  // 其后所有「缩进更大」的行都属于该段，直到缩进回退为止。
  // 这样既能匹配顶层段(如 zero_offsets:)，也能匹配嵌套段
  // (如 bridge_params.yaml 中 real_rviz_bridge → ros__parameters:)。
  int target = -1;

  if (section.empty()) {
    // 顶层模式：找行首无缩进的顶层 key
    for (size_t i = 0; i < lines.size(); i++) {
      const std::string & l = lines[i];
      size_t first = l.find_first_not_of(" \t");
      if (first == std::string::npos || l[first] == '#') {
        continue;
      }
      if (first == 0 && line_starts_with_key(l, key)) {
        target = static_cast<int>(i);
        break;
      }
    }
  } else {
    // 段模式：先定位 section 行与其缩进
    int sec_line = -1;
    size_t sec_indent = 0;

    for (size_t i = 0; i < lines.size(); i++) {
      const std::string & l = lines[i];
      size_t first = l.find_first_not_of(" \t");
      if (first == std::string::npos || l[first] == '#') {
        continue;
      }
      if (line_starts_with_key(l, section)) {
        sec_line = static_cast<int>(i);
        sec_indent = first;
        break;
      }
    }

    if (sec_line < 0) {
      return false;  // 未找到该段
    }

    // 在 section 之后，找缩进更大的 key 行
    for (size_t i = sec_line + 1; i < lines.size(); i++) {
      const std::string & l = lines[i];
      size_t first = l.find_first_not_of(" \t");
      if (first == std::string::npos || l[first] == '#') {
        continue;
      }
      if (first <= sec_indent) {
        break;  // 缩进回退，段已结束
      }
      if (line_starts_with_key(l, key)) {
        target = static_cast<int>(i);
        break;
      }
    }
  }

  if (target < 0) {
    return false;  // 未找到
  }

  // ── 替换值，保留锚点标记与行尾注释 ──
  // 形如 "  kp: &kp_value 2.0" 或 "  joint_1: 0.0" 或 "    enabled: true"
  //   组1 = 前缀(缩进 + key + 冒号 + 空格)
  //   组2 = 可选的锚点标记(如 "&kp_value ")
  //   组3 = 原值
  //   组4 = 行尾剩余(可能是注释)
  std::regex re("^(\\s*" + key + "\\s*:\\s*)(&[^\\s]+\\s+)?([^\\s#]+)(.*)$");
  std::smatch m;
  if (!std::regex_match(lines[target], m, re)) {
    return false;
  }

  lines[target] = m[1].str() + (m[2].matched ? m[2].str() : "") + new_value + m[4].str();

  std::ofstream out(file, std::ios::trunc);
  if (!out) {
    return false;
  }
  for (const auto & l : lines) {
    out << l << "\n";
  }
  out.close();

  return true;
}

bool ConfigManager::replace_both(
  const std::string & src_file,
  const std::string & install_file,
  const std::string & section,
  const std::string & key,
  const std::string & new_value)
{
  bool ok_src = false;
  bool ok_inst = false;

  std::ifstream f(src_file);
  if (f.good()) {
    f.close();
    ok_src = replace_key_in_section(src_file, section, key, new_value);
  }

  std::ifstream g(install_file);
  if (g.good()) {
    g.close();
    ok_inst = replace_key_in_section(install_file, section, key, new_value);
  }

  return ok_src || ok_inst;  // 任一处成功即可
}

namespace
{

/// 零位偏移是标定值，需要较高精度（6 位小数）
std::string fmt_precise(double v)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6) << v;
  return oss.str();
}

}  // namespace

bool ConfigManager::set_zero_offsets(const std::vector<double> & offsets)
{
  std::string ms, bs, mi, bi;
  if (!resolve_paths(ms, bs, mi, bi)) {
    return false;
  }

  bool any_ok = false;
  for (size_t i = 0; i < offsets.size() && i < 7; i++) {
    std::string key = "joint_" + std::to_string(i + 1);
    if (replace_both(ms, mi, "zero_offsets", key, fmt_precise(offsets[i]))) {
      any_ok = true;
    }
  }
  return any_ok;
}

double ConfigManager::get_numeric_in_section(
  const std::string & file,
  const std::string & section,
  const std::string & key)
{
  std::ifstream in(file);
  if (!in) {
    return -1e9;
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) {
    lines.push_back(line);
  }

  // 与 replace_key_in_section 一致的段定位：section 可为任意层级
  size_t start_idx = 0;
  size_t sec_indent = 0;

  if (!section.empty()) {
    int sec_line = -1;
    for (size_t i = 0; i < lines.size(); i++) {
      size_t first = lines[i].find_first_not_of(" \t");
      if (first == std::string::npos || lines[i][first] == '#') {
        continue;
      }
      if (line_starts_with_key(lines[i], section)) {
        sec_line = static_cast<int>(i);
        sec_indent = first;
        break;
      }
    }
    if (sec_line < 0) {
      return -1e9;  // 未找到该段
    }
    start_idx = static_cast<size_t>(sec_line) + 1;
  }

  for (size_t i = start_idx; i < lines.size(); i++) {
    const std::string & l = lines[i];
    size_t first = l.find_first_not_of(" \t");
    if (first == std::string::npos || l[first] == '#') {
      continue;
    }
    if (!section.empty()) {
      if (first <= sec_indent) {
        break;  // 缩进回退，段已结束
      }
    } else if (first != 0) {
      continue;  // 顶层模式：只认行首无缩进的 key
    }

    if (line_starts_with_key(l, key)) {
      std::regex re("^\\s*" + key + "\\s*:\\s*(?:&[^\\s]+\\s+)?([^\\s#]+)");
      std::smatch m;
      if (std::regex_search(l, m, re)) {
        try {
          return std::stod(m[1].str());
        } catch (const std::exception &) {
          return -1e9;
        }
      }
    }
  }
  return -1e9;
}

std::vector<double> ConfigManager::get_zero_offsets()
{
  std::vector<double> offsets;
  std::string ms, bs, mi, bi;
  if (!resolve_paths(ms, bs, mi, bi)) {
    return offsets;
  }

  std::ifstream f(ms);
  const std::string & file = f.good() ? ms : mi;

  for (int i = 1; i <= 7; i++) {
    double v = get_numeric_in_section(file, "zero_offsets", "joint_" + std::to_string(i));
    offsets.push_back(v);
  }
  return offsets;
}

}  // namespace panda_arm_tools
