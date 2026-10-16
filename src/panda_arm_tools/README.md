# panda_arm_tools

Panda 机械臂的**调试与标定工具包**：读取电机位置、零位置校准、参数管理。

本包与 `panda_arm_hardware`（实时控制）职责分离——硬件包只管控制，
工具包只管调试标定，两者通过配置文件解耦。

---

## 目录结构

```
panda_arm_tools/
├── include/panda_arm_tools/
│   ├── motor_reader.hpp       # 库：零力矩安全读取电机状态
│   └── config_manager.hpp     # 库：读写 real_arm_config 的 YAML 配置
├── src/
│   ├── motor_reader.cpp
│   ├── config_manager.cpp
│   ├── read_position_node.cpp     # 节点①：读位置
│   └── zero_calibrator_node.cpp   # 节点②：零位校准
├── CMakeLists.txt
└── package.xml
```

---

## 两个节点

| 节点 | 功能 | 需要串口 | 可与 launch 同时运行 |
|------|------|---------|---------------------|
| `read_position_node` | 读 7 个电机舵盘位置，打印后退出 | ✅ | ❌ |
| `zero_calibrator_node` | 零位校准 / 清除，写入 YAML | ✅（`clear` 模式不需要） | ❌ |

> **串口独占提醒**：串口同一时刻只能被一个进程打开。运行需要串口的节点前，
> 请先 Ctrl+C 停掉 `slider_control.launch.py`。

---

## 编译

```bash
cd ~/panda_mpc_arm
colcon build --packages-select panda_arm_tools && source install/setup.bash
```

---

## 使用

### ① `read_position_node` — 读取当前位置

```bash
ros2 run panda_arm_tools read_position_node --ros-args -p serial_port:=/dev/ttyACM0
```

一次性执行，打印每个电机的舵盘位置后退出（不常驻、不 spin）：

```
===================== 电机当前舵盘位置 =====================
串口: /dev/ttyACM0    波特率: 6000000

  ID   位置(rad)    位置(deg)    外部编码(rad)   温度   电压   状态
  ----------------------------------------------------------------------
  1      0.4245      24.3259          0.4310     36    24.0   正常
  2     -0.1234      -7.0700         -0.1180     35    24.0   正常
  ...
  ----------------------------------------------------------------------
  在线 7/7 个电机
```

**参数**：

| 参数 | 默认 | 说明 |
|------|------|------|
| `serial_port` | `/dev/ttyACM0` | 串口设备 |
| `timeout` | `0.2` | 串口超时（秒） |

**注意**：读取采用**零力矩指令**（mode=1 但 kp=kd=0，使 τ=0），
电机处于自由状态，不会因本次读取而转动。

输出末尾会提示内部编码器（`outputPos`）与外部编码器（`ExPos`）的偏差，
可用于装臂前校验编码器一致性。

---

### ② `zero_calibrator_node` — 零位置校准

把机械臂**当前姿态**标定为零位。

```bash
# 校准：把当前姿态设为零位
ros2 run panda_arm_tools zero_calibrator_node --ros-args -p serial_port:=/dev/ttyACM0

# 清除：把 zero_offsets 全部置 0
ros2 run panda_arm_tools zero_calibrator_node --ros-args -p clear:=true
```

**参数**：

| 参数 | 默认 | 说明 |
|------|------|------|
| `serial_port` | `/dev/ttyACM0` | 串口设备 |
| `clear` | `false` | true 时清除零位（不需要串口） |
| `timeout` | `0.2` | 串口超时（秒） |

执行流程：读取各电机当前位置 → 写入 `motor_params.yaml` 的 `zero_offsets` →
回读校验 → 提示重启方式。

**生效约定**：硬件层 `实际角度 = 编码器读数 − zero_offset`。
校准后当前姿态的软件角度即为 0。**需重启 launch 生效**。

> 单圈绝对编码器的物理位置与读数唯一对应，因此该偏移持久有效：
> 断电重启后只要物理姿态未变，软件角度仍正确。
> 若电机被手动拨动或重新装配，需重新标定。

---

## 库说明

### `motor_reader`

封装 `MotorProtocol`，提供零力矩安全读取。

核心：下发 `mode=1`(FOC 闭环)、`timeout=0`(禁用超时保护)、
力矩/速度/位置/刚度/阻尼**全 0** 的指令。由控制律
`τ = τ_ff + kp·(p_des−p) + kd·(ω_des−ω)` 可知，kp=kd=0 且 τ_ff=0 时
输出力矩 τ=0，电机零力矩自由，既能取回完整反馈又不会自行转动。

### `config_manager`

读写 `real_arm_config` 的 YAML 配置，目前只服务 `zero_calibrator_node`
的 `zero_offsets` 读写。

**关键设计**：采用**文本行处理**而非通用 YAML 库（如 yaml-cpp）写回，
只替换目标行的数值，因此**行尾注释、空行、其余内容全部原样保留**
（手写的 YAML 不会被库重新格式化掉）。

> 说明：原先还提供 `set_default_kp/kd`、`set_bridge_param` 等方法，是给已删除的
> `param_manager_node` 用的，现已移除。kp/kd/max_velocity 等参数请直接编辑
> `real_arm_config/config/` 下的 YAML，然后重新 `colcon build`。

段定位支持**任意层级嵌套**（按缩进判断归属），因此既能处理顶层段
（`zero_offsets:`），也能处理嵌套段（`real_rviz_bridge: → ros__parameters:`）。

---

## 依赖关系

```
panda_arm_tools
    ├── panda_arm_hardware（复用 MotorProtocol 协议层）
    ├── real_arm_config（访问其 share 目录下的配置文件）
    ├── rclcpp
    └── ament_index_cpp
```

**只链接 `motor_protocol` 轻量库**（仅依赖 rclcpp），
不引入 `hardware_interface` / `controller_interface` 等 ros2_control 依赖链。

---

## 典型工作流（装臂后）

```bash
# 1. 停掉 ROS，扫描确认电机在线
ros2 run panda_arm_tools read_position_node --ros-args -p serial_port:=/dev/ttyACM0

# 2. 把臂摆到机械零位，校准
ros2 run panda_arm_tools zero_calibrator_node --ros-args -p serial_port:=/dev/ttyACM0

# 3. 启动正常控制
ros2 launch real_arm_config slider_control.launch.py serial_port:=/dev/ttyACM0

# 4. 调参：直接编辑 YAML 后重新编译（kp/kd 在 on_init 读入，不支持热更新）
#    src/real_arm_config/config/motor_params.yaml   → kp / kd
#    src/real_arm_config/config/bridge_params.yaml  → max_velocity / publish_rate
colcon build --packages-select real_arm_config && source install/setup.bash
```
