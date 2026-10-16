# real_arm_config

真实 Panda 机械臂（J288/S288 电机）的 **配置与启动包**。

本包只存放**配置与 launch 文件**，不含 C++ 源码。它的职责是：把硬件参数、
控制器参数、RViz 配置集中管理，并组装出可运行的 ros2_control 启动流程。

> 代码（硬件接口、通信协议、控制器）位于 `panda_arm_hardware`；
> 调试与标定工具位于 `panda_arm_tools`。

---

## 目录结构

```
real_arm_config/
├── config/
│   ├── motor_params.yaml              # 硬件参数：kp/kd/default_velocity/zero_offsets
│   ├── bridge_params.yaml             # 桥接参数：max_velocity/publish_rate/...
│   ├── ros2_controllers.yaml          # 控制器管理器：update_rate 与控制器列表
│   ├── initial_positions.yaml         # 各关节初始位置
│   ├── panda_arm.urdf.xacro           # URDF 入口：组装模型并传入参数
│   ├── panda_arm.ros2_control.xacro   # ros2_control 定义：插件、joint 参数、接口
│   └── display.rviz                   # RViz 显示配置
├── launch/
│   └── slider_control.launch.py       # RViz 滑块控制真实机械臂
├── CMakeLists.txt
└── package.xml
```

---

## 快速开始

```bash
cd ~/panda_mpc_arm
colcon build --packages-select real_arm_config && source install/setup.bash

ros2 launch real_arm_config slider_control.launch.py serial_port:=/dev/ttyACM0
```

启动后会依次拉起：`robot_state_publisher` → `ros2_control_node` →
`joint_state_broadcaster` → `panda_arm_controller` → `real_rviz_bridge` → `RViz2`。

---

## 配置文件详解

### 1. `motor_params.yaml` — 硬件参数

被 `panda_arm.ros2_control.xacro` 通过 `xacro.load_yaml()` 读取，
展开进 URDF 的 `<param>` 标签，最终由 `PandaArmHardware::on_init()` 使用。

```yaml
# 共享默认值：改这两处，7 个关节全部生效
defaults:
  kp: &kp_value 2.0
  kd: &kd_value 0.2

kp:                      # 各关节引用锚点
  joint_1: *kp_value
  ...
  joint_7: *kp_value

kd:
  joint_1: *kd_value
  ...

default_velocity: 1.5    # rad/s，补位满速

zero_offsets:            # rad，零位偏移（由标定工具写入）
  joint_1: 0.0
  ...
```

**锚点机制说明**：YAML 的 `&kp_value`（定义）与 `*kp_value`（引用）让所有关节共享
一个数值。改 `defaults` 一处即可全局生效；若某个关节需单独调节，直接把该行写成
具体数值（如 `joint_3: 3.0`），其余关节不受影响。

**参数含义**（舵盘端 / 输出端单位）：

| 参数 | 单位 | 说明 |
|------|------|------|
| `kp` | N·m/rad | 位置刚度，误差产生的力矩 = kp × Δθ |
| `kd` | N·m/(rad/s) | 阻尼，抑制振荡 |
| `default_velocity` | rad/s | 补位满速上限 |
| `zero_offsets` | rad | 零位偏移 |

### 2. `bridge_params.yaml` — 桥接节点参数

由 `slider_control.launch.py` 加载给 `real_rviz_bridge` 节点（ROS 参数机制）。

```yaml
real_rviz_bridge:
  ros__parameters:
    publish_rate: 50.0      # Hz，轨迹发布频率
    max_velocity: 1.5       # rad/s，滑块目标位置变化限幅
    step_duration: 0.1      # s，每步轨迹时长
    enabled: true           # 总开关，false 则桥接完全停摆
```

**关键关系**：每步位移上限 = `max_velocity × step_duration`（当前 1.5 × 0.1 = 0.15 rad ≈ 8.6°）。
`step_duration` 同时作为轨迹点时间戳，告诉控制器该目标点应在多久内到达。

**为何不与 `motor_params.yaml` 合并**：ROS 参数文件强制要求 `节点名: ros__parameters:`
嵌套结构，而 xacro 的 `load_yaml` 只认扁平字典，两种格式不兼容。

### 3. `ros2_controllers.yaml` — 控制器配置

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200        # Hz，控制周期（即 write() 调用频率）

    panda_arm_controller:
      type: panda_arm_hardware/PandaArmController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

`update_rate` 决定硬件通信频率。当前 200 Hz 下，串口负载约 322 kbps，
仅占 6 Mbps 波特率的 5.4%，余量充足。

### 4. xacro 文件

- **`panda_arm.urdf.xacro`**：入口，定义参数默认值（串口、电机 ID 1~7、配置文件路径），
  引入模型并调用 ros2_control 宏。
- **`panda_arm.ros2_control.xacro`**：定义 `ros2_control` 节点，为每个 joint 声明
  `motor_id`/`kp`/`kd`/`zero_offset` 参数，以及 position/velocity/effort 三类
  命令与状态接口。

---

## 参数修改与生效

所有参数改完后：

```bash
colcon build --packages-select real_arm_config && source install/setup.bash
# 然后重启 launch
```

> `real_arm_config` 是纯配置包，`colcon build` 只是把 `src/` 的 YAML 同步到
> `install/`，耗时约 0.1 秒，不编译 C++。重启 launch 是必须的——参数在启动时
> 被 xacro 展开进 URDF / 被 launch 加载进节点。

推荐使用 `panda_arm_tools` 的参数管理工具来改，它会自动同步 src 与 install 两处：

```bash
ros2 run panda_arm_tools param_manager_node --ros-args -p kp:=3.0
```

---

## 数据流

```
motor_params.yaml ──xacro load_yaml──> URDF <param> ──> PandaArmHardware::on_init()
                                                              ↓
                                                         kp / kd / default_velocity / zero_offset

bridge_params.yaml ──launch 加载──> real_rviz_bridge 节点参数
                                          ↓
                             publish_rate / max_velocity / step_duration / enabled

ros2_controllers.yaml ──> controller_manager（update_rate）
```

---

## 注意事项

1. **串口独占**：`/dev/ttyACM0` 同一时刻只能被一个进程打开。若要运行
   `panda_arm_tools` 的 `read_position_node` / `zero_calibrator_node`，
   需先 Ctrl+C 停掉本包的 launch。

2. **零位偏移的双向闭环**：硬件层约定 `实际角度 = 编码器读数 − zero_offset`。
   下发命令时会加回偏移（`pos_cmd + zero_offset`），读取时再减掉。
   两处必须对称，否则上电瞬间电机会猛拽。

3. **单圈绝对编码器**：断电后累积圈数会清零，但物理位置与读数的对应关系不变，
   因此零位偏移持久有效——只要物理姿态未变，重启后软件角度仍正确。
   若电机被手动拨动或重新装配，需重新标定。
