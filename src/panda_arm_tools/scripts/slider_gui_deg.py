#!/usr/bin/env python3
"""
自定义滑块节点

与 joint_state_publisher_gui 的区别：
  1. 滑块以【度】显示与拖动（内部仍用弧度，符合 ROS 标准），读数直观
  2. 限位自动从 URDF（robot_description 参数）读取 —— 改 URDF 后滑块范围
     自动跟随，无需在代码里硬编码角度
  3. 启动后订阅 /joint_states，用电机真实位置初始化滑块，解决"滑块总在 0、
     一拖机械臂就猛拽"的问题
  4. 同步完成前滑块置灰禁用，且不发布任何指令，防止启动时猛冲

话题：
  订阅 /joint_states       真实反馈（仅用于初始化滑块）
  发布 /gui_joint_states   滑块命令（弧度），由 real_rviz_bridge 接收
"""

import math
import threading
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
import tkinter as tk

PUBLISH_HZ = 50
SYNC_WARN_SEC = 5.0      # 多久没收到 /joint_states 就告警


class SliderGuiDeg(Node):
    def __init__(self):
        super().__init__('slider_gui_deg')

        self.joints = []
        self.lower_deg = []
        self.upper_deg = []
        self.rad = []
        self.synced = False
        self.scales = []
        self.labels = []
        self.root = None

        # ① 从 URDF 解析关节列表与限位（度）
        self._load_limits()

        # ② 真实反馈 → 初始化滑块
        #    BEST_EFFORT：无论发布端是 RELIABLE 还是 BEST_EFFORT 都能收到
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # ③ 滑块命令 → bridge
        self.pub = self.create_publisher(JointState, '/gui_joint_states', 10)

        # ④ 发布频率（本节点自己的采样/发送周期）
        #
        #    速率限制不在这里做，统一交给 real_rviz_bridge（本节点是所有上游的
        #    公共入口，joint_state_publisher_gui 也发同一个话题、且没有源头限速，
        #    所以限速必须放在入口、且只放一处，否则会出现两个 max_velocity 参数
        #    互相打架）。本节点因此只做「采样 + 发布」：把滑块真值原样发出去。
        self.declare_parameter('publish_rate', float(PUBLISH_HZ))
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.get_logger().info(
            f'发布频率: {self.publish_rate} Hz（限速由 real_rviz_bridge 负责）')

        # ⑤ 长时间收不到 /joint_states 时提示（但不自动启用，保持安全）
        self._warn_timer = self.create_timer(SYNC_WARN_SEC, self._on_sync_timeout)

    # ── 从 robot_description 解析 URDF ──
    def _load_limits(self):
        self.declare_parameter('robot_description', '')
        urdf_str = self.get_parameter('robot_description').value

        if not urdf_str:
            self.get_logger().warn(
                '未收到 robot_description 参数，回退为 joint_1~7、±150°。'
                '建议通过 launch 传入 robot_description。')
            self.joints = [f'joint_{i}' for i in range(1, 8)]
            self.lower_deg = [-150.0] * 7
            self.upper_deg = [150.0] * 7
        else:
            try:
                root = ET.fromstring(urdf_str)
            except ET.ParseError as e:
                raise RuntimeError(f'robot_description 不是合法 XML: {e}') from e

            for j in root.findall('joint'):
                # 跳过 fixed 关节（如 end_joint）：没有 limit，也不可动
                if j.get('type') not in ('revolute', 'continuous', 'prismatic'):
                    continue
                limit = j.find('limit')
                if limit is None:
                    self.get_logger().warn(f"关节 {j.get('name')} 缺少 <limit>，已跳过")
                    continue
                try:
                    lo = float(limit.get('lower'))
                    hi = float(limit.get('upper'))
                except (TypeError, ValueError):
                    self.get_logger().warn(f"关节 {j.get('name')} 的 limit 无法解析，已跳过")
                    continue

                self.joints.append(j.get('name'))
                self.lower_deg.append(math.degrees(lo))
                self.upper_deg.append(math.degrees(hi))

            if not self.joints:
                raise RuntimeError('URDF 中没有找到带 <limit> 的可动关节')

        self.rad = [0.0] * len(self.joints)

        self.get_logger().info(f'共 {len(self.joints)} 个关节: {self.joints}')
        for name, lo, hi in zip(self.joints, self.lower_deg, self.upper_deg):
            self.get_logger().info(f'  {name}: {lo:+.1f}° ~ {hi:+.1f}°')
        self.get_logger().info('等待 /joint_states 同步真实位置，同步前滑块禁用…')

    def _on_sync_timeout(self):
        if not self.synced:
            self.get_logger().warn(
                f'{SYNC_WARN_SEC:.0f} 秒未收到 /joint_states，滑块保持禁用。'
                '请确认 joint_state_broadcaster 已正常启动。')
        self._warn_timer.cancel()

    # ── 首次真实反馈：用它初始化滑块（弧度 → 度 显示）──
    def _on_state(self, msg: JointState):
        if self.synced:
            return

        positions = {}
        for i, name in enumerate(self.joints):
            if name in msg.name:
                j = msg.name.index(name)
                if j < len(msg.position):
                    positions[i] = msg.position[j]

        # rclpy.spin 在独立线程，不能直接操作 tkinter 控件，
        # 用 after(0) 把 UI 更新排入 tkinter 主线程
        if self.root is not None:
            self.root.after(0, lambda: self._apply_sync(positions))
        self.synced = True

    def _apply_sync(self, positions):
        for i, rad in positions.items():
            # self.rad 既用于显示，也是直接发布的采样值：
            # 对齐真实位置后首帧不会跳变（bridge 侧还有限速兜底）
            self.rad[i] = rad
            # 必须先解除禁用、再设值：
            # DISABLED 状态下 set() 在部分 Tk 版本上不生效，把手会停留在初始 0
            self.scales[i].config(state=tk.NORMAL)
            self.scales[i].set(math.degrees(rad))
            self.labels[i].config(fg='black')
            self._refresh_label(i)
        self.get_logger().info('滑块已同步电机真实位置，可以拖动')

    # ── 拖动回调：界面是度，内部转弧度 ──
    def _on_slide(self, i, deg_val):
        self.rad[i] = math.radians(float(deg_val))
        self._refresh_label(i)

    def _refresh_label(self, i):
        self.labels[i].config(
            text=f'{self.joints[i]}: {math.degrees(self.rad[i]):+7.2f}°')

    def _build_ui(self):
        self.root = tk.Tk()
        self.root.title('Joint Sliders (deg) — 同步真实电机位置')

        for i, name in enumerate(self.joints):
            lab = tk.Label(self.root, text=f'{name}: 等待同步…',
                           font=('Consolas', 11), anchor='w', fg='gray')
            lab.pack(fill='x', padx=8, pady=(6, 0))
            self.labels.append(lab)

            s = tk.Scale(self.root,
                         from_=self.lower_deg[i], to=self.upper_deg[i],
                         resolution=0.1, orient=tk.HORIZONTAL, length=420,
                         state=tk.DISABLED,
                         command=lambda v, i=i: self._on_slide(i, v))
            s.set(0.0)
            s.pack(fill='x', padx=8)
            self.scales.append(s)

    def _publish_loop(self):
        def tick():
            # 同步完成前不发布任何指令：
            # 此时 self.rad 全为 0，若发布会把机械臂猛地拽向 0 位
            if self.synced:
                # 只做「采样 + 发布」：直接发滑块真值。
                # 一把甩到 ±150° 也没关系 —— 平滑爬升由 real_rviz_bridge 的限速负责。
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = list(self.joints)
                msg.position = list(self.rad)      # 弧度，符合 ROS 标准
                self.pub.publish(msg)
            self.root.after(int(1000 / self.publish_rate), tick)
        tick()


def main():
    rclpy.init()
    node = SliderGuiDeg()

    if tk is None:
        node.get_logger().error(
            '未找到 tkinter，请先安装: sudo apt install python3-tk')
        rclpy.shutdown()
        return

    node._build_ui()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    node._publish_loop()

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
