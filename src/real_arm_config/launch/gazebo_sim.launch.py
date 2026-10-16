"""
Gazebo Classic 11 仿真启动文件（与真机的 slider_control.launch.py 并行的一套）。

区别：
  1. URDF 用 config/panda_arm.sim.urdf.xacro
  2. 参数文件用 config/sim_controllers.yaml
  3. 不创建 ros2_control_node—— 仿真里 controller_manager 由
     libgazebo_ros2_control.so 在 gzserver 进程内自己创建（在.gazebo 里执行）。

其他完全复用： robot_state_publisher 、 joint_state_broadcaster 、 PandaArmController 、
          real_rviz_bridge 、 slider_gui_deg、RViz2 。

启动顺序（有硬约束）：
  robot_state_publisher 必须先于 spawn_entity ，因为 gazebo_ros2_control 插件是调用 robot_state_publisher 的
  get_parameters 服务来拿 robot_description 的，拿不到会每 100 ms 重试、一直阻塞。

⚠️ 为什么不把 gzserver 设成 pause:=true 等控制器激活后再放行：
  暂停时仿真时间不推进 → 插件挂的 world update 回调里 sim_period 恒为 0
  → controller_manager::update() 从不执行 → 控制器的 update() 也不执行
  → ros2_control 的 switch_controller 会等不到激活完成，
     5 秒后报 "Switch controller timed out" 并让 spawner 全部失败。
  所以必须让物理正常跑。副作用：控制器激活前的零点几秒里没有力矩，
  机械臂会在重力下塌到限位 —— 这是正常的，激活后用滑块把它拖起来即可。

无头调试：
  ros2 launch real_arm_config gazebo_sim.launch.py gui:=false rviz:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ─── Launch 参数 ───
    gui = LaunchConfiguration('gui')      
    rviz = LaunchConfiguration('rviz')

    # ─── 仿真控制器参数───
    sim_controllers = PathJoinSubstitution([
        FindPackageShare('real_arm_config'), 'config', 'sim_controllers.yaml'
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('real_arm_config'), 'config', 'display.rviz'
    ])

    bridge_params = PathJoinSubstitution([
        FindPackageShare('real_arm_config'), 'config', 'bridge_params.yaml'
    ])

    # ─── Robot Description（仿真版 URDF via xacro）───
    # ★ 传入参数 sim_controllers 覆盖 ros2_controllers_file 里的默认值：
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('real_arm_config'),
                'config', 'panda_arm.sim.urdf.xacro'
            ]),
            ' ros2_controllers_file:=', sim_controllers,
        ]),
        value_type=str
    )

    # ─── Gazebo（Classic 11）───
    # gazebo.launch.py = gzserver + gzclient；gzserver 以 -s 加载以下两个插件：
    #   libgazebo_ros_init.so    → 发布 /clock（use_sim_time 依赖它）
    #   libgazebo_ros_factory.so → 提供 /spawn_entity 服务
    # 默认世界 = gazebo_ros/worlds/empty.world（sun + ground_plane）。
    # 不要传 pause:=true，原因见文件头。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'gui': gui}.items(),
    )

    # ─── 关于 world → base_link 的 TF ───
    # 仿真里【不需要】static_transform_publisher：
    #   panda_arm.sim.urdf.xacro 里有 <link name="world"/> + world_to_base(fixed)，
    #   robot_state_publisher 会从这条关节自动发布 world→base_link。
    # 若再起一个 static_transform_publisher 发同一个 TF，就会出现两个发布者。
    # 真机的 launch 写 static_transform_publisher 是因为实物的
    # URDF 里没有 <link name="world"/> + world_to_base(fixed) 这几行。
    # <link name="world"/> + world_to_base(fixed) 和 launch 里写 static_transform_publisher 是等价的

    # ─── robot_state_publisher ───
    #  必须早于 spawn 起来（插件靠它的 get_parameters 服务取 URDF）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}],
        output='screen'
    )

    # ─── 根据 robot_description 生成机械臂模型 ───
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'panda_arm',
            '-timeout', '60.0',
        ],
        output='screen',
    )

    # ─── 加载控制器（ controller_manager 在 gzserver 进程内已经创建了，在spawn_entity的时候触发这件事）───
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--param-file', sim_controllers,
        ],
        output='screen',
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'panda_arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30.0',
            '--param-file', sim_controllers,
        ],
        output='screen',
    )

    # ─── 滑块 GUI（与真机同一个节点：读写用度、限位从 robot_description 解析）───
    # 与真机一样挂在 controller 之后：此时 /joint_states 已稳定发布，
    # 滑块启动后能立刻用当前实际位置初始化，不会停在 0° 被误拖。
    # 使用自定义滑块不用 ROS 自带的 joint_state_publisher_gui 是因为这个滑块只会加载
    # initial_positions.yaml作为滑块初始位置，不能将当前实际位置作为初始位置。这样导致启动后就会猛冲
    slider_gui_node = Node(
        condition=IfCondition(gui),
        package='panda_arm_tools',
        executable='slider_gui_deg.py',
        name='slider_gui_deg',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ─── 滑块 → 轨迹桥接（与真机共用同一份代码和参数）───
    # use_sim_time : true（真机 false）
    real_rviz_bridge_node = Node(
        package='panda_arm_hardware',
        executable='real_rviz_bridge',
        name='real_rviz_bridge',
        parameters=[bridge_params, {'use_sim_time': True}],
        output='screen',
    )

    # ─── RViz2 ───
    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    # ─── 启动顺序事件链 ───
    # RSP 即 robot_state_publisher
    # RSP → gazebo + spawn_entity → joint_state_broadcaster
    #     → panda_arm_controller → real_rviz_bridge → slider + rviz
    #
    #  RSP 与桥接都用 OnProcessStart：它们是常驻节点，不主动退出，
    #    用 OnProcessExit 的话后续动作不会启动。
    #    （同 slider_control.launch.py 里 real_rviz_bridge → rviz2 的注释。）
    start_gazebo_and_spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[gazebo, spawn_entity],
        )
    )

    start_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_joint_state_broadcaster],
        )
    )

    start_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_controller],
        )
    )

    start_bridge_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_controller,
            on_exit=[real_rviz_bridge_node],
        )
    )

    start_slider_and_rviz_after_bridge = RegisterEventHandler(
        OnProcessStart(
            target_action=real_rviz_bridge_node,
            on_start=[slider_gui_node, rviz_node],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='是否启动 gzclient 与滑块 GUI（false = 无头模式）'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='True',
            description='是否启动 RViz2（false = 只看日志）'
        ),
        robot_state_publisher_node,
        start_gazebo_and_spawn_after_rsp,
        start_broadcaster_after_spawn,
        start_controller_after_broadcaster,
        start_bridge_after_controller,
        start_slider_and_rviz_after_bridge,
    ])
