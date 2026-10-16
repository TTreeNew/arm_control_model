from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ─── Launch Configuration（可在命令行覆盖） ───
    # 转接板有两个串口：
    #   /dev/ttyACM0 = 与电机通信（控制用，默认）
    #   /dev/ttyACM1 = 固件更新用，接上去电机不会响应，勿用于控制
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')

    # ─── 控制器配置 YAML ───
    controller_yaml = PathJoinSubstitution([
        FindPackageShare('real_arm_config'),
        'config', 'ros2_controllers.yaml'
    ])

    # ─── 重力补偿参数 YAML（独立文件）───
    # 这些参数归属 panda_arm_controller，所以必须和 controller_yaml 一起，
    # 被【ros2_control_node 的 parameters】和【spawner 的 --param-file】同时加载。
    # 只加一处 → 参数静默取默认值：不报错，但重力补偿不生效。
    # 验证：ros2 param get /panda_arm_controller gravity_scale
    gravity_params = PathJoinSubstitution([
        FindPackageShare('real_arm_config'),
        'config', 'gravity_params.yaml'
    ])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('real_arm_config'),
        'config', 'display.rviz'
    ])

    # ─── 桥接节点参数 YAML ───
    # 包含 publish_rate / max_velocity / step_duration / enabled，
    # 改参数只需改这些yaml文件 + colcon build，无需改动 C++ 代码
    bridge_params = PathJoinSubstitution([
        FindPackageShare('real_arm_config'),
        'config', 'bridge_params.yaml'
    ])

    # ─── Robot Description (URDF via xacro) ───
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('real_arm_config'),
                'config', 'panda_arm.urdf.xacro'
            ]),
            ' serial_port:=', serial_port,
        ]),
        value_type=str
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    # ─── 发布静态 TF: world -> base_link ───
    static_tf_world_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen',
    )

    # ─── robot_state_publisher ───
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': False}],
        output='screen'
    )

    # ─── ros2_control_node ───
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controller_yaml,
            gravity_params,
            {'use_sim_time': False},
        ],
        output='screen'
    )

    # ─── 加载控制器 ───
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_yaml,
        ],
        output='screen',
    )

    spawn_arm_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'panda_arm_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controller_yaml,
            '--param-file', gravity_params,
        ],
        output='screen',
    )

    # ─── 滑块 GUI（以「度」显示）───
    # 与 joint_state_publisher_gui 的区别：
    #   1) 读写单位都是【度】，内部仍转换成弧度发布，符合 ROS 标准
    #   2) 限位从 robot_description 解析，改 URDF 后滑块范围自动跟随
    #   3) 启动后订阅 /joint_states 用电机真实位置初始化，同步前禁用滑块
    slider_gui_node = Node(
        condition=IfCondition(show_gui),
        package='panda_arm_tools',
        executable='slider_gui_deg.py',
        name='slider_gui_deg',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ─── Rviz2滑块 → 机械臂轨迹桥接 ───
    real_rviz_bridge_node = Node(
        package='panda_arm_hardware',
        executable='real_rviz_bridge',
        name='real_rviz_bridge',
        parameters=[bridge_params],
        output='screen',
    )

    # ─── RViz2 ───
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False},
        ],
        output='screen'
    )

    # ─── 启动顺序事件链 ───
    # 1. ros2_control_node → spawn_joint_state_broadcaster
    # 2. spawn_joint_state_broadcaster → spawn_arm_group_controller
    # 3. spawn_arm_group_controller → real_rviz_bridge
    # 4. real_rviz_bridge → rviz2

    start_spawn_broadcaster_after_control = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[spawn_joint_state_broadcaster]
        )
    )

    start_spawn_controller_after_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_arm_group_controller]
        )
    )

    start_bridge_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_group_controller,
            on_exit=[real_rviz_bridge_node]
        )
    )

    # 滑块同样挂在 controller 之后：此刻 joint_state_broadcaster
    # 已在稳定发布 /joint_states，滑块启动后能立刻拿到真实位置完成同步、
    # 解除禁用，避免"停在 0° 被误拖导致机械臂猛拽"。
    start_slider_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_arm_group_controller,
            on_exit=[slider_gui_node]
        )
    )

    start_rviz_after_bridge = RegisterEventHandler(
        # 注意：real_rviz_bridge 是常驻节点（内部 rclcpp::spin，不主动退出），
        # 因此必须用 OnProcessStart 在其启动后立即拉起 RViz；
        # 若沿用 OnProcessExit，则 RViz 永远不会启动（除非 bridge 异常崩溃）。
        OnProcessStart(
            target_action=real_rviz_bridge_node,
            on_start=[rviz_node]
        )
    )

    return LaunchDescription([
        gui_arg,
        static_tf_world_to_base,
        robot_state_publisher_node,
        ros2_control_node,
        start_spawn_broadcaster_after_control,
        start_spawn_controller_after_broadcaster,
        start_bridge_after_controller,
        start_slider_after_controller,
        start_rviz_after_bridge,
    ])
