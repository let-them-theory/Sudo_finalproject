import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='dsr01',     description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345',     description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual',   description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='e0509',     description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white',     description='ROBOT_COLOR'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('rviz',  default_value='true',      description='Launch RViz'),
    ]

    pkg_path = get_package_share_directory('e0509_gripper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'e0509_with_gripper.urdf.xacro')

    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')

    # Robot description with all parameters
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' name:=', LaunchConfiguration('name'),
        ' host:=', LaunchConfiguration('host'),
        ' rt_host:=', LaunchConfiguration('rt_host'),
        ' port:=', LaunchConfiguration('port'),
        ' mode:=', LaunchConfiguration('mode'),
        ' model:=', LaunchConfiguration('model'),
        ' color:=', LaunchConfiguration('color'),
        ' update_rate:=100',
    ])

    robot_controllers = [
        PathJoinSubstitution([
            FindPackageShare("dsr_controller2"),
            "config",
            "dsr_controller2.yaml",
        ])
    ]

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dsr_description2"), "rviz", "default.rviz"
    ])

    # Emulator node (virtual mode only)
    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration('name'),
        parameters=[
            {"name":    LaunchConfiguration('name')},
            {"rate":    100},
            {"standby": 5000},
            {"command": True},
            {"host":    LaunchConfiguration('host')},
            {"port":    LaunchConfiguration('port')},
            {"mode":    LaunchConfiguration('mode')},
            {"model":   LaunchConfiguration('model')},
            {"gripper": "none"},
            {"mobile":  "none"},
            {"rt_host": LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output="screen",
    )

    # Controller Manager (delayed to wait for emulator)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration('name'),
        parameters=[{"robot_description": robot_description_content}] + robot_controllers,
        output="both",
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Gripper Joint State Publisher
    gripper_joint_pub_node = Node(
        package='e0509_gripper_description',
        executable='gripper_joint_publisher',
        name='gripper_joint_publisher',
        namespace=LaunchConfiguration('name'),
        output='screen',
    )

    # Gripper Service Node (ROS2 서비스로 그리퍼 제어)
    gripper_service_node = Node(
        package='e0509_gripper_description',
        executable='gripper_service_node',
        name='gripper_service_node',
        namespace=LaunchConfiguration('name'),
        parameters=[{'mode': LaunchConfiguration('mode')}],
        output='screen',
    )

    # RViz (조건부 실행)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration('name'),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
    )

    # Doosan Controller
    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration('name'),
        executable="spawner",
        arguments=[
            "dsr_controller2",
            "-c", "controller_manager",
            "--controller-manager-timeout", "120"
        ],
    )

    # Delay control_node start by 7 seconds to allow emulator to initialize (virtual mode)
    delayed_control_node = TimerAction(
        period=7.0,
        actions=[control_node]
    )

    # Wait for control_node to start, then spawn joint_state_broadcaster after 5 seconds
    delay_jsb_after_control_node = RegisterEventHandler(
        OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    # Delay controller spawner until joint_state_broadcaster is ready
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # Delay RViz and Gripper Service until controller is ready
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[rviz_node, gripper_service_node],
        )
    )

    nodes = [
        run_emulator_node,
        robot_state_pub_node,
        gripper_joint_pub_node,
        delayed_control_node,
        delay_jsb_after_control_node,
        delay_controller,
        delay_rviz,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
