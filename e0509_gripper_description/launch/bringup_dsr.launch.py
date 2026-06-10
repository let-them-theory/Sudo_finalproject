from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='dsr01',     description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1', description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345',     description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual',   description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='e0509',     description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white',     description='ROBOT_COLOR'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
        DeclareLaunchArgument('standalone', default_value='true', description='Run common nodes?'),
    ]

    mode = LaunchConfiguration('mode')

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

    # Gripper Joint State Publisher
    gripper_joint_pub_node = Node(
        package='e0509_gripper_description',
        executable='gripper_joint_publisher.py',
        name='gripper_joint_publisher',
        namespace=LaunchConfiguration('name'),
        output='screen',
    )

    # Gripper Service Node (ROS2 서비스로 그리퍼 제어)
    gripper_service_node = Node(
        package='e0509_gripper_description',
        executable='gripper_service_node.py',
        name='gripper_service_node',
        namespace=LaunchConfiguration('name'),
        parameters=[{'mode': LaunchConfiguration('mode')}],
        output='screen',
    )

    nodes = [
        run_emulator_node,
        gripper_joint_pub_node,
        gripper_service_node,
    ]

    return LaunchDescription(ARGUMENTS + nodes)
