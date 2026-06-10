"""Launch the GripperWebDashboardNode as a gripper_service client.

Example:

  ros2 launch dsr_gripper_tcp web_dashboard_node.launch.py \\
      gripper_service_ns:=/gripper_service \\
      web_port:=5000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('gripper_service_ns', default_value='/gripper_service'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='5000'),
        DeclareLaunchArgument('joint_name', default_value='rh_p12_rn'),
        DeclareLaunchArgument('position_max', default_value='1150'),
        DeclareLaunchArgument('move_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument('command_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument('service_wait_timeout_sec', default_value='2.0'),
    ]

    dashboard_node = Node(
        package='dsr_gripper_tcp',
        executable='web_dashboard_node',
        name='gripper_web_dashboard',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gripper_service_ns': LaunchConfiguration('gripper_service_ns'),
            'web_host': LaunchConfiguration('web_host'),
            'web_port': LaunchConfiguration('web_port'),
            'joint_name': LaunchConfiguration('joint_name'),
            'position_max': LaunchConfiguration('position_max'),
            'move_timeout_sec': LaunchConfiguration('move_timeout_sec'),
            'command_timeout_sec': LaunchConfiguration('command_timeout_sec'),
            'service_wait_timeout_sec': LaunchConfiguration('service_wait_timeout_sec'),
        }],
    )

    return LaunchDescription([*args, dashboard_node])
