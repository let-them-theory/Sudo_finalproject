# 그리퍼 TCP 서비스 노드 실행 파라미터를 구성하는 launch 파일

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('controller_host', default_value='110.120.1.50'),
        DeclareLaunchArgument('tcp_port', default_value='20002'),
        DeclareLaunchArgument('namespace', default_value='dsr01'),
        DeclareLaunchArgument('service_prefix', default_value=''),
        DeclareLaunchArgument('skip_set_autonomous', default_value='false'),
        DeclareLaunchArgument('initialize_on_start', default_value='true'),
        DeclareLaunchArgument('goal_current', default_value='400'),
        DeclareLaunchArgument('profile_velocity', default_value='1500'),
        DeclareLaunchArgument('profile_acceleration', default_value='1000'),
        DeclareLaunchArgument('poll_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('joint_name', default_value='rh_p12_rn'),
        DeclareLaunchArgument('position_max', default_value='1150'),
        DeclareLaunchArgument('default_move_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument('default_safe_grasp_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('safe_grasp_feedback_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('grasp_current_threshold', default_value='300'),
        DeclareLaunchArgument('object_lost_current_threshold', default_value='80'),
        DeclareLaunchArgument('object_lost_position_delta', default_value='80'),
        DeclareLaunchArgument('state_poll_timeout_sec', default_value='2.0'),
        DeclareLaunchArgument('command_retry_count', default_value='1'),
        DeclareLaunchArgument('connect_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('post_drl_start_sleep_sec', default_value='2.0'),
        DeclareLaunchArgument('stop_existing_drl', default_value='true'),
        DeclareLaunchArgument('drl_stop_mode', default_value='1'),
        DeclareLaunchArgument('drl_stop_settle_sec', default_value='5.0'),
        DeclareLaunchArgument('drl_idle_stable_sec', default_value='2.0'),
        DeclareLaunchArgument('drl_start_retry_count', default_value='3'),
        DeclareLaunchArgument('drl_start_retry_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('tcp_server_open_retry_sec', default_value='0.5'),
        DeclareLaunchArgument('init_attempts', default_value='10'),
        DeclareLaunchArgument('init_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('init_retry_delay_sec', default_value='2.0'),
    ]

    service_node = Node(
        package='dsr_gripper_tcp',
        executable='gripper_service_node',
        name='gripper_service',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'controller_host': LaunchConfiguration('controller_host'),
            'tcp_port': LaunchConfiguration('tcp_port'),
            'namespace': LaunchConfiguration('namespace'),
            'service_prefix': LaunchConfiguration('service_prefix'),
            'skip_set_autonomous': LaunchConfiguration('skip_set_autonomous'),
            'initialize_on_start': LaunchConfiguration('initialize_on_start'),
            'goal_current': LaunchConfiguration('goal_current'),
            'profile_velocity': LaunchConfiguration('profile_velocity'),
            'profile_acceleration': LaunchConfiguration('profile_acceleration'),
            'poll_rate_hz': LaunchConfiguration('poll_rate_hz'),
            'joint_name': LaunchConfiguration('joint_name'),
            'position_max': LaunchConfiguration('position_max'),
            'default_move_timeout_sec': LaunchConfiguration('default_move_timeout_sec'),
            'default_safe_grasp_timeout_sec': LaunchConfiguration('default_safe_grasp_timeout_sec'),
            'safe_grasp_feedback_rate_hz': LaunchConfiguration('safe_grasp_feedback_rate_hz'),
            'grasp_current_threshold': LaunchConfiguration('grasp_current_threshold'),
            'object_lost_current_threshold': LaunchConfiguration('object_lost_current_threshold'),
            'object_lost_position_delta': LaunchConfiguration('object_lost_position_delta'),
            'state_poll_timeout_sec': LaunchConfiguration('state_poll_timeout_sec'),
            'command_retry_count': LaunchConfiguration('command_retry_count'),
            'connect_timeout_sec': LaunchConfiguration('connect_timeout_sec'),
            'post_drl_start_sleep_sec': LaunchConfiguration('post_drl_start_sleep_sec'),
            'stop_existing_drl': LaunchConfiguration('stop_existing_drl'),
            'drl_stop_mode': LaunchConfiguration('drl_stop_mode'),
            'drl_stop_settle_sec': LaunchConfiguration('drl_stop_settle_sec'),
            'drl_idle_stable_sec': LaunchConfiguration('drl_idle_stable_sec'),
            'drl_start_retry_count': LaunchConfiguration('drl_start_retry_count'),
            'drl_start_retry_delay_sec': LaunchConfiguration('drl_start_retry_delay_sec'),
            'tcp_server_open_retry_sec': LaunchConfiguration('tcp_server_open_retry_sec'),
            'init_attempts': LaunchConfiguration('init_attempts'),
            'init_timeout_sec': LaunchConfiguration('init_timeout_sec'),
            'init_retry_delay_sec': LaunchConfiguration('init_retry_delay_sec'),
        }],
    )

    return LaunchDescription([*args, service_node])
