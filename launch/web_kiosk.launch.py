# user web 주문 키오스크 백엔드(FastAPI+rclpy)를 실행하는 launch. 로봇과 분리 실행 가능.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('dsr_realsense_pick_place')
    backend = os.path.join(pkg, 'web_kiosk', 'backend', 'main.py')

    args = [
        DeclareLaunchArgument('kiosk_port', default_value='8000',
                              description='웹 키오스크 포트'),
        DeclareLaunchArgument('open_browser', default_value='false',
                              description='현장 키오스크면 true — chrome 앱창 자동 실행'),
    ]

    backend_proc = ExecuteProcess(
        name='kiosk_backend',
        cmd=['python3', backend],
        additional_env={'KIOSK_PORT': LaunchConfiguration('kiosk_port')},
        output='screen',
    )

    browser = ExecuteProcess(
        name='kiosk_browser',
        cmd=['google-chrome', '--new-window',
             ['--app=http://localhost:', LaunchConfiguration('kiosk_port')]],
        condition=IfCondition(LaunchConfiguration('open_browser')),
        output='screen',
    )

    return LaunchDescription(args + [backend_proc, browser])
