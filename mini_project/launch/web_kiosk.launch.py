# user web 주문 키오스크 백엔드(FastAPI+rclpy)를 실행하는 launch. 로봇과 분리 실행 가능.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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

    # 이전 세션에서 포트를 점유 중인 프로세스를 먼저 정리한다.
    kill_port = ExecuteProcess(
        name='kill_port',
        cmd=['bash', '-c',
             'fuser -k ${KIOSK_PORT}/tcp 2>/dev/null; sleep 0.3; true'],
        additional_env={'KIOSK_PORT': LaunchConfiguration('kiosk_port')},
        output='screen',
    )

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

    # kill_port 종료 후 backend·browser 시작
    start_after_kill = RegisterEventHandler(
        OnProcessExit(target_action=kill_port, on_exit=[backend_proc, browser])
    )

    return LaunchDescription(args + [kill_port, start_after_kill])
