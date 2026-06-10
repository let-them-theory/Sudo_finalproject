"""
cuRobo Vision-based Control Launch File (Grounding DINO)

cuRobo planner를 먼저 실행하고, 로딩 완료 후 object tracking을 시작합니다.

Usage:
    ros2 launch e0509_gripper_description curobo_vision.launch.py
    ros2 launch e0509_gripper_description curobo_vision.launch.py \
        prompt:="cup . bottle . pen"

Keys (object tracking 화면):
    s: 선택된 물체 위치로 이동
    p: 선택된 물체 pick
    1-9: 감지된 물체 선택 (lock)
    r: 선택 해제 (unlock)
    q: 종료
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction


def generate_launch_description():

    ARGUMENTS = [
        DeclareLaunchArgument('calibration_path',
            default_value=os.path.expanduser('~/sim2real/sim2real/config/calibration_eye_to_hand.npz'),
            description='Path to calibration .npz file'),
        DeclareLaunchArgument('prompt',
            default_value='red block . green block . yellow block',
            description='Text prompt for Grounding DINO'),
        DeclareLaunchArgument('box_threshold', default_value='0.3',
            description='Detection confidence threshold'),
    ]

    # cuRobo Planner Node (starts first)
    curobo_planner = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'export CUDA_HOME=/usr/local/cuda-12.8 && '
            'source /opt/ros/humble/setup.bash && '
            'source ~/doosan_ws/install/setup.bash && '
            'python3 ~/doosan_ws/src/e0509_gripper_description/scripts/curobo_planner_node.py'
        ],
        output='screen',
    )

    # Object Tracking Node (delayed 30s to let cuRobo finish JIT)
    object_tracking = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'export CUDA_HOME=/usr/local/cuda-12.8 && '
                    'source /opt/ros/humble/setup.bash && '
                    'source ~/doosan_ws/install/setup.bash && '
                    'python3 ~/doosan_ws/src/e0509_gripper_description/scripts/object_tracking_node.py'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription(ARGUMENTS + [
        curobo_planner,
        object_tracking,
    ])
