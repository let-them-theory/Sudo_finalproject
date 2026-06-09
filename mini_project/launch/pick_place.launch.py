# RealSense Pick & Place 전체 노드와 그리퍼 TCP 브릿지를 실행하는 launch 파일

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


ARGUMENTS = [
    DeclareLaunchArgument('mode',  default_value='virtual',
                          description='virtual | real'),

    DeclareLaunchArgument('host',  default_value='110.120.1.50',

                          description='로봇 IP (real 모드)'),
    DeclareLaunchArgument('port',  default_value='12345',
                          description='로봇 포트'),
    DeclareLaunchArgument('model', default_value='e0509',
                          description='Doosan 모델명'),
    DeclareLaunchArgument('color', default_value='white',
                          description='로봇 색상'),
    DeclareLaunchArgument('robot_name', default_value='dsr01',
                          description='Doosan ROS namespace/name'),
    DeclareLaunchArgument('robot_base_frame', default_value='base_link',
                          description='로봇 기준 base frame'),
    DeclareLaunchArgument('use_realsense', default_value='true',
                          description='RealSense 카메라 노드 실행 여부'),
    DeclareLaunchArgument('camera_serial', default_value='',
                          description='RealSense 시리얼 번호'),
    DeclareLaunchArgument('cam_tf_x',  default_value='0.5'),
    DeclareLaunchArgument('cam_tf_y',  default_value='0.0'),
    DeclareLaunchArgument('cam_tf_z',  default_value='0.6'),
    DeclareLaunchArgument('cam_tf_qx', default_value='0.0'),
    DeclareLaunchArgument('cam_tf_qy', default_value='0.707'),
    DeclareLaunchArgument('cam_tf_qz', default_value='0.0'),
    DeclareLaunchArgument('cam_tf_qw', default_value='0.707'),
    DeclareLaunchArgument('gui', default_value='true',
                          description='PyQt GUI 실행 여부'),
    DeclareLaunchArgument('use_launch_set_robot_mode', default_value='false',
                          description='Fallback: launch에서 set_robot_mode service call 실행 여부'),
    DeclareLaunchArgument('gripper_tcp_port', default_value='20002',
                          description='컨트롤러 DRL 그리퍼 TCP 서버 포트'),
    DeclareLaunchArgument('use_ultrasonic', default_value='true',
                          description='아두이노 HC-SR04 초음파 거리 노드 실행 여부'),
    DeclareLaunchArgument('ultrasonic_port', default_value='/dev/ttyACM0',
                          description='아두이노 시리얼 포트 (보통 /dev/ttyACM0 또는 /dev/ttyUSB0)'),
    DeclareLaunchArgument('ultrasonic_baudrate', default_value='9600',
                          description='아두이노 시리얼 baudrate (현장 스케치: 9600)'),
    DeclareLaunchArgument('robot_ready_timeout_sec', default_value='120',
                          description='ros2_control/DRL 서비스 대기 최대 시간(초)'),
    DeclareLaunchArgument('gripper_ready_timeout_sec', default_value='90',
                          description='gripper_service ready 대기 최대 시간(초)'),
]


def generate_launch_description():

    pkg_this = get_package_share_directory('dsr_realsense_pick_place')
    params_file = os.path.join(pkg_this, 'config', 'pick_place_params.yaml')
    wait_robot_script = os.path.join(pkg_this, 'scripts', 'wait_for_robot_ready.sh')
    wait_gripper_script = os.path.join(pkg_this, 'scripts', 'wait_for_gripper_ready.py')
    launch_cleanup_script = os.path.join(pkg_this, 'scripts', 'launch_cleanup.sh')

    doosan_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('dsr_bringup2'),
            '/launch/dsr_bringup2_rviz.launch.py'
        ]),
        launch_arguments={
            'mode':  LaunchConfiguration('mode'),
            'host':  LaunchConfiguration('host'),
            'port':  LaunchConfiguration('port'),
            'model': LaunchConfiguration('model'),
            'color': LaunchConfiguration('color'),
            'name':  LaunchConfiguration('robot_name'),
        }.items(),
    )

    set_robot_mode = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    ["/", LaunchConfiguration('robot_name'), '/system/set_robot_mode'],
                    'dsr_msgs2/srv/SetRobotMode',
                    '{robot_mode: 1}',
                ],
                output='screen',
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_launch_set_robot_mode')),
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable':  'true',
            'serial_no':          LaunchConfiguration('camera_serial'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_realsense')),
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_tf',
        arguments=[
            LaunchConfiguration('cam_tf_x'),
            LaunchConfiguration('cam_tf_y'),
            LaunchConfiguration('cam_tf_z'),
            LaunchConfiguration('cam_tf_qx'),
            LaunchConfiguration('cam_tf_qy'),
            LaunchConfiguration('cam_tf_qz'),
            LaunchConfiguration('cam_tf_qw'),
            LaunchConfiguration('robot_base_frame'),
            'camera_color_optical_frame',
        ],
        output='screen',
    )

    object_detector = Node(
        package='dsr_realsense_pick_place',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[params_file, {
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        }],
    )

    gui_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='dsr_realsense_pick_place',
                executable='gui_node',
                name='pick_place_gui',
                output='screen',
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration('gui')),
            )
        ],
    )

    # ── 이벤트 기반 그리퍼 / pick_place 기동 ─────────────────────────────
    # 1) wait_for_robot_ready (drl_start) → 2) gripper 노드 →
    # 3) wait_for_gripper_ready (state.ready) → 4) pick_place_node

    wait_robot_ready = ExecuteProcess(
        cmd=[
            'bash', wait_robot_script,
            LaunchConfiguration('robot_name'),
            LaunchConfiguration('robot_ready_timeout_sec'),
        ],
        output='screen',
        name='wait_for_robot_ready',
    )

    gripper_service_node = Node(
        package='dsr_gripper_tcp',
        executable='gripper_service_node',
        name='gripper_service',
        output='screen',
        parameters=[{
            'controller_host': LaunchConfiguration('host'),
            'tcp_port': LaunchConfiguration('gripper_tcp_port'),
            'namespace': LaunchConfiguration('robot_name'),
            'goal_current': 400,
            'profile_velocity': 1500,
            'profile_acceleration': 1000,
            'connect_timeout_sec': 60.0,
            'post_drl_start_sleep_sec': 2.0,
            'drl_idle_stable_sec': 2.0,
            'tcp_server_open_retry_sec': 0.5,
            'init_attempts': 5,
            'init_timeout_sec': 20.0,
            'init_retry_delay_sec': 1.0,
        }],
        # boot_bridge(최대 수십 초) 동안 DrlStop 정리 시간 확보
        sigterm_timeout='20',
        sigkill_timeout='5',
    )

    gripper_wrapper_node = Node(
        package='dsr_realsense_pick_place',
        executable='gripper_node',
        name='rh_p12_rna_gripper',
        output='screen',
        parameters=[params_file, {
            'robot_ns': LaunchConfiguration('robot_name'),
        }],
        sigterm_timeout='10',
        sigkill_timeout='3',
    )

    wait_gripper_ready = ExecuteProcess(
        cmd=[
            'python3', wait_gripper_script,
            LaunchConfiguration('gripper_ready_timeout_sec'),
        ],
        output='screen',
        name='wait_for_gripper_ready',
    )

    pick_place_node = Node(
        package='dsr_realsense_pick_place',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[params_file, {
            'robot_namespace': LaunchConfiguration('robot_name'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
        }],
    )

    start_gripper_after_robot_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_robot_ready,
            on_exit=[gripper_service_node, gripper_wrapper_node],
        ),
    )

    start_gripper_ready_wait = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gripper_service_node,
            on_start=[wait_gripper_ready],
        ),
    )

    start_pick_place_after_gripper_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_gripper_ready,
            on_exit=[pick_place_node],
        ),
    )

    # Ctrl+C 시 이벤트로 기동된 gripper/pick_place가 고아 프로세스로 남는 문제 방지
    cleanup_on_shutdown = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['bash', launch_cleanup_script],
                    output='screen',
                    name='launch_cleanup',
                ),
            ],
        ),
    )

    ultrasonic = Node(
        package='dsr_realsense_pick_place',
        executable='ultrasonic_node',
        name='ultrasonic_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('ultrasonic_port'),
            'baudrate': LaunchConfiguration('ultrasonic_baudrate'),
        }],
        condition=IfCondition(LaunchConfiguration('use_ultrasonic')),
    )

    return LaunchDescription(ARGUMENTS + [
        doosan_bringup,
        set_robot_mode,
        realsense_node,
        static_tf,
        object_detector,
        gui_node,
        ultrasonic,
        wait_robot_ready,
        start_gripper_after_robot_ready,
        start_gripper_ready_wait,
        start_pick_place_after_gripper_ready,
        cleanup_on_shutdown,
    ])
