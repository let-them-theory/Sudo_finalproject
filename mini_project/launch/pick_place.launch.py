# RealSense Pick & Place 전체 노드와 그리퍼 TCP 브릿지를 실행하는 launch 파일

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    # gui(PyQt5 GUI)는 SQLite+HTTP 웹 제어로 대체되어 기본 비활성.
    # 필요 시 gui:=true 로 기존 GUI를 여전히 띄울 수 있다.
    DeclareLaunchArgument('gui', default_value='false',
                          description='PyQt GUI 실행 여부 (대체됨, 기본 false)'),
    DeclareLaunchArgument('web', default_value='true',
                          description='SQLite+HTTP 웹 제어 노드(web_control_node) 실행 여부'),
    DeclareLaunchArgument('web_host', default_value='0.0.0.0',
                          description='웹 제어 HTTP 바인드 호스트'),
    DeclareLaunchArgument('web_port', default_value='8080',
                          description='웹 제어 HTTP 포트'),
    DeclareLaunchArgument('use_launch_set_robot_mode', default_value='false',
                          description='Fallback: launch에서 set_robot_mode service call 실행 여부'),
    DeclareLaunchArgument('gripper_tcp_port', default_value='20002',
                          description='그리퍼 TCP 서버 포트'),
    DeclareLaunchArgument('use_ultrasonic', default_value='true',
                          description='아두이노 HC-SR04 초음파 거리 노드 실행 여부'),
    DeclareLaunchArgument('ultrasonic_port', default_value='/dev/ttyACM0',
                          description='아두이노 시리얼 포트'),
    DeclareLaunchArgument('ultrasonic_baudrate', default_value='9600',
                          description='아두이노 시리얼 baudrate'),
]


def generate_launch_description():

    pkg_this = get_package_share_directory('dsr_realsense_pick_place')
    params_file = os.path.join(pkg_this, 'config', 'pick_place_params.yaml')
    yolo_model_path = os.path.join(pkg_this, 'models', 'proto.pt')
    fastsam_weights_path = os.path.join(pkg_this, 'models', 'FastSAM-s.pt')

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
            'yolo_model': yolo_model_path,
            'fastsam_weights': fastsam_weights_path,
        }],
    )

    # ── 기존 PyQt5 GUI 노드 (SQLite+HTTP 웹 제어로 대체, 기본 비활성) ──────────
    # gui:=true 로 실행하면 여전히 동작한다. 아래는 주석 처리하지 않고 condition으로만
    # 끄도록 두어, 기존 동작을 그대로 보존한다(gui 기본값을 false로 변경했을 뿐).
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

    # ── 신규: SQLite + HTTP 웹 제어 노드 ──────────────────────────────────────
    # 기존 GUI가 하던 물체 선택/로봇·그리퍼 제어/파라미터 튜닝을 화면 없이
    # 데이터베이스(SQLite) + HTTP(웹 페이지/REST)로 수행한다. 기본 web:=true.
    web_control_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='dsr_realsense_pick_place',
                executable='web_control_node',
                name='web_control_node',
                output='screen',
                parameters=[params_file, {
                    'http_host': LaunchConfiguration('web_host'),
                    # web_port는 문자열 substitution이므로 int로 명시 변환(노드의 http_port는 integer).
                    'http_port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
                }],
                condition=IfCondition(LaunchConfiguration('web')),
            )
        ],
    )

    gripper = TimerAction(
        period=5.0,
        actions=[
            Node(
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
                    'poll_rate_hz': 10.0,
                }]
            ),
            Node(
                package='dsr_realsense_pick_place',
                executable='gripper_node',
                name='rh_p12_rna_gripper',
                output='screen',
                parameters=[params_file, {
                    'robot_ns': LaunchConfiguration('robot_name'),
                }],
            )
        ]
    )

    pick_place = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='dsr_realsense_pick_place',
                executable='pick_place_node',
                name='pick_place_node',
                output='screen',
                parameters=[params_file, {
                    'robot_namespace': LaunchConfiguration('robot_name'),
                    'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                }],
            )
        ]
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
        web_control_node,
        ultrasonic,
        gripper,
        pick_place,
    ])
