import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_path = get_package_share_directory('e0509_gripper_description')
    rh_pkg_path = get_package_share_directory('rh_p12_rn_a_description')
    dsr_pkg_path = get_package_share_directory('dsr_description2')
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'e0509_with_gripper.urdf.xacro')

    ARGUMENTS = [
        DeclareLaunchArgument('name', default_value='e0509_gripper'),
    ]

    # Gazebo 리소스 경로
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
            ':',
            os.path.dirname(rh_pkg_path),
            ':',
            os.path.dirname(dsr_pkg_path),
        ]
    )

    # Robot description
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_gazebo:=true',
        ' namespace:=', LaunchConfiguration('name'),
    ])

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items(),
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='screen',
        parameters=[robot_description],
    )

    # Spawn robot
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=LaunchConfiguration('name'),
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'e0509_gripper',
            '-allow_renaming', 'true',
        ],
    )

    # Spawners (without --param-file)
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        )]
    )

    arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['joint_trajectory_controller', '-c', 'controller_manager'],
        )]
    )

    gripper_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['gripper_controller', '-c', 'controller_manager'],
        )]
    )

    return LaunchDescription(ARGUMENTS + [
        gz_resource_path,
        gazebo,
        robot_state_pub,
        gz_spawn,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
