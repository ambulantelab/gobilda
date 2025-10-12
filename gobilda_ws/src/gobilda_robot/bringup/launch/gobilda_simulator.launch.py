import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Start robot with mock hardware mirroring command to its states.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=PathJoinSubstitution([
                    FindPackageShare('gobilda_robot'),
                    'config',
                    'ekf.yaml',
                ])
        )
    )
    
    declared_arguments.append (
        DeclareLaunchArgument(
            'simulation', default_value='true',
            description='Set to true to launch Gazebo Ignition simulation'
        )
    )

    use_sim_time = LaunchConfiguration('simulation')

    pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    ekf_params_file = LaunchConfiguration('ekf_params_file')

    # Start the gz simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),

        launch_arguments={
            'gz_args': '--render-engine ogre2 -r empty.sdf',
        }.items()
    )

    # Spawn robot into Gazebo
    simulated_robot = Node(
        package='ros_gz_sim',
        executable='create',
                
        arguments=[
            '-name', 'gobilda',
            '-topic', 'robot_description'
        ],
        
        output='screen'
    )
    
    # Grab the FoxGlove launch file
    foxglove = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('foxglove_bridge'),
                    'launch',
                    'foxglove_bridge_launch.xml',
                ])
            ]),
    )

    robot_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            output='screen',
            parameters=[ekf_params_file],
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('gobilda_robot'), 'urdf', 'gobilda.urdf.xacro']
            ),
            ' ',
            'use_mock_hardware:=',
            use_mock_hardware,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('gobilda_robot'),
            'config',
            'gobilda_controllers.yaml',
        ]
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_controllers],
        output='both',
        remappings=[
            ('~/robot_description', '/robot_description'),
            ('/gobilda_base_controller/cmd_vel', '/gobilda/cmd_vel'),
        ],
        # uncomment for debugging
        # arguments=[ '--ros-args', '--log-level', 'debug', ],
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 
                   '--controller-manager',
                   '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gobilda_base_controller', 
                   '--controller-manager',
                   '/controller_manager',],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        robot_localization,
        simulated_robot,
    ]
    
    # Ordering here has some effects on the startup timing of the nodes
    return LaunchDescription(declared_arguments + [gz_sim] + [foxglove] + nodes)