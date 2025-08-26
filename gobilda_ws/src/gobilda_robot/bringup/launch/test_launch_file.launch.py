from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
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
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_id",
            default_value="rplidar_link",
            description="Frame id for the lidar link. Name found in the URDF.",
        )
    )
    laser_frame_id = LaunchConfiguration("frame_id")
    
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

    # Grab the RPLidar launch file
    rplidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_a1_launch.py',
                ])
            ]),

           launch_arguments={
                'frame_id': laser_frame_id,
                'serial_port': '/dev/rplidar',
                'serial_baudrate': '115200',
            }.items(),
    )

   
    # Add the KISS-ICP Node for LiDAR based odometry
    # Before KISS-ICP node we need to convert from Laser --> PointCloud message
    laser_to_pointcloud = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        remappings=[
            ('scan_in', 'scan'),
        ],
    )
    
    nodes = [
        laser_to_pointcloud,
    ]

    launch_files = [
        foxglove,
        rplidar,
    ]
    
    return LaunchDescription(declared_arguments + launch_files + nodes)