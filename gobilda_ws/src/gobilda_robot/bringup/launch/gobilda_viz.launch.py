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

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
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

    # Grab the Camera launch file
    # Path to local camera configure file
    
    # TODO: learn how to add my own config 
    camera_file = PathJoinSubstitution([
            FindPackageShare('depthai_ros_driver'),
            'config',
            'segmentation.yaml',
    ])
    oakd_camera= IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('depthai_ros_driver'),
                    'launch',
                    'camera_as_part_of_a_robot.launch.py',
                ])
            ]),

            launch_arguments={
                'name': 'oakd',
                'parent_frame': 'base_link',
                'publish_tf_from_calibration': 'false',
                'imu_from_descr': 'false',
                # 'params_file': camera_file,
            }.items()
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("gobilda_robot"), "urdf", "gobilda.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("gobilda_robot"),
            "config",
            "gobilda_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/gobilda_base_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "--controller-manager",
                   "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gobilda_base_controller", 
                   "--controller-manager",
                   "/controller_manager"],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    launch_files = [
        foxglove,
        rplidar,
        oakd_camera,
    ]
    
    return LaunchDescription(declared_arguments + nodes + launch_files)