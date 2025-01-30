from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_rgbd": "true",
            "enable_sync": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "depth_module.depth_profile": "848x480x15",
            "depth_module.color_profile": "848x480x15",
            "initial_reset": "true",
        }.items(),
    )

    micro_ros_node = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen',
            emulate_tty=True
        )
    

    mason_navigation_node = Node(
        package='mason_navigation',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        emulate_tty=True
    )

    mason_camera_node = Node(
        package='mason_camera',
        executable='camera_node',
        name='camera_node',
        output='screen',
        emulate_tty=True
    )

    mason_flask_node = Node(
        package='mason_flask',
        executable='flask_server',
        name='flask_server',
        output='screen',
        emulate_tty=True
    )

    mason_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("mason_control"), "launch", "mason_control.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "namespace": "mason",
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            realsense_launch,
            mason_control_launch,
            micro_ros_node,
            mason_navigation_node,
            mason_camera_node,
            mason_flask_node
        ]
    )
