from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("rover_stella_vslam")
    )

    default_camera_config = str(
        package_share / "config" / "stella_camera_corrected.yaml"
    )
    default_adapter_config = str(
        package_share / "config" / "adapter.yaml"
    )

    image_topic = LaunchConfiguration("image_topic")
    vocabulary = LaunchConfiguration("vocabulary")
    camera_config = LaunchConfiguration("camera_config")
    map_output = LaunchConfiguration("map_output")

    return LaunchDescription([
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/front/image_raw",
        ),
        DeclareLaunchArgument(
            "vocabulary",
            default_value="/home/jetson/ros2_ws/src/stella_vslam_ros/orb_vocab.fbow",
        ),
        DeclareLaunchArgument(
            "camera_config",
            default_value=default_camera_config,
        ),
        DeclareLaunchArgument(
            "map_output",
            default_value="/home/jetson/fisheye_calibration/stella_map.msg",
        ),

        Node(
            package="stella_vslam_ros",
            executable="run_slam",
            name="stella_vslam",
            output="screen",
            arguments=[
                "-v", vocabulary,
                "-c", camera_config,
                "--map-db-out", map_output,
            ],
            parameters=[{
                "publish_tf": False,
                "publish_keyframes": True,
                "map_frame": "visual_map",
                "odom_frame": "odom",
                "robot_base_frame": "base_link",
                "camera_frame": "camera_link",
                "encoding": "bgr8",
            }],
            remappings=[
                ("camera/image_raw", image_topic),
                ("~/camera_pose", "/stella_vslam/camera_pose"),
                ("~/keyframes", "/stella_vslam/keyframes"),
                ("~/keyframes_2d", "/stella_vslam/keyframes_2d"),
            ],
        ),

        Node(
            package="rover_stella_vslam",
            executable="stella_pose_adapter",
            name="stella_pose_adapter",
            output="screen",
            parameters=[default_adapter_config],
        ),
    ])
