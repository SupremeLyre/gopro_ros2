from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("gopro_video", default_value=""),
        DeclareLaunchArgument("gopro_folder", default_value=""),
        DeclareLaunchArgument("multiple_files", default_value="false"),
        DeclareLaunchArgument("rosbag", default_value=""),
        DeclareLaunchArgument("scale", default_value="0.5"),
        DeclareLaunchArgument("compressed_image_format", default_value="true"),
        DeclareLaunchArgument("grayscale", default_value="true"),
        DeclareLaunchArgument("display_images", default_value="false"),
    ]

    node = Node(
        package="gopro_ros",
        executable="gopro_to_rosbag",
        name="gopro_to_rosbag",
        output="screen",
        parameters=[{
            "gopro_video": LaunchConfiguration("gopro_video"),
            "gopro_folder": LaunchConfiguration("gopro_folder"),
            "multiple_files": LaunchConfiguration("multiple_files"),
            "rosbag": LaunchConfiguration("rosbag"),
            "scale": LaunchConfiguration("scale"),
            "compressed_image_format": LaunchConfiguration("compressed_image_format"),
            "grayscale": LaunchConfiguration("grayscale"),
            "display_images": LaunchConfiguration("display_images"),
        }],
    )

    return LaunchDescription(args + [node])
