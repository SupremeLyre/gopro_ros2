from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("gopro_video", default_value=""),
        DeclareLaunchArgument("gopro_folder", default_value=""),
        DeclareLaunchArgument("asl_dir", default_value=""),
        DeclareLaunchArgument("multiple_files", default_value="false"),
        DeclareLaunchArgument("scale", default_value="1.0"),
        DeclareLaunchArgument("grayscale", default_value="false"),
        DeclareLaunchArgument("display_images", default_value="false"),
    ]

    node = Node(
        package="gopro_ros",
        executable="gopro_to_asl",
        name="gopro_to_asl",
        output="screen",
        parameters=[{
            "gopro_video": LaunchConfiguration("gopro_video"),
            "gopro_folder": LaunchConfiguration("gopro_folder"),
            "asl_dir": LaunchConfiguration("asl_dir"),
            "multiple_files": LaunchConfiguration("multiple_files"),
            "scale": LaunchConfiguration("scale"),
            "grayscale": LaunchConfiguration("grayscale"),
            "display_images": LaunchConfiguration("display_images"),
        }],
    )

    return LaunchDescription(args + [node])
