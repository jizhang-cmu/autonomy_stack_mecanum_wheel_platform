from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("detector_backend", default_value="mock"),
            DeclareLaunchArgument("target_label", default_value="chair"),
            DeclareLaunchArgument("image_transport", default_value="none"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image/transmitted"),
            DeclareLaunchArgument("require_image", default_value="false"),
            DeclareLaunchArgument("target_distance", default_value="1.5"),
            DeclareLaunchArgument("speed", default_value="0.25"),
            DeclareLaunchArgument("joy_forward_axis", default_value="1.0"),
            DeclareLaunchArgument("mock_center_x", default_value="0.5"),
            DeclareLaunchArgument("mock_center_y", default_value="0.5"),
            DeclareLaunchArgument("publish_boundary", default_value="false"),
            Node(
                package="semantic_waypoint_demo",
                executable="semantic_waypoint_node",
                name="semantic_waypoint_node",
                output="screen",
                parameters=[
                    {
                        "detector_backend": LaunchConfiguration("detector_backend"),
                        "target_label": LaunchConfiguration("target_label"),
                        "image_transport": LaunchConfiguration("image_transport"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "require_image": ParameterValue(
                            LaunchConfiguration("require_image"), value_type=bool
                        ),
                        "target_distance": ParameterValue(
                            LaunchConfiguration("target_distance"), value_type=float
                        ),
                        "speed": ParameterValue(LaunchConfiguration("speed"), value_type=float),
                        "joy_forward_axis": ParameterValue(
                            LaunchConfiguration("joy_forward_axis"), value_type=float
                        ),
                        "mock_center_x": ParameterValue(
                            LaunchConfiguration("mock_center_x"), value_type=float
                        ),
                        "mock_center_y": ParameterValue(
                            LaunchConfiguration("mock_center_y"), value_type=float
                        ),
                        "publish_boundary": ParameterValue(
                            LaunchConfiguration("publish_boundary"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
