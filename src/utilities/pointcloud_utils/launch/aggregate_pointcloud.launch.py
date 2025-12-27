from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'viz_voxel_size',
            default_value='0.1',
            description='Voxel size for visualization cloud (meters)'
        ),
        DeclareLaunchArgument(
            'save_voxel_size',
            default_value='0.025',
            description='Voxel size for saved cloud (meters)'
        ),
        DeclareLaunchArgument(
            'max_points',
            default_value='1000000',
            description='Maximum points in visualization cloud'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='1.0',
            description='Rate to publish aggregated cloud (Hz)'
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/registered_scan',
            description='Input point cloud topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/aggregated_pointcloud',
            description='Output aggregated point cloud topic'
        ),
        DeclareLaunchArgument(
            'save_directory',
            default_value='/tmp',
            description='Directory to save point cloud files'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='Frame ID for output point cloud'
        ),
        DeclareLaunchArgument(
            'save_ply',
            default_value='false',
            description='Also save as PLY format'
        ),

        Node(
            package='pointcloud_utils',
            executable='aggregate_pointcloud',
            name='aggregate_pointcloud',
            output='screen',
            parameters=[{
                'viz_voxel_size': LaunchConfiguration('viz_voxel_size'),
                'save_voxel_size': LaunchConfiguration('save_voxel_size'),
                'max_points': LaunchConfiguration('max_points'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'save_directory': LaunchConfiguration('save_directory'),
                'frame_id': LaunchConfiguration('frame_id'),
                'save_ply': LaunchConfiguration('save_ply'),
            }]
        ),
    ])
