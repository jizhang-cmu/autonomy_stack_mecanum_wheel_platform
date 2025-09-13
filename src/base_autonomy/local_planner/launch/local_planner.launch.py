import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    local_planner_share = get_package_share_directory('local_planner')

    # Get robot config from environment variable or use default
    robot_config_env = os.environ.get('ROBOT_CONFIG_PATH', 'unitree/unitree_g1')

    # Declare launch arguments
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='omniDir',
        description='omniDir: if with mecanum wheels, standard: if with standard wheels'
    )

    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value=robot_config_env,
        description='Robot-specific config file (without .yaml extension)'
    )

    twoWayDrive_arg = DeclareLaunchArgument(
        'twoWayDrive',
        default_value='false'
    )

    autonomyMode_arg = DeclareLaunchArgument(
        'autonomyMode',
        default_value='false'
    )

    joyToSpeedDelay_arg = DeclareLaunchArgument(
        'joyToSpeedDelay',
        default_value='2.0'
    )

    goalX_arg = DeclareLaunchArgument(
        'goalX',
        default_value='0.0'
    )

    goalY_arg = DeclareLaunchArgument(
        'goalY',
        default_value='0.0'
    )

    cameraOffsetZ_arg = DeclareLaunchArgument(
        'cameraOffsetZ',
        default_value='0.0'
    )

    # Read sensor offsets from robot config YAML
    sensor_offsets = {
        'sensorOffsetX': 0.0,
        'sensorOffsetY': 0.0,
        'sensorOffsetZ': 0.0
    }

    try:
        robot_config_path = os.path.join(local_planner_share, 'config', robot_config_env + '.yaml')
        with open(robot_config_path, 'r') as file:
            config_data = yaml.safe_load(file)

        # Extract sensor offsets from sensorMountingOffsets section
        if 'sensorMountingOffsets' in config_data:
            mounting_offsets = config_data['sensorMountingOffsets']
            for key in sensor_offsets.keys():
                if key in mounting_offsets:
                    sensor_offsets[key] = mounting_offsets[key]
    except Exception as e:
        print(f"Warning: Could not read robot config from {robot_config_env}.yaml, using defaults: {e}")

    # Config file paths will be resolved by the nodes using substitutions

    # LocalPlanner node
    localPlanner_node = Node(
        package='local_planner',
        executable='localPlanner',
        name='localPlanner',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('local_planner'),
                'config',
                [LaunchConfiguration('config'), '.yaml']
            ]),
            PathJoinSubstitution([
                FindPackageShare('local_planner'),
                'config',
                [LaunchConfiguration('robot_config'), '.yaml']
            ]),
            {
                'pathFolder': os.path.join(local_planner_share, 'paths'),
                'twoWayDrive': True,
                'laserVoxelSize': 0.05,
                'terrainVoxelSize': 0.2,
                'useTerrainAnalysis': True,
                'checkObstacle': True,
                'checkRotObstacle': False,
                'adjacentRange': 3.5,
                'obstacleHeightThre': 0.1,
                'groundHeightThre': 0.1,
                'costHeightThre1': 0.1,
                'costHeightThre2': 0.05,
                'useCost': False,
                'slowPathNumThre': 5,
                'slowGroupNumThre': 1,
                'pointPerPathThre': 2,
                'minRelZ': -0.4,
                'maxRelZ': 0.3,
                'dirWeight': 0.02,
                'dirThre': 90.0,
                'dirToVehicle': False,
                'pathScale': 0.875,
                'minPathScale': 0.675,
                'pathScaleStep': 0.1,
                'pathScaleBySpeed': True,
                'minPathRange': 0.8,
                'pathRangeStep': 0.6,
                'pathRangeBySpeed': True,
                'pathCropByGoal': True,
                'autonomyMode': LaunchConfiguration('autonomyMode'),
                'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay'),
                'joyToCheckObstacleDelay': 5.0,
                'freezeAng': 90.0,
                'freezeTime': 0.0,
                'goalX': LaunchConfiguration('goalX'),
                'goalY': LaunchConfiguration('goalY'),
            }
        ]
    )

    # PathFollower node
    pathFollower_node = Node(
        package='local_planner',
        executable='pathFollower',
        name='pathFollower',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('local_planner'),
                'config',
                [LaunchConfiguration('config'), '.yaml']
            ]),
            PathJoinSubstitution([
                FindPackageShare('local_planner'),
                'config',
                [LaunchConfiguration('robot_config'), '.yaml']
            ]),
            {
                'pubSkipNum': 1,
                'twoWayDrive': LaunchConfiguration('twoWayDrive'),
                'switchTimeThre': 1.0,
                'useInclRateToSlow': False,
                'inclRateThre': 120.0,
                'slowRate1': 0.25,
                'slowRate2': 0.5,
                'slowRate3': 0.75,
                'slowTime1': 2.0,
                'slowTime2': 2.0,
                'useInclToStop': False,
                'inclThre': 45.0,
                'stopTime': 5.0,
                'noRotAtStop': False,
                'noRotAtGoal': True,
                'autonomyMode': LaunchConfiguration('autonomyMode'),
                'joyToSpeedDelay': LaunchConfiguration('joyToSpeedDelay'),
            }
        ]
    )

    # Static transform publishers with sensor offsets from config
    vehicleTransPublisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='vehicleTransPublisher',
        arguments=[
            str(sensor_offsets['sensorOffsetX']),
            str(sensor_offsets['sensorOffsetY']),
            str(sensor_offsets['sensorOffsetZ']),
            '0', '0', '0',
            '/sensor',
            '/vehicle'
        ]
    )

    sensorTransPublisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensorTransPublisher',
        arguments=[
            '0', '0', LaunchConfiguration('cameraOffsetZ'),
            '-1.5707963', '0', '-1.5707963',
            '/sensor', '/camera'
        ]
    )

    return LaunchDescription([
        config_arg,
        robot_config_arg,
        twoWayDrive_arg,
        autonomyMode_arg,
        joyToSpeedDelay_arg,
        goalX_arg,
        goalY_arg,
        cameraOffsetZ_arg,
        localPlanner_node,
        pathFollower_node,
        vehicleTransPublisher_node,
        sensorTransPublisher_node,
    ])