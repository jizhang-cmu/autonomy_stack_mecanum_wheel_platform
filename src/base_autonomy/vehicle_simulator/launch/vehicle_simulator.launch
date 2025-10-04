import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def declare_world_action(context, world_name):
  world_name_str = str(world_name.perform(context))
  declare_world = DeclareLaunchArgument('world', default_value=[os.path.join(get_package_share_directory('vehicle_simulator'), 'world', world_name_str + '.world')], description='')
  return [declare_world]

def generate_launch_description():
  vehicleHeight = LaunchConfiguration('vehicleHeight')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  vehicleZ = LaunchConfiguration('vehicleZ')
  terrainZ = LaunchConfiguration('terrainZ')
  vehicleYaw = LaunchConfiguration('vehicleYaw')
  terrainVoxelSize = LaunchConfiguration('terrainVoxelSize')
  groundHeightThre = LaunchConfiguration('groundHeightThre')
  adjustZ = LaunchConfiguration('adjustZ')
  terrainRadiusZ = LaunchConfiguration('terrainRadiusZ')
  minTerrainPointNumZ = LaunchConfiguration('minTerrainPointNumZ')
  smoothRateZ = LaunchConfiguration('smoothRateZ')
  adjustIncl = LaunchConfiguration('adjustIncl')
  terrainRadiusIncl = LaunchConfiguration('terrainRadiusIncl')
  minTerrainPointNumIncl = LaunchConfiguration('minTerrainPointNumIncl')
  smoothRateIncl = LaunchConfiguration('smoothRateIncl')
  InclFittingThre = LaunchConfiguration('InclFittingThre')
  maxIncl = LaunchConfiguration('maxIncl')
  pause = LaunchConfiguration('pause')
  world_name = LaunchConfiguration('world_name')

  declare_vehicleHeight = DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_vehicleZ = DeclareLaunchArgument('vehicleZ', default_value='0.0', description='')
  declare_terrainZ = DeclareLaunchArgument('terrainZ', default_value='0.0', description='')
  declare_vehicleYaw = DeclareLaunchArgument('vehicleYaw', default_value='0.0', description='')
  declare_terrainVoxelSize = DeclareLaunchArgument('terrainVoxelSize', default_value='0.05', description='')
  declare_groundHeightThre = DeclareLaunchArgument('groundHeightThre', default_value='0.1', description='')
  declare_adjustZ = DeclareLaunchArgument('adjustZ', default_value='true', description='')
  declare_terrainRadiusZ = DeclareLaunchArgument('terrainRadiusZ', default_value='1.0', description='')
  declare_minTerrainPointNumZ = DeclareLaunchArgument('minTerrainPointNumZ', default_value='5', description='')
  declare_smoothRateZ = DeclareLaunchArgument('smoothRateZ', default_value='0.2', description='')
  declare_adjustIncl = DeclareLaunchArgument('adjustIncl', default_value='true', description='')
  declare_terrainRadiusIncl = DeclareLaunchArgument('terrainRadiusIncl', default_value='2.0', description='')
  declare_minTerrainPointNumIncl = DeclareLaunchArgument('minTerrainPointNumIncl', default_value='200', description='')
  declare_smoothRateIncl = DeclareLaunchArgument('smoothRateIncl', default_value='0.2', description='')
  declare_InclFittingThre = DeclareLaunchArgument('InclFittingThre', default_value='0.2', description='')
  declare_maxIncl = DeclareLaunchArgument('maxIncl', default_value='30.0', description='')
  declare_pause = DeclareLaunchArgument('pause', default_value='false', description='')
  declare_world_name = DeclareLaunchArgument('world_name', default_value='garage', description='')

  start_vehicle_simulator = Node(
    package='vehicle_simulator', 
    executable='vehicleSimulator',
    parameters=[
      {
        'vehicleHeight': vehicleHeight,
        'vehicleX': vehicleX,
        'vehicleY': vehicleY,
        'vehicleZ': vehicleZ,
        'terrainZ': terrainZ,
        'vehicleYaw': vehicleYaw,
        'terrainVoxelSize': terrainVoxelSize,
        'groundHeightThre': groundHeightThre,
        'adjustZ': adjustZ,
        'terrainRadiusZ': terrainRadiusZ,
        'minTerrainPointNumZ': minTerrainPointNumZ,
        'smoothRateZ': smoothRateZ,
        'adjustIncl': adjustIncl,
        'terrainRadiusIncl': terrainRadiusIncl,
        'minTerrainPointNumIncl': minTerrainPointNumIncl,
        'smoothRateIncl': smoothRateIncl,
        'InclFittingThre': InclFittingThre,
        'maxIncl': maxIncl,
      }
      ],
      output='screen'
  )

  delayed_start_vehicle_simulator = TimerAction(
    period=0.0,
    actions=[
      start_vehicle_simulator
    ]
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_vehicleHeight)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_vehicleZ)
  ld.add_action(declare_terrainZ)
  ld.add_action(declare_vehicleYaw)
  ld.add_action(declare_terrainVoxelSize)
  ld.add_action(declare_groundHeightThre)
  ld.add_action(declare_adjustZ)
  ld.add_action(declare_terrainRadiusZ)
  ld.add_action(declare_minTerrainPointNumZ)
  ld.add_action(declare_smoothRateZ)
  ld.add_action(declare_adjustIncl)
  ld.add_action(declare_terrainRadiusIncl)
  ld.add_action(declare_minTerrainPointNumIncl)
  ld.add_action(declare_smoothRateIncl)
  ld.add_action(declare_InclFittingThre)
  ld.add_action(declare_maxIncl)
  ld.add_action(declare_pause)
  ld.add_action(declare_world_name)
  ld.add_action(OpaqueFunction(function=declare_world_action, args=[world_name]))
  
  ld.add_action(delayed_start_vehicle_simulator)

  return ld
