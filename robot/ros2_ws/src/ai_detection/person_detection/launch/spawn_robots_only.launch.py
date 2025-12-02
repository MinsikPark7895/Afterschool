import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    world = os.path.join(pkg_turtlebot3_gazebo, 'worlds', 'turtlebot3_school.world')
    waffle_model = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_waffle', 'model.sdf')
    
    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )
    
    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn TB1
    spawn_tb1 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'tb1',
             '-file', waffle_model,
             '-x', '-2.0',
             '-y', '0.0',
             '-z', '0.01',
             '-robot_namespace', 'tb1'],
        output='screen'
    )
    
    # Spawn TB2
    spawn_tb2 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'tb2',
             '-file', waffle_model,
             '-x', '2.0',
             '-y', '0.0',
             '-z', '0.01',
             '-robot_namespace', 'tb2'],
        output='screen'
    )
    
    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_tb1,
        spawn_tb2
    ])