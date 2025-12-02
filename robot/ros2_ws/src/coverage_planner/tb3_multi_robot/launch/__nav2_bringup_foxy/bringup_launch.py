#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Arshad Mehmood

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    robots = [
        {'name': 'tb1', 'x_pose': '-2.5', 'y_pose': '-3.5', 'z_pose': '0.01'},
        {'name': 'tb2', 'x_pose': '2.5', 'y_pose': '-3.5', 'z_pose': '0.01'},
    ]

    TURTLEBOT3_MODEL = 'burger_cam'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')
    package_dir = get_package_share_directory('turtlebot3_multi_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    urdf = os.path.join(turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf')
    world = os.path.join(turtlebot3_multi_robot, 'worlds', 'school_final.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'paused': 'true'}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params_' + TURTLEBOT3_MODEL + '.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Global map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(turtlebot3_multi_robot, 'maps', 'map.yaml'), 'use_sim_time': use_sim_time}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'node_names': ['map_server']}]
    )

    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle)

    last_action = None
    for robot in robots:
        namespace = robot['name']

        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_frequency': 10.0,
                'frame_prefix': namespace + '/',
                'robot_description': Command(['xacro ', urdf])
            }],
            remappings=[('/tf', '/' + namespace + '/tf'), ('/tf_static', '/' + namespace + '/tf_static')]
        )

        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-timeout', '600',
                '-file', os.path.join(turtlebot3_multi_robot, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL + '_' + namespace, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'], '-z', robot['z_pose'], '-Y', '0.0'
            ],
            output='screen'
        )

        nav2_bringup = GroupAction(
            actions=[
                PushRosNamespace(namespace=namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={
                        'slam': 'False',
                        'namespace': namespace,
                        'use_namespace': 'True',
                        'map': os.path.join(turtlebot3_multi_robot, 'maps', 'map.yaml'),
                        'use_sim_time': use_sim_time,
                        'params_file': os.path.join(package_dir, 'params', 'nav2_params_' + TURTLEBOT3_MODEL + '_' + namespace + '.yaml'),
                        'autostart': 'True'
                    }.items()
                )
            ]
        )

        if last_action is None:
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            ld.add_action(nav2_bringup)
        else:
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot3_burger, turtlebot_state_publisher, nav2_bringup]
                )
            )
            ld.add_action(spawn_turtlebot3_event)

        last_action = spawn_turtlebot3_burger

    # Unpause Gazebo after all robots are spawned
    unpause_physics = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/Empty'],
        output='screen'
    )
    unpause_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=last_action,
            on_exit=[unpause_physics]
        )
    )
    ld.add_action(unpause_event)

    # Start RViz and drive nodes after unpausing
    for robot in robots:
        namespace = robot['name']

        message = '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', '/' + namespace + '/initialpose',
                 'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'rviz_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': 'True',
                'rviz_config': rviz_config_file,
                'use_sim_time': use_sim_time,
                'log_level': 'warn'
            }.items(),
            condition=IfCondition(enable_rviz)
        )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_drive',
            namespace=namespace,
            output='screen',
            condition=IfCondition(enable_drive)
        )

        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger]
            )
        )

        last_action = initial_pose_cmd
        ld.add_action(post_spawn_event)

    return ld