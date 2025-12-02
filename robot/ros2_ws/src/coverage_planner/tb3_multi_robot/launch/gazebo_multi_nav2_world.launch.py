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
import yaml
import cv2
import numpy as np
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

def generate_launch_description():
    ld = LaunchDescription()

    ####### Robot Configuration #######
    ####### TODO 1: Toggle the robots #######
    # Names and poses of the robots
    robots = [
        # {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        # {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # {'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        # {'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # ...
        # ...

        {'name': 'tb1', 'x_pose': '-2.30', 'y_pose': '13.0', 'z_pose': 0.01, 'yaw': '-1.5'},
        {'name': 'tb2', 'x_pose': '2.30', 'y_pose': '13.0', 'z_pose': 0.01, 'yaw': '-1.5'},
    ]

    TURTLEBOT3_MODEL = 'waffle'

    ####### Launch Arguments #######
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

    enable_coverage_planner = LaunchConfiguration('enable_coverage_planner', default='true')
    declare_enable_coverage_planner = DeclareLaunchArgument(
        name='enable_coverage_planner', default_value=enable_coverage_planner, description='Enable coverage planner node'
    )

    enable_person_nodes = LaunchConfiguration('enable_person_nodes', default='false')
    declare_enable_person_nodes = DeclareLaunchArgument(
        name='enable_person_nodes', default_value=enable_person_nodes, description='Enable person detector and tracker nodes'
    )

    enable_fsm = LaunchConfiguration('enable_fsm', default='false')
    declare_enable_fsm = DeclareLaunchArgument(
        name='enable_fsm', default_value=enable_fsm, description='Enable FSM node'
    )

    package_dir = get_package_share_directory('turtlebot3_multi_robot')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            # TODO 2: Change the RViz configuration file here
            # package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
            package_dir, 'rviz', 'afterschool_multi_nav2_view.rviz'),
        description='Full path to the RVIZ config file to use')

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params_' + TURTLEBOT3_MODEL + '.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    ####### Paths #######
    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')
    
    urdf = os.path.join(
        turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    world = os.path.join(
        get_package_share_directory('turtlebot3_multi_robot'),
        ####### TODO 3: change world file #######
        # 'worlds', 'multi_robot_world.world')
        'worlds', 'school_final.world')

    ####### TODO 4: change map file #######
    # map_yaml_path = os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')
    map_yaml_path = os.path.join(turtlebot3_multi_robot, 'maps', 'map.yaml')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    ####### Global Nodes #######
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path}],
        remappings=remappings
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # bt_viewer = ExecuteProcess(
    #     cmd=['py-trees-tree-viewer'],
    #     output='screen',
    # )

    ####### Add Launch Actions #######
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_enable_coverage_planner)
    ld.add_action(declare_enable_person_nodes)
    ld.add_action(declare_enable_fsm)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(map_server)
    ld.add_action(map_server_lifecycle)
    # ld.add_action(bt_viewer)

    ####### Robot Spawning Loop #######
    # Remapping is required for state publisher otherwise /tf and /tf_static 
    # will get be published on root '/' namespace
    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:
        namespace = '/' + robot['name']

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(turtlebot3_multi_robot, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-R', '0.0', '-P', '0.0', '-Y', robot['yaw'],
                '-timeout', '180.0',
                '-unpause',
            ],
            output='screen',
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_dir, 'bringup_launch.py')),
            launch_arguments={  
                'slam': 'False',
                'namespace': namespace,
                'use_namespace': 'True',
                'map': map_yaml_path,
                'map_server': 'True',
                'params_file': params_file,
                'default_bt_xml_filename': os.path.join(
                    turtlebot3_multi_robot, 'behavior_trees', 'coverage_bt.xml'),
                    # get_package_share_directory('nav2_bt_navigator'),
                    # 'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                'autostart': 'true',
                'use_sim_time': use_sim_time,
                'log_level': 'info'
            }.items()
        )
    
        # coverage_planner_node_tb1 = Node(
        #     package='turtlebot3_multi_robot',
        #     executable='coverage_planner.py',
        #     namespace='tb1',
        #     output='screen',
        #     parameters=[os.path.join(package_dir, 'params', 'waypoints.yaml')],
        #     remappings=remappings,
        #     condition=IfCondition(enable_coverage_planner)
        # )

        # coverage_planner_node_tb2 = Node(
        #     package='turtlebot3_multi_robot',
        #     executable='coverage_planner.py',
        #     namespace='tb2',
        #     output='screen',
        #     parameters=[os.path.join(package_dir, 'params', 'waypoints.yaml')],
        #     remappings=remappings,
        #     condition=IfCondition(enable_coverage_planner)
        # )

        coverage_planner_node = Node(
            package='turtlebot3_multi_robot',
            executable='coverage_planner.py',
            namespace=namespace,
            output='screen',
            parameters=[os.path.join(package_dir, 'params', 'waypoints.yaml')],
            remappings=remappings,
            condition=IfCondition(enable_coverage_planner)
        )

        trail_publisher_node = Node(
            package='turtlebot3_multi_robot',
            executable='trail_publisher.py',
            namespace=namespace,
            output='screen',
            parameters=[
                {'robot_id': robot['name']},
                {'use_sim_time': use_sim_time}
            ],
            remappings=remappings,
            condition=IfCondition(enable_coverage_planner)
        )

        person_detector_node = Node(
            package='turtlebot3_multi_robot',
            executable='person_detector.py',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings,
            condition=IfCondition(enable_person_nodes)
        )

        person_tracker_node = Node(
            package='turtlebot3_multi_robot',
            executable='person_tracker.py',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings,
            condition=IfCondition(enable_person_nodes)
        )

        fsm_node = Node(
            package='turtlebot3_multi_robot',
            executable='fsm.py',
            namespace=namespace,
            output='screen',
            parameters=[{'spawn_x': robot['x_pose'], 'spawn_y': robot['y_pose']}],
            remappings=remappings,
            condition=IfCondition(enable_fsm)
        )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(turtlebot_state_publisher)
            ld.add_action(spawn_turtlebot3_burger)
            ld.add_action(bringup_cmd)
            ld.add_action(coverage_planner_node)
            # ld.add_action(coverage_planner_node_tb1)
            # ld.add_action(coverage_planner_node_tb2)
            ld.add_action(trail_publisher_node)
            ld.add_action(person_detector_node)
            ld.add_action(person_tracker_node)
            ld.add_action(fsm_node)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[
                        turtlebot_state_publisher,
                        spawn_turtlebot3_burger,
                        bringup_cmd,
                        coverage_planner_node,
                        # coverage_planner_node_tb1,
                        # coverage_planner_node_tb2,
                        trail_publisher_node,
                        person_detector_node,
                        person_tracker_node,
                        fsm_node
                    ],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn_turtlebot3_burger
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create a initial pose topic publish call
        # message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
        #     robot['x_pose'] + ', y: ' + robot['y_pose'] + \
        #     ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        message = (
            '{'
            'header: {frame_id: map, stamp: {sec: 0, nanosec: 0}}, '
            'pose: {'
            'pose: {'
            'position: {x: ' + robot['x_pose'] + ', y: ' + robot['y_pose'] + ', z: 0.0}, '
            'orientation: {x: 0.0, y: 0.0, z: ' + str(np.sin(float(robot['yaw'])/2)) + ', w: ' + str(np.cos(float(robot['yaw'])/2)) + '}'
            '}, '
            'covariance: ['
            '0.1, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.1, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.01'
            ']'
            '}'
            '}'
        )

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'info'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )

        drive_turtlebot3_burger = Node(
            package='turtlebot3_gazebo', executable='turtlebot3_drive',
            namespace=namespace, output='screen',
            condition=IfCondition(enable_drive),
        )

        # Use RegisterEventHandler to ensure next robot rviz launch happens 
        # only after all robots are spawned
        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
            )
        )

        # Perform next rviz and other node instantiation after the previous intialpose request done
        ld.add_action(post_spawn_event)
        ld.add_action(declare_params_file_cmd)

        last_action = initial_pose_cmd
    ######################

    return ld