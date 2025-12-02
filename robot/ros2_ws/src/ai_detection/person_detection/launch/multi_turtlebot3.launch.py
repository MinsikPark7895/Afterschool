import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node  # Node 클래스를 import 합니다.

def generate_launch_description():
    # --- 기존 코드 (Gazebo 및 로봇 스폰) ---
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
    
    # Spawn Robot 1
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
    
    # Spawn Robot 2
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

    # --- 추가된 코드 (탐지 및 추적 노드) ---

    # ⚠️ 여기를 실제 패키지 이름으로 변경하세요!
    your_package_name = 'person_detection' 

    # Robot 1의 탐지 및 추적 노드
    tb1_detector_node = Node(
        package=your_package_name,
        executable='person_detect_only',
        name='tb1_person_detector',
        remappings=[
            ('/camera/image_raw', '/tb1/camera/image_raw'),
            ('/person_detected', '/tb1/person_detected'),
            ('/person_position', '/tb1/person_position')
        ]
    )
    
    tb1_tracker_node = Node(
        package=your_package_name,
        executable='person_tracking_only',
        name='tb1_person_tracker',
        remappings=[
            ('/person_detected', '/tb1/person_detected'),
            ('/person_position', '/tb1/person_position'),
            ('/cmd_vel', '/tb1/cmd_vel')
        ]
    )

    # Robot 2의 탐지 및 추적 노드
    tb2_detector_node = Node(
        package=your_package_name,
        executable='person_detect_only',
        name='tb2_person_detector',
        remappings=[
            ('/camera/image_raw', '/tb2/camera/image_raw'),
            ('/person_detected', '/tb2/person_detected'),
            ('/person_position', '/tb2/person_position')
        ]
    )

    tb2_tracker_node = Node(
        package=your_package_name,
        executable='person_tracking_only',
        name='tb2_person_tracker',
        remappings=[
            ('/person_detected', '/tb2/person_detected'),
            ('/person_position', '/tb2/person_position'),
            ('/cmd_vel', '/tb2/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        # 기존 Gazebo 및 스폰 노드
        gzserver,
        gzclient,
        spawn_tb1,
        spawn_tb2,
        
        # 추가된 탐지 및 추적 노드
        tb1_detector_node,
        tb1_tracker_node,
        tb2_detector_node,
        tb2_tracker_node
    ])
