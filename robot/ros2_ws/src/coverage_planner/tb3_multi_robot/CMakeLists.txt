cmake_minimum_required(VERSION 3.5)
project(turtlebot3_multi_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(action_msgs REQUIRED)

# Install directories
install(DIRECTORY
  launch
  maps
  models
  params
  rviz
  urdf
  worlds
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/coverage_planner.py
  scripts/fsm.py
  scripts/person_detector.py
  scripts/person_tracker.py
  scripts/trail_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


