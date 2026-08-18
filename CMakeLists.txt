cmake_minimum_required(VERSION 3.8)
project(robot_dog_laser_designator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# ---- Core kinematics/model library (no ROS deps, easy to unit test) -----
add_library(robot_dog_laser_designator_core
  src/robot_dog_model.cpp
  src/leg_kinematics.cpp
  src/gait_engine.cpp
  src/trajectory_generator.cpp
  src/body_pose_controller.cpp
  src/trick_player.cpp
)
target_include_directories(robot_dog_laser_designator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS robot_dog_laser_designator_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(robot_dog_laser_designator_core)
ament_export_dependencies(rclcpp geometry_msgs std_msgs sensor_msgs)

# ---- Main ROS2 node (teleop -> gait -> IK -> joint command publisher) ---
add_executable(robot_dog_controller_node
  src/robot_dog_controller_node.cpp
  src/robot_dog_controller_main.cpp
)
target_link_libraries(robot_dog_controller_node robot_dog_laser_designator_core)
ament_target_dependencies(robot_dog_controller_node
  rclcpp geometry_msgs std_msgs sensor_msgs rcl_interfaces
)

install(TARGETS robot_dog_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Custom keyboard teleop (cmd_vel + sit/stand posture_cmd) -----------
add_executable(robot_dog_teleop_node
  src/keyboard_teleop_node.cpp
  src/keyboard_teleop_main.cpp
)
ament_target_dependencies(robot_dog_teleop_node
  rclcpp geometry_msgs std_msgs
)

install(TARGETS robot_dog_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Leg odometry (stance feet + FK -> /leg_odom) --------------------
add_executable(leg_odometry_node
  src/leg_odometry_node.cpp
  src/leg_odometry_main.cpp
)

target_link_libraries(leg_odometry_node robot_dog_laser_designator_core)

ament_target_dependencies(leg_odometry_node
  rclcpp sensor_msgs nav_msgs ros_gz_interfaces
)

install(TARGETS leg_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- FAST-LIO IMU QoS + TF bridge (identity until LIO odom) -------------
add_executable(fast_lio_bridge_node
  src/fast_lio_bridge_node.cpp
)
ament_target_dependencies(fast_lio_bridge_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros
)
install(TARGETS fast_lio_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  worlds
  meshes
  rviz
  maps
  DESTINATION share/${PROJECT_NAME}
)

# ---- Tests ---------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_leg_kinematics test/test_leg_kinematics_standalone.cpp)
    target_link_libraries(test_leg_kinematics robot_dog_laser_designator_core)

    ament_add_gtest(test_gait_trajectory test/test_gait_trajectory_standalone.cpp)
    target_link_libraries(test_gait_trajectory robot_dog_laser_designator_core)
  endif()
endif()

ament_package()
