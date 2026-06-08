cmake_minimum_required(VERSION 3.8)
project(edi_bottle_picking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(manipulator_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(edi_robot_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(edi_bottle_picking src/edi_bottle_picking.cpp
				   src/edi_bottle_picking_utils.cpp)

target_include_directories(edi_bottle_picking PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(edi_bottle_picking PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  edi_bottle_picking
  rclcpp
  moveit_ros_planning_interface
  control_msgs
  moveit_visual_tools
  manipulator_interface
  edi_robot_msgs
)

add_executable(grasping_test src/grasping_test.cpp
				   src/grasping_test_utils.cpp)

target_include_directories(grasping_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(grasping_test PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  grasping_test
  rclcpp
  moveit_ros_planning_interface
  control_msgs
  moveit_visual_tools
  manipulator_interface
  edi_robot_msgs
  ur_msgs
  yaml-cpp
  controller_manager_msgs
  std_srvs
)

add_executable(constant_pose src/constant_pose.cpp
				   src/constant_pose_utils.cpp)

target_include_directories(constant_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(constant_pose PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  constant_pose
  rclcpp
  moveit_ros_planning_interface
  control_msgs
  moveit_visual_tools
  manipulator_interface
  edi_robot_msgs
  yaml-cpp
  controller_manager_msgs
)

add_executable(conveyor_feeding src/conveyor_feeding.cpp
				   src/conveyor_feeding_utils.cpp)

target_include_directories(conveyor_feeding PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(conveyor_feeding PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  conveyor_feeding
  rclcpp
  moveit_ros_planning_interface
  control_msgs
  moveit_visual_tools
  manipulator_interface
  edi_robot_msgs
  ur_msgs
  yaml-cpp
  controller_manager_msgs
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS 
    edi_bottle_picking
    grasping_test
    constant_pose
    conveyor_feeding
  DESTINATION lib/${PROJECT_NAME})

# Python nodes (rclpy) installed as executables: ros2 run edi_bottle_picking <name>.py
install(PROGRAMS
  scripts/vacuum_gripper_bridge.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
