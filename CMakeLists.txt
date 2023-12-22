cmake_minimum_required(VERSION 3.5)
project(manipulation_planning)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(search REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(
  include
  /usr/local/include/${search_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# Declare the C++ library
add_library(${PROJECT_NAME}
  src/confspace_test.cpp
  include/manipulation_planning/action_space/manipulation_action_space.hpp
  include/manipulation_planning/common/moveit_scene_interface.hpp
  include/manipulation_planning/heuristics/manip_heuristics.hpp
  include/manipulation_planning/common/utils.hpp
)

# Specify additional locations of header files
include_directories(
  include
  ${search_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_cspace_test src/confspace_test.cpp)

# Link the library and dependencies to the executable
target_link_libraries(${PROJECT_NAME}_cspace_test
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${moveit_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  /usr/local/lib/${search_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

# Specify dependencies for the target
ament_target_dependencies(${PROJECT_NAME}
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp
  std_msgs
  Eigen3
  geometry_msgs
  moveit_msgs
  # /usr/local/lib${search_LIBRARIES} # Removed because it is not a ROS package.
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_cspace_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files, config files, and other resources
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
