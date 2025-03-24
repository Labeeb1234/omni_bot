cmake_minimum_required(VERSION 3.8)
project(omni_bot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

###########################################################################
add_library(omni_bot_kinematics SHARED
  src/omni_bot_kinematics.cpp
)
target_include_directories(omni_bot_kinematics PUBLIC
  include
)

add_executable(omni_bot_controller src/omni_bot_controller.cpp)
target_include_directories(omni_bot_controller PUBLIC
  include
)
target_link_libraries(omni_bot_controller omni_bot_kinematics)
ament_target_dependencies(omni_bot_controller
  rclcpp
  geometry_msgs
  sensor_msgs
)

# Install targets with proper destinations
install(TARGETS
  omni_bot_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  omni_bot_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
##############################################################################

add_executable(lidar_utils_node src/lidar_utils.cpp)
ament_target_dependencies(lidar_utils_node
  rclcpp
  sensor_msgs
)
install(
  TARGETS
    lidar_utils_node
  DESTINATION lib/${PROJECT_NAME}
)

###############################################################################

add_executable(nav_commander_node src/nav_commander.cpp)
ament_target_dependencies(nav_commander_node
  rclcpp
  geometry_msgs
  tf2
)
install(
  TARGETS
    nav_commander_node
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################

install(
  DIRECTORY
    config
    rviz
    launch
    nav_params
    maps
  DESTINATION
    share/${PROJECT_NAME}
)

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
