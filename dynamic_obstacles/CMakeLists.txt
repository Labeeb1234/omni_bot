cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(pallet_jack_actor src/pallet_jack_actor.cpp)
ament_target_dependencies(pallet_jack_actor
  rclcpp
  geometry_msgs
  nav_msgs
)

install(
  TARGETS
    pallet_jack_actor
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(dumb_bot_routine_node src/dumb_bot_routine.cpp)
ament_target_dependencies(dumb_bot_routine_node
  rclcpp
  geometry_msgs
  nav_msgs
)

install(
  TARGETS
    dumb_bot_routine_node
  DESTINATION lib/${PROJECT_NAME} 
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
