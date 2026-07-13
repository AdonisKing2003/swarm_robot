cmake_minimum_required(VERSION 3.8)
project(swarm_flocking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(swarm_interfaces REQUIRED)

add_executable(flocking_node src/flocking_node.cpp)
ament_target_dependencies(flocking_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2_ros 
  tf2 
  tf2_geometry_msgs
)
target_include_directories(flocking_node PUBLIC include)
target_link_libraries(flocking_node
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS flocking_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_boids test/unit_test/test_boids.cpp)
  target_include_directories(test_boids PUBLIC include)
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
