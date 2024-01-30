cmake_minimum_required(VERSION 3.8)
project(tuw_object_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tuw_object_map_msgs REQUIRED)
find_package(tuw_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)





add_library(object_map src/object_map.cpp)
target_compile_features(object_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(object_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  object_map
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tuw_object_map_msgs"
  "OpenCV"
)

add_executable(generate_object_costmap src/standalone_object_map.cpp src/object_map_node.cpp)
target_include_directories(generate_object_costmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(generate_object_costmap PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(generate_object_costmap object_map )
ament_target_dependencies(
  generate_object_costmap
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tuw_object_map_msgs"
  "OpenCV"
)

install(TARGETS 
  generate_object_costmap
  object_map
  DESTINATION lib/${PROJECT_NAME})

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
