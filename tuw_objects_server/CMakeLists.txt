cmake_minimum_required(VERSION 3.8)
project(tuw_objects_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tuw_geometry_msgs REQUIRED)
find_package(tuw_object_map_msgs REQUIRED)
find_package(tuw_object_msgs REQUIRED)
find_package(tuw_json REQUIRED)

add_executable(shape_server_node src/shape_server_node.cpp src/standalone_shape_server.cpp)
target_include_directories(shape_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(shape_server_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  shape_server_node
  "rclpy"
  "rclcpp"
  "std_srvs"
  "rclcpp_components"
  "tuw_geometry_msgs"
  "tuw_object_msgs"
  "tuw_json"
)

add_executable(objects_server_node src/objects_server_node.cpp src/standalone_objects_server.cpp)
target_include_directories(objects_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(objects_server_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  objects_server_node
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "std_srvs"
  "tuw_geometry_msgs"
  "tuw_object_map_msgs"
  "tuw_json"
)


add_executable(convert src/main_convert.cpp)
target_include_directories(convert PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(convert PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  convert
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tuw_object_msgs"
  "tuw_object_map_msgs"
  "tuw_json"
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  objects_server_node 
  shape_server_node 
  convert
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
