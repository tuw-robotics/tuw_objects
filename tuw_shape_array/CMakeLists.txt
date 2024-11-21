cmake_minimum_required(VERSION 3.8)
project(tuw_shape_array)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tuw_object_msgs REQUIRED)
find_package(tuw_ros2_utils REQUIRED)
find_package(tuw_json REQUIRED)
find_package(tf2_ros REQUIRED)


# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

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
  "tuw_object_msgs"
  "tuw_json"
)

add_executable(transform_from_wgs84_node src/transform_from_wgs84_node.cpp src/standalone_transform_from_wgs84.cpp)
target_include_directories(transform_from_wgs84_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(transform_from_wgs84_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  transform_from_wgs84_node
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tf2_ros"
  "tuw_object_msgs"
  "tuw_ros2_utils"
  "tuw_json"
  GeographicLib
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  shape_server_node 
  transform_from_wgs84_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  scripts/file.py
#  DESTINATION lib/${PROJECT_NAME}
#)

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

ament_export_include_directories(
  include
)
#ament_export_libraries(
#)
#ament_export_targets(
#  export_${PROJECT_NAME}
#)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3 
"tuw_geometry"
"geographic_msgs")

ament_package()