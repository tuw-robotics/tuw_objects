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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tuw_geometry REQUIRED)
find_package(tuw_geometry_msgs REQUIRED)
find_package(tuw_object_map_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tuw_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)


# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)



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
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "visualization_msgs"
  "tuw_geometry"
  "tuw_geometry_msgs"
  "tuw_object_map_msgs"
  "tuw_json"
  "OpenCV"
  "GeographicLib"
)
target_link_libraries(object_map ${GeographicLib_LIBRARIES})


add_executable(object_map_node src/standalone_object_map.cpp src/object_map_node.cpp)
target_include_directories(object_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(object_map_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(object_map_node object_map )
ament_target_dependencies(
  object_map_node
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "visualization_msgs"
  "tuw_geometry"
  "tuw_geometry_msgs"
  "tuw_object_map_msgs"
  "tuw_json"
  "OpenCV"
)

add_executable(example_object_map src/example/main_object_map.cpp)
target_include_directories(example_object_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_object_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(example_object_map object_map )
ament_target_dependencies(
  example_object_map
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "visualization_msgs"
  "tuw_geometry"
  "tuw_geometry_msgs"
  "tuw_object_map_msgs"
  "tuw_json"
  "OpenCV"
  "GeographicLib"
)

add_executable(example_geo_map src/example/main_geo_map.cpp)
target_include_directories(example_geo_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(example_geo_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(example_geo_map object_map )
ament_target_dependencies(
  example_geo_map
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "visualization_msgs"
  "tuw_geometry"
  "tuw_geometry_msgs"
  "tuw_object_map_msgs"
  "tuw_json"
  "OpenCV"
  "GeographicLib"
)

install(TARGETS 
  example_object_map
  example_geo_map
  object_map_node
  object_map
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/gps_publisher_node.py
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
