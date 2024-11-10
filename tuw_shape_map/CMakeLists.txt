cmake_minimum_required(VERSION 3.8)
project(tuw_shape_map)

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
find_package(tuw_object_msgs REQUIRED)
find_package(tuw_ros2_utils REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tuw_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)


# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)



add_library(tuw_object_map src/object_map.cpp)
target_compile_features(tuw_object_map PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(tuw_object_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  tuw_object_map
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tuw_geometry"
  "tuw_object_msgs"
  "geographic_msgs"
  "tuw_ros2_utils"
  "OpenCV"
  "GeographicLib"
)
target_link_libraries(tuw_object_map ${GeographicLib_LIBRARIES})


add_executable(object_map_node src/standalone_object_map.cpp src/object_map_node.cpp)
target_include_directories(object_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(object_map_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(object_map_node tuw_object_map )
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
  "tuw_object_msgs"
  "tuw_ros2_utils"
  "tuw_json"
  "OpenCV"
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  tuw_object_map
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  object_map_node
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
ament_export_libraries(
  tuw_shape_map
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3 
"tuw_geometry"
"geographic_msgs")

ament_package()