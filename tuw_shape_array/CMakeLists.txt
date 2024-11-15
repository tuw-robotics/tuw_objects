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
find_package(tuw_object_msgs REQUIRED)
find_package(tuw_ros2_utils REQUIRED)
find_package(tuw_json REQUIRED)


# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)


add_executable(transform_shape_array src/transform_shapes_node.cpp src/standalone_transform_shapes.cpp)
target_include_directories(transform_shape_array PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(transform_shape_array PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  transform_shape_array
  "rclpy"
  "rclcpp"
  "rclcpp_components"
  "tuw_object_msgs"
  "tuw_ros2_utils"
  "tuw_json"
)


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
transform_shape_array
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