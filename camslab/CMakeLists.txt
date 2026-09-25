cmake_minimum_required(VERSION 3.8)
project(camslab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

# Control law and reference trajectories, without ROS dependencies.
add_library(camslab_core STATIC src/controller.cpp src/trajectory.cpp)
target_include_directories(camslab_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(camslab_core PUBLIC cxx_std_17)

add_executable(controller src/controller_node.cpp)
target_link_libraries(controller camslab_core)
ament_target_dependencies(controller rclcpp geometry_msgs nav_msgs std_srvs)

add_executable(pose_tf src/pose_tf.cpp)
ament_target_dependencies(pose_tf rclcpp geometry_msgs tf2_ros)

install(TARGETS controller pose_tf DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models rviz urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
