cmake_minimum_required(VERSION 3.8)
project(gtg_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# Control law and plant model, without ROS dependencies.
add_library(gtg_core STATIC src/controller.cpp src/plant.cpp src/formation.cpp)
target_include_directories(gtg_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(gtg_core PUBLIC cxx_std_17)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node gtg_core)
ament_target_dependencies(controller_node rclcpp geometry_msgs std_srvs)

add_executable(pose_tf_node src/pose_tf_node.cpp)
ament_target_dependencies(pose_tf_node rclcpp geometry_msgs tf2_ros)

install(TARGETS controller_node pose_tf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config models rviz urdf worlds DESTINATION share/${PROJECT_NAME})

ament_package()
