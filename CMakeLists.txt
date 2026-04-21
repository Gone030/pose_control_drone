cmake_minimum_required(VERSION 3.8)
project(marker_velocity_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco)

add_executable(md
  src/marker_detector_node.cpp
)
ament_target_dependencies(md
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)
target_include_directories(md PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(md ${OpenCV_LIBRARIES})

add_executable(controller
  src/controller_node.cpp
)
ament_target_dependencies(controller
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(teleop
  src/teleop_node.cpp
)
ament_target_dependencies(teleop
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(body_rate_select_node
  src/body_rate_select_node.cpp
)
ament_target_dependencies(body_rate_select_node
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(vc
  src/vision_body_rate_controller.cpp
)
ament_target_dependencies(vc
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(mi
  src/mavlink_injector_node.cpp
)
ament_target_dependencies(mi
  rclcpp
  std_msgs
  geometry_msgs
)
target_include_directories(mi PRIVATE
  /home/jiwon/PX4-Autopilot/build/px4_sitl_default/mavlink
)

install(TARGETS
  md
  controller
  teleop
  body_rate_select_node
  vc
  mi
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../cam/iris
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/../cam/box
  DESTINATION share/${PROJECT_NAME}/models/small_box_cam_target
)

ament_package()
