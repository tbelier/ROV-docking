cmake_minimum_required(VERSION 3.5)
project(subsonus_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(msg_files
  "msg/Range.msg" 
  "msg/Usblmain.msg"
  "msg/SubsonusRawSensorsPacket.msg"
  "msg/SubsonusRemoteTrackPacket.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(talker src/main.cpp src/an_packet_protocol.c src/subsonus_packets.c)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs tf2_ros tf2 visualization_msgs tf2_geometry_msgs)
target_link_libraries(talker "${cpp_typesupport_target}")
target_include_directories(talker PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


add_executable(nodeRawdata_surface src/nodeRawdata_surface.cpp src/an_packet_protocol.c src/subsonus_packets.c)
ament_target_dependencies(nodeRawdata_surface rclcpp std_msgs geometry_msgs tf2_ros tf2 visualization_msgs tf2_geometry_msgs)
target_link_libraries(nodeRawdata_surface "${cpp_typesupport_target}")
target_include_directories(nodeRawdata_surface PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(nodeRawdata_surface
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(nodeTrackPacket src/nodeTrackPacket.cpp src/an_packet_protocol.c src/subsonus_packets.c)
ament_target_dependencies(nodeTrackPacket rclcpp std_msgs geometry_msgs tf2_ros tf2 visualization_msgs tf2_geometry_msgs)
target_link_libraries(nodeTrackPacket "${cpp_typesupport_target}")
target_include_directories(nodeTrackPacket PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(nodeTrackPacket
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(nodeRawdata_ROV src/nodeRawdata_ROV.cpp src/an_packet_protocol.c src/subsonus_packets.c)
ament_target_dependencies(nodeRawdata_ROV rclcpp std_msgs geometry_msgs tf2_ros tf2 visualization_msgs tf2_geometry_msgs)
target_link_libraries(nodeRawdata_ROV "${cpp_typesupport_target}")
target_include_directories(nodeRawdata_ROV PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(nodeRawdata_ROV
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  nodeRawdata_surface
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  nodeRawdata_ROV
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  nodeTrackPacket
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
