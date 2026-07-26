cmake_minimum_required(VERSION 3.8)
project(jetpilot_bridge_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(jetpilot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_protocol
  src/bridge_protocol.cpp
)
target_compile_features(${PROJECT_NAME}_protocol PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(jetpilot_bridge_interface_node
  src/jetpilot_bridge_interface_node.cpp
)
target_compile_features(jetpilot_bridge_interface_node PUBLIC cxx_std_17)
target_link_libraries(jetpilot_bridge_interface_node ${PROJECT_NAME}_protocol)
ament_target_dependencies(jetpilot_bridge_interface_node
  diagnostic_msgs
  jetpilot_msgs
  rclcpp
  std_msgs
)

install(TARGETS
  ${PROJECT_NAME}_protocol
  jetpilot_bridge_interface_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bridge_protocol test/test_bridge_protocol.cpp)
  target_link_libraries(test_bridge_protocol ${PROJECT_NAME}_protocol)
endif()

ament_package()
