cmake_minimum_required(VERSION 3.8)
project(jetpilot_bridge_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_protocol SHARED
  src/bridge_protocol.cpp
)
target_compile_features(${PROJECT_NAME}_protocol PUBLIC cxx_std_17)

ament_auto_add_library(${PROJECT_NAME}_node_impl SHARED
  src/jetpilot_bridge_interface_node.cpp
)
target_compile_features(${PROJECT_NAME}_node_impl PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node_impl ${PROJECT_NAME}_protocol)

ament_auto_add_executable(jetpilot_bridge_interface_node
  src/jetpilot_bridge_interface_node_main.cpp
)
target_compile_features(jetpilot_bridge_interface_node PUBLIC cxx_std_17)
target_link_libraries(jetpilot_bridge_interface_node ${PROJECT_NAME}_node_impl)

install(DIRECTORY
  config
  firmware
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bridge_protocol test/test_bridge_protocol.cpp)
  target_link_libraries(test_bridge_protocol ${PROJECT_NAME}_protocol)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
