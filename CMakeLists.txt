cmake_minimum_required(VERSION 3.8)
project(fork_position_controller_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(fork_position_controller_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(
  ${PROJECT_NAME}_nodes
  src/fork_position_controller_client.cpp
  src/fork_position_controller_client_single_goal_sequential.cpp
  src/fork_position_controller_server.cpp)

target_include_directories(
  ${PROJECT_NAME}_nodes
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_nodes rclcpp rclcpp_action
                          sensor_msgs std_msgs
                          fork_position_controller_interfaces)
target_link_libraries(${PROJECT_NAME}_nodes
                      ${fork_position_controller_interfaces_TARGETS})

add_executable(fork_position_controller_server_node
               src/fork_position_controller_server_node.cpp)
target_link_libraries(fork_position_controller_server_node
                      ${PROJECT_NAME}_nodes)
ament_target_dependencies(fork_position_controller_server_node rclcpp)
target_include_directories(fork_position_controller_server_node PRIVATE include)

add_executable(fork_position_controller_client_node
               src/fork_position_controller_client_node.cpp)
target_link_libraries(fork_position_controller_client_node
                      ${PROJECT_NAME}_nodes)
ament_target_dependencies(fork_position_controller_client_node rclcpp)
target_include_directories(fork_position_controller_client_node PRIVATE include)

add_executable(fork_position_controller_client_single_goal_node
               src/fork_position_controller_client_node.cpp)
target_link_libraries(fork_position_controller_client_single_goal_node
                      ${PROJECT_NAME}_nodes)
ament_target_dependencies(fork_position_controller_client_single_goal_node
                          rclcpp)
target_include_directories(fork_position_controller_client_single_goal_node
                           PRIVATE include)

add_executable(
  fork_position_controller_client_single_goal_sequential_node
  src/fork_position_controller_client_single_goal_sequential_node.cpp)
target_link_libraries(
  fork_position_controller_client_single_goal_sequential_node
  ${PROJECT_NAME}_nodes)
ament_target_dependencies(
  fork_position_controller_client_single_goal_sequential_node rclcpp)
target_include_directories(
  fork_position_controller_client_single_goal_sequential_node PRIVATE include)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(
  PROGRAMS fork_position_controller_server/fork_serial_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fork_serial_node)

install(
  TARGETS ${PROJECT_NAME}_nodes
          fork_position_controller_server_node
          fork_position_controller_client_node
          fork_position_controller_client_single_goal_node
          fork_position_controller_client_single_goal_sequential_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Turn on as needed: ament_lint_auto_find_test_dependencies()
endif()

ament_package()
