cmake_minimum_required(VERSION 3.8)
project(joint_position_controller_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(joint_position_controller_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(
  joint_position_controller_server_node
  src/joint_position_controller_server_node.cpp
  src/joint_position_controller_server.cpp)

target_link_libraries(
  joint_position_controller_server_node
  ${joint_position_controller_interfaces_TARGETS})

ament_target_dependencies(
  joint_position_controller_server_node rclcpp rclcpp_action sensor_msgs
  std_msgs joint_position_controller_interfaces)

target_include_directories(
  joint_position_controller_server_node
  PRIVATE include)

add_executable(
  joint_position_controller_client_node
  src/joint_position_controller_client_node.cpp
  src/joint_position_controller_client.cpp)

target_link_libraries(
  joint_position_controller_client_node
  ${joint_position_controller_interfaces_TARGETS})

ament_target_dependencies(
  joint_position_controller_client_node rclcpp rclcpp_action
  joint_position_controller_interfaces)

target_include_directories(joint_position_controller_client_node PRIVATE include)

add_executable(
  joint_position_controller_client_single_goal_node
  src/joint_position_controller_client_node.cpp
  src/joint_position_controller_client.cpp)

target_link_libraries(
  joint_position_controller_client_single_goal_node
  ${joint_position_controller_interfaces_TARGETS})

ament_target_dependencies(
  joint_position_controller_client_single_goal_node
  rclcpp
  rclcpp_action
  joint_position_controller_interfaces)

target_include_directories(joint_position_controller_client_single_goal_node PRIVATE include)

add_executable(
  joint_position_controller_client_single_goal_sequential_node
  src/joint_position_controller_client_single_goal_sequential_node.cpp
  src/joint_position_controller_client_single_goal_sequential.cpp)

target_link_libraries(
  joint_position_controller_client_single_goal_sequential_node
  ${joint_position_controller_interfaces_TARGETS})

ament_target_dependencies(
  joint_position_controller_client_single_goal_sequential_node rclcpp
  rclcpp_action joint_position_controller_interfaces)

target_include_directories(
  joint_position_controller_client_single_goal_sequential_node PRIVATE include)

install(
  FILES launch/joint_position_controller_server.launch.py
        launch/prismatic_joint_position_serial_driver.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

install(
  FILES config/example_joint_position_controller_server.yaml
        config/example_prismatic_joint_position_serial_driver.yaml
  DESTINATION share/${PROJECT_NAME}/config)

install(FILES README.md LICENSE CONTRIBUTING.md DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    joint_position_controller_server/prismatic_joint_position_serial_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME prismatic_joint_position_serial_driver_node)

install(
  TARGETS joint_position_controller_server_node
          joint_position_controller_client_node
          joint_position_controller_client_single_goal_node
          joint_position_controller_client_single_goal_sequential_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Exclude linters that require package-specific configuration.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
    ament_cmake_cpplint
    ament_cmake_flake8
    ament_cmake_cppcheck)

  # Register the remaining linters declared as test dependencies in package.xml.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format REQUIRED)

  ament_clang_format(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format" include src test)

  find_package(ament_cmake_cpplint REQUIRED)

  ament_cpplint(
    MAX_LINE_LENGTH
    120
    FILTERS
    -whitespace/newline
    -whitespace/forcolon
    -build/include_order
    -runtime/indentation_namespace
    -legal/copyright)

  find_package(ament_cmake_flake8 REQUIRED)

  ament_flake8(CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ament_flake8.ini")

  find_package(ament_cmake_cppcheck REQUIRED)

  # Check only headers and source files.
  # Some gtest macros such as TEST_F and TEST_P can produce cppcheck false positives in test
  # sources.
  ament_cppcheck(include src INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")

  # ament_cppcheck skips cppcheck versions known to be slow unless this environment variable
  # explicitly allows them.
  set_tests_properties(cppcheck PROPERTIES ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")

  ament_add_gtest(
    test_joint_position_controller_server
    test/test_joint_position_controller_server.cpp
    src/joint_position_controller_server.cpp
    src/joint_position_controller_client.cpp
    src/joint_position_controller_client_single_goal_sequential.cpp)

  target_link_libraries(
    test_joint_position_controller_server
    ${joint_position_controller_interfaces_TARGETS})

  ament_target_dependencies(
    test_joint_position_controller_server rclcpp rclcpp_action sensor_msgs
    std_msgs joint_position_controller_interfaces)

  target_include_directories(
    test_joint_position_controller_server
    PRIVATE include)

  ament_add_pytest_test(
    test_python test TIMEOUT 90 ENV
    "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
endif()

ament_package()
