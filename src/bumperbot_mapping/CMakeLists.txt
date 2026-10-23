cmake_minimum_required(VERSION 3.8)
project(bumperbot_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories((include))

add_executable(mapping_with_known_poses src/mapping_with_known_poses.cpp)
ament_target_dependencies(mapping_with_known_poses rclcpp sensor_msgs tf2_ros tf2 nav_msgs geometry_msgs)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  mapping_with_known_poses
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/mapping_with_known_poses.py
  ${PROJECT_NAME}/frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Exploration tests (registered only when the files are present)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(_pythonpath "${CMAKE_CURRENT_SOURCE_DIR}")
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_frontier_exploration.py)
    ament_add_pytest_test(test_frontier_exploration test/test_frontier_exploration.py
      APPEND_ENV PYTHONPATH=${_pythonpath} TIMEOUT 120)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_exploration_synthetic.py)
    ament_add_pytest_test(test_exploration_synthetic test/test_exploration_synthetic.py
      APPEND_ENV PYTHONPATH=${_pythonpath} TIMEOUT 600)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_map_compare.py)
    ament_add_pytest_test(test_map_compare test/test_map_compare.py
      APPEND_ENV PYTHONPATH=${_pythonpath} TIMEOUT 300)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_frontier_explorer_node.launch.py)
    add_launch_test(test/test_frontier_explorer_node.launch.py
      TARGET test_frontier_explorer_node TIMEOUT 90)
  endif()
endif()

ament_package()
