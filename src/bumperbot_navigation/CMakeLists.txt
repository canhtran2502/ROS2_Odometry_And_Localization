cmake_minimum_required(VERSION 3.8)
project(bumperbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(bumperbot_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

# Pure mission logic, no ROS dependency
add_library(mission_logic src/mission_logic.cpp)
target_include_directories(mission_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_logic PUBLIC yaml-cpp)

add_executable(mission_manager src/mission_manager.cpp)
target_link_libraries(mission_manager mission_logic)
ament_target_dependencies(mission_manager
  rclcpp rclcpp_action bumperbot_msgs nav2_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  ament_index_cpp)

install(TARGETS mission_logic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mission_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

install(PROGRAMS
  ${PROJECT_NAME}/mission_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Tests are registered only when the files are present
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_logic.cpp)
    ament_add_gtest(test_mission_logic_cpp test/test_mission_logic.cpp)
    target_link_libraries(test_mission_logic_cpp mission_logic)
    target_compile_definitions(test_mission_logic_cpp PRIVATE
      TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
      CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endif()

  foreach(py_test
      test_mission_logic
      test_stations_on_map
      test_interfaces_and_static
      test_mission_docs)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/${py_test}.py)
      ament_add_pytest_test(${py_test} test/${py_test}.py
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/test
        TIMEOUT 120)
    endif()
  endforeach()

  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_manager.launch.py)
    add_launch_test(test/test_mission_manager.launch.py
      TARGET test_mission_manager TIMEOUT 300)
  endif()

  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_review_fixes.launch.py)
    add_launch_test(test/test_mission_review_fixes.launch.py
      TARGET test_mission_review_fixes TIMEOUT 300)
  endif()
endif()

ament_package()
