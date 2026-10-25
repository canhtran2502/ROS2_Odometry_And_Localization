cmake_minimum_required(VERSION 3.8)
project(bumperbot_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bumperbot_msgs REQUIRED)
find_package(twist_mux_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

add_executable(safety_stop src/safety_stop.cpp)
ament_target_dependencies(safety_stop rclcpp rclcpp_action std_msgs sensor_msgs twist_mux_msgs visualization_msgs)

# Pure safety logic, no ROS dependency
add_library(safety_logic src/safety_logic.cpp)
target_include_directories(safety_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(safety_supervisor src/safety_supervisor.cpp)
target_link_libraries(safety_supervisor safety_logic)
ament_target_dependencies(safety_supervisor
  rclcpp std_msgs std_srvs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros bumperbot_msgs)

install(TARGETS safety_logic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  safety_stop
  safety_supervisor
  DESTINATION lib/${PROJECT_NAME}
)

install (PROGRAMS
  ${PROJECT_NAME}/safety_stop.py
  ${PROJECT_NAME}/safety_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
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
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_safety_logic.cpp)
    find_package(yaml_cpp_vendor REQUIRED)
    ament_add_gtest(test_safety_logic_cpp test/test_safety_logic.cpp TIMEOUT 120)
    target_link_libraries(test_safety_logic_cpp safety_logic yaml-cpp)
    target_compile_definitions(test_safety_logic_cpp PRIVATE
      TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
      CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  endif()

  foreach(py_test
      test_safety_logic
      test_safety_static
      test_safety_docs)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/${py_test}.py)
      ament_add_pytest_test(${py_test} test/${py_test}.py
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/test
        TIMEOUT 120)
    endif()
  endforeach()

  # Process-level robustness tests start both safety_supervisor executables
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_safety_robustness.py)
    ament_add_pytest_test(test_safety_robustness test/test_safety_robustness.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/test
      TIMEOUT 600)
  endif()

  # Node tests run both executables against MockSafetyWorld
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_safety_supervisor.launch.py)
    add_launch_test(test/test_safety_supervisor.launch.py
      TARGET test_safety_supervisor TIMEOUT 900)
  endif()
endif()

ament_package()
