cmake_minimum_required(VERSION 3.8)
project(bumperbot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Tests are registered only when the files are present
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_config.py)
    ament_add_pytest_test(test_launch_config test/test_launch_config.py TIMEOUT 120)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_auto_explore_mujoco.launch.py)
    add_launch_test(test/test_auto_explore_mujoco.launch.py
      TARGET test_auto_explore_mujoco TIMEOUT 180)
  endif()
endif()

ament_package()
