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
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_launch_config.py)
    ament_add_pytest_test(test_mission_launch_config test/test_mission_launch_config.py
      TIMEOUT 120)
  endif()
  # Headless MuJoCo mission smoke test; the full mission E2E needs BUMPERBOT_MISSION_E2E=1
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mission_sim_mujoco.launch.py)
    add_launch_test(test/test_mission_sim_mujoco.launch.py
      TARGET test_mission_sim_mujoco TIMEOUT 900)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_auto_explore_mujoco.launch.py)
    add_launch_test(test/test_auto_explore_mujoco.launch.py
      TARGET test_auto_explore_mujoco TIMEOUT 180)
  endif()
  # Full-stack SLAM map quality test: skips itself unless BUMPERBOT_SLAM_E2E=1 (runs ~30 min)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_slam_map_quality_mujoco.launch.py)
    add_launch_test(test/test_slam_map_quality_mujoco.launch.py
      TARGET test_slam_map_quality_mujoco TIMEOUT 2100)
  endif()
  # Auto-docking: launch/config checks, real twist_mux priorities and the headless MuJoCo stack
  # (the full dock/charge/undock E2E needs BUMPERBOT_DOCK_E2E=1)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dock_launch_config.py)
    ament_add_pytest_test(test_dock_launch_config test/test_dock_launch_config.py TIMEOUT 120)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dock_twist_mux.launch.py)
    add_launch_test(test/test_dock_twist_mux.launch.py
      TARGET test_dock_twist_mux TIMEOUT 180)
  endif()
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dock_sim_mujoco.launch.py)
    add_launch_test(test/test_dock_sim_mujoco.launch.py
      TARGET test_dock_sim_mujoco TIMEOUT 1200)
  endif()
endif()

ament_package()
