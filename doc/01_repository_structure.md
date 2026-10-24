# 01 - Repository Structure & Package Breakdown

## A. Objective

This document provides a complete anatomical directory map of the BumperBot ROS 2 workspace (`bumperbot_ws`). It outlines the directory hierarchy, categorizes package build types, details critical source and configuration files, and maps compile-time and runtime package dependencies.

---

## B. Complete Workspace Directory Tree

*(Excluding build artifacts `build/`, `install/`, and `log/`)*

```
bumperbot_ws/
├── README.txt                                         # Root quickstart and operational reference
├── README.md                                          # Original repository stub
├── doc/                                               # System engineering documentation
│   ├── 00_project_overview.md
│   ├── 01_repository_structure.md
│   ├── 02_system_architecture.md
│   ├── 03_ros2_nodes_topics_services.md
│   ├── 04_tf_and_coordinate_frames.md
│   ├── 05_robot_description_and_simulation.md
│   ├── 06_odometry_and_motion_control.md
│   ├── 07_lidar_and_sensor_processing.md
│   ├── 08_mapping_and_slam.md
│   ├── 09_localization.md
│   ├── 10_navigation.md
│   ├── 11_build_run_and_command_reference.md
│   └── 12_troubleshooting.md
└── src/
    ├── bumperbot_bringup/                             # Top-level launch orchestrations
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── launch/
    │       ├── real_robot.launch.py                   # Physical AMR hardware launch
    │       ├── simulated_robot.launch.py              # Full simulation bringup (Gazebo/MuJoCo, RViz/SLAM/AMCL)
    │       ├── auto_explore.launch.py                 # SLAM + frontier_explorer autonomous exploration
    │       ├── mission_sim.launch.py                  # Simulator + AMCL + Nav2 + mission_manager (AGV missions)
    │       └── dock_sim.launch.py                     # mission_sim + dock_manager + battery_sim + dock marker emulator
    │
    ├── bumperbot_controller/                          # Kinematics, odometry, velocity multiplexing
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── bumperbot_controller/                      # Python package modules
    │   │   ├── __init__.py
    │   │   ├── simple_controller.py                   # Analytical differential-drive odometry & control
    │   │   ├── noisy_controller.py                    # Synthetic Gaussian noise odometry injector
    │   │   └── twist_relay.py                         # Twist stamped <-> unstamped translation relay
    │   ├── include/bumperbot_controller/
    │   │   ├── simple_controller.hpp
    │   │   └── noisy_controller.hpp
    │   ├── src/
    │   │   ├── simple_controller.cpp
    │   │   ├── noisy_controller.cpp
    │   │   └── twist_relay.cpp
    │   ├── config/
    │   │   ├── bumperbot_controllers.yaml             # ros2_control diff_drive_controller parameters
    │   │   ├── joy_config.yaml                        # Linux joystick device driver configuration
    │   │   ├── joy_teleop.yaml                        # Button-to-Twist axis mappings
    │   │   ├── twist_mux_topics.yaml                  # Priority input channels (joy:99, key:90, dock:30, nav:20, explore:10)
    │   │   ├── twist_mux_locks.yaml                   # Safety stop emergency inhibitor lock (255)
    │   │   └── twist_mux_joy.yaml                     # Turbo speed scaling configuration
    │   └── launch/
    │       ├── controller.launch.py                   # ros2_control spawner & odometry launch
    │       └── joystick_teleop.launch.py              # Joy, teleop, twist_mux, and relay launch
    │
    ├── bumperbot_description/                         # Robot kinematics, visual/collision meshes, worlds
    ├── bumperbot_mujoco/                              # MuJoCo models, ROS 2 bridge node, SDF/URDF converters
    │   │                                              # (also /mujoco/ground_truth, dock_marker_sim.py + config/dock_marker_sim.yaml, sim only)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── urdf/
    │   │   ├── bumperbot.urdf.xacro                   # Mechanical link/joint hierarchy & inertial tensors
    │   │   ├── bumperbot_gazebo.xacro                 # Ignition Gazebo sensor & control plugins
    │   │   └── bumperbot_ros2_control.xacro           # ros2_control hardware system interface tags
    │   ├── meshes/                                    # STL surface geometry for links and wheels
    │   ├── models/                                    # AWS RoboMaker residential/warehouse 3D SDF models
    │   ├── worlds/
    │   │   ├── empty.world                            # Minimal flat plane environment
    │   │   ├── small_house.world                      # Complex residential environment (80+ models)
    │   │   └── small_warehouse.world                  # Industrial warehouse environment
    │   ├── rviz/
    │   │   └── display.rviz                           # Model visualization layout
    │   └── launch/
    │       ├── display.launch.py                      # Joint state publisher GUI + RViz2 model viewer
    │       └── gazebo.launch.py                       # Ignition Gazebo simulation & spawn launch
    │
    ├── bumperbot_firmware/                            # Physical actuator interface, drivers, MCU code
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── bumperbot_interface.xml                    # pluginlib export for ros2_control SystemInterface
    │   ├── include/bumperbot_firmware/
    │   │   └── bumperbot_interface.hpp                # Hardware interface header
    │   ├── src/
    │   │   ├── bumperbot_interface.cpp                # LibSerial-based hardware interface implementation
    │   │   ├── simple_serial_receiver.cpp             # Raw serial byte stream reader test utility
    │   │   └── simple_serial_transmitter.cpp          # Raw serial byte stream writer test utility
    │   ├── bumperbot_firmware/
    │   │   ├── __init__.py
    │   │   ├── mpu6050_driver.py                      # SMBus I2C Linux driver for MPU6050 IMU
    │   │   ├── simple_serial_receiver.py              # Python pyserial reader test node
    │   │   └── simple_serial_transmitter.py           # Python pyserial writer test node
    │   ├── firmware/                                  # Arduino AVR / Microcontroller source sketches
    │   │   ├── robot_control/robot_control.ino        # Dual closed-loop PID velocity controller
    │   │   ├── simple_motor_control/                  # Open-loop PWM motor test sketch
    │   │   ├── simple_encoder_reader/                 # Quadrature interrupt encoder counter sketch
    │   │   ├── simple_serial_receiver/                # Serial packet command parser sketch
    │   │   └── simple_serial_transmitter/             # Serial encoder telemetric transmitter sketch
    │   └── launch/
    │       └── hardware_interface.launch.py           # ros2_control_node launch for physical hardware
    │
    ├── bumperbot_localization/                        # State estimation, sensor fusion, AMCL
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/bumperbot_localization/
    │   │   ├── kalman_filter.hpp                      # 1D linear Kalman filter header
    │   │   └── odometry_motion_model.hpp              # Probabilistic sampling model header
    │   ├── src/
    │   │   ├── kalman_filter.cpp                      # 1D linear Kalman filter implementation
    │   │   ├── odometry_motion_model.cpp              # Thrun sample odometry model implementation
    │   │   └── imu_republisher.cpp                    # IMU reference frame sign converter
    │   ├── bumperbot_localization/
    │   │   ├── __init__.py
    │   │   ├── kalman_filter.py                       # Python 1D Kalman filter node
    │   │   ├── odometry_motion_model.py               # Python sample odometry motion model node
    │   │   └── imu_republisher.py                     # Python IMU frame adapter node
    │   ├── config/
    │   │   ├── ekf.yaml                               # robot_localization ekf_node configuration
    │   │   └── amcl.yaml                              # nav2_amcl particle filter configuration
    │   ├── rviz/
    │   │   └── global_localization.rviz               # RViz2 layout with map, particle cloud, scan
    │   └── launch/
    │       ├── local_localization.launch.py           # EKF node + IMU republisher launch
    │       └── global_localization.launch.py          # nav2_map_server + nav2_amcl + lifecycle launch
    │
    ├── bumperbot_mapping/                             # Occupancy grid generation & SLAM Toolbox
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/bumperbot_mapping/
    │   │   └── mapping_with_known_poses.hpp           # Known-pose occupancy mapper header
    │   ├── src/
    │   │   └── mapping_with_known_poses.cpp           # Known-pose ray projection grid generator
    │   ├── bumperbot_mapping/
    │   │   ├── __init__.py
    │   │   ├── mapping_with_known_poses.py            # Multi-threaded Python occupancy grid generator
    │   │   ├── frontier_exploration.py                # Pure-Python exploration core (frontiers, A*, pure pursuit, state machine)
    │   │   └── frontier_explorer.py                   # ROS 2 node wrapper: autonomous frontier exploration + map saving
    │   ├── config/
    │   │   ├── slam_toolbox.yaml                      # Ceres-solver scan-matching SLAM configuration
    │   │   └── frontier_explorer.yaml                 # frontier_explorer parameters
    │   ├── maps/
    │   │   └── small_house/                           # Pre-built map assets
    │   │       ├── map.yaml                           # Occupancy grid metadata
    │   │       └── map.pgm                            # Grayscale 2D occupancy raster
    │   ├── rviz/
    │   │   └── slam.rviz                              # RViz2 layout for real-time map building
    │   └── launch/
    │       └── slam.launch.py                         # slam_toolbox + map_saver + lifecycle launch
    │
    ├── bumperbot_utils/                               # Autonomous supervisory & safety utilities
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── src/
    │   │   └── safety_stop.cpp                        # C++ 2-tier reactive safety supervisor node
    │   └── bumperbot_utils/
    │       ├── __init__.py
    │       └── safety_stop.py                         # Python 2-tier reactive safety supervisor node
    │
    ├── bumperbot_navigation/                          # Nav2 configuration + AGV waypoint mission manager
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/bumperbot_navigation/
    │   │   ├── mission_logic.hpp                      # C++ pure mission logic API (no rclcpp)
    │   │   ├── mission_manager.hpp                    # C++ mission_manager node class
    │   │   ├── dock_logic.hpp                         # C++ pure docking logic API (no rclcpp)
    │   │   └── dock_manager.hpp                       # C++ dock_manager node class
    │   ├── src/
    │   │   ├── mission_logic.cpp                      # Station loading (yaml-cpp) + MissionTracker
    │   │   ├── mission_manager.cpp                    # C++ /execute_mission server + Nav2 client
    │   │   ├── dock_logic.cpp                         # Docking geometry, controller, DockTracker
    │   │   └── dock_manager.cpp                       # C++ /dock_robot server + Nav2 client
    │   ├── bumperbot_navigation/
    │   │   ├── mission_logic.py                       # Python twin of mission_logic.cpp (no rclpy)
    │   │   ├── mission_manager.py                     # Python twin of mission_manager.cpp
    │   │   ├── dock_logic.py                          # Python twin of dock_logic.cpp (no rclpy)
    │   │   ├── dock_manager.py                        # Python twin of dock_manager.cpp
    │   │   ├── battery_logic.py                       # BatteryModel (sim) + BatteryMonitorLogic (no rclpy)
    │   │   ├── battery_monitor.py                     # Low-battery dock trigger, undock when full
    │   │   └── battery_sim.py                         # SIMULATION ONLY: /battery_state publisher
    │   ├── config/
    │   │   ├── stations.yaml                          # Named stations on small_house, incl. the dock
    │   │   ├── mission_manager.yaml                   # mission_manager parameters
    │   │   ├── dock_manager.yaml                      # dock_manager parameters
    │   │   ├── battery_monitor.yaml                   # battery_monitor parameters
    │   │   ├── battery_sim.yaml                       # SIMULATION ONLY: battery model parameters
    │   │   └── nav2_params.yaml                       # Nav2 parameters for BumperBot
    │   ├── launch/
    │   │   ├── navigation.launch.py                   # Nav2 servers + lifecycle manager
    │   │   └── docking.launch.py                      # dock_manager + battery_monitor (simulator independent)
    │   └── test/                                      # Unit, launch and static tests + shared test data
    │
    ├── bumperbot_msgs/                                # Custom ROS 2 IDL interface definitions
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── srv/
    │   │   ├── AddTwoInts.srv                         # Standard arithmetic request/response
    │   │   └── GetTransform.srv                       # TF lookup service returning TransformStamped
    │   ├── msg/
    │   │   └── MissionStatus.msg                      # /mission/status of the mission manager
    │   └── action/
    │       ├── ExecuteMission.action                  # Ordered station mission (goal/feedback/result)
    │       └── Fibonacci.action                       # Asynchronous iterative goal/feedback/result
    │
    ├── bumperbot_cpp_examples/                        # C++ rclcpp pedagogical reference implementations
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/bumperbot_cpp_examples/
    │   │   ├── simple_tf_kinematics.hpp
    │   │   └── simple_turtlesim_kinematics.hpp
    │   └── src/
    │       ├── simple_publisher.cpp                   # String publisher (500ms timer)
    │       ├── simple_subscriber.cpp                  # String subscriber
    │       ├── simple_parameter.cpp                   # Dynamic parameter callbacks
    │       ├── simple_qos_publisher.cpp               # Reliable/Best-Effort & Volatile/Transient QoS
    │       ├── simple_qos_subscriber.cpp              # QoS subscriber compatibility checks
    │       ├── simple_service_server.cpp              # AddTwoInts service implementation
    │       ├── simple_service_client.cpp              # AddTwoInts asynchronous service invocation
    │       ├── simple_action_server.cpp               # Fibonacci action goal/cancel/execute server
    │       ├── simple_action_client.cpp               # Fibonacci action client with feedback callbacks
    │       ├── simple_lifecycle_node.cpp              # Managed node state transitions
    │       ├── simple_tf_kinematics.cpp               # TF dynamic broadcast & GetTransform server
    │       └── simple_turtlesim_kinematics.cpp        # Turtlesim pose subscriber & transform math
    │
    └── bumperbot_py_examples/                         # Python rclpy pedagogical reference implementations
        ├── package.xml
        ├── setup.py                                   # Python packaging metadata & entry points
        ├── setup.cfg
        └── bumperbot_py_examples/
            ├── simple_publisher.py
            ├── simple_subscriber.py
            ├── simple_parameter.py
            ├── simple_qos_publisher.py
            ├── simple_qos_subscriber.py
            ├── simple_service_server.py
            ├── simple_service_client.py
            ├── simple_action_server.py
            ├── simple_action_client.py
            ├── simple_lifecycle_node.py
            ├── simple_tf_kinematics.py
            └── simple_turtlesim_kinematics.py
```

---

## C. Package Dependency & Build Graph

The repository dependencies form a clean layered architecture without circular references:

```
                      +-------------------+
                      |  bumperbot_msgs   |
                      +-------------------+
                                ^
            +-------------------+-------------------+
            |                                       |
+--------------------------+             +--------------------------+
|  bumperbot_cpp_examples  |             |  bumperbot_py_examples   |
+--------------------------+             +--------------------------+

                      +-------------------------+
                      | bumperbot_description   |
                      +-------------------------+
                                ^
            +-------------------+-------------------+
            |                                       |
+-------------------------+             +-------------------------+
|  bumperbot_firmware     |             |  bumperbot_controller   |
+-------------------------+             +-------------------------+
            ^                                       ^
            |                                       |
+-------------------------+             +-------------------------+
|  bumperbot_utils        |             |  bumperbot_localization |
+-------------------------+             +-------------------------+
                                                    ^
                                                    |
                                        +-------------------------+
                                        |   bumperbot_mapping     |
                                        +-------------------------+
                                                    ^
                                                    |
                                        +-------------------------+
                                        |   bumperbot_bringup     |
                                        +-------------------------+
```

### Dependency Classification Table

| Package | Build Tool | Direct Workspace Dependencies | Primary External Dependencies |
|---|---|---|---|
| `bumperbot_msgs` | `ament_cmake` | None | `rosidl_default_generators`, `geometry_msgs`, `builtin_interfaces` |
| `bumperbot_description` | `ament_cmake` | None | `urdf`, `xacro`, `ros_gz_sim`, `ros_gz_bridge` |
| `bumperbot_controller` | `ament_cmake` | None | `rclcpp`, `rclpy`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2`, `tf2_ros`, `Eigen3`, `diff_drive_controller` |
| `bumperbot_mujoco` | `ament_python` | `bumperbot_description`, `bumperbot_controller` | `rclpy`, `sensor_msgs`, `topic_based_ros2_control`, `mujoco` (pip) |
| `bumperbot_firmware` | `ament_cmake` | `bumperbot_controller`, `bumperbot_description` | `hardware_interface`, `pluginlib`, `rclcpp`, `rclpy`, `libserial-dev` |
| `bumperbot_utils` | `ament_cmake` | None | `rclcpp`, `rclpy`, `sensor_msgs`, `std_msgs`, `visualization_msgs`, `twist_mux_msgs` |
| `bumperbot_localization`| `ament_cmake` | `bumperbot_controller` | `rclcpp`, `rclpy`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `robot_localization`, `nav2_amcl`, `nav2_map_server` |
| `bumperbot_mapping` | `ament_cmake` | None | `rclcpp`, `rclpy`, `sensor_msgs`, `nav_msgs`, `tf2`, `tf2_ros`, `slam_toolbox`, `nav2_map_server`, `nav2_msgs`, `visualization_msgs`, `numpy`, `scipy` |
| `bumperbot_navigation` | `ament_cmake` | `bumperbot_msgs` | `rclcpp`, `rclcpp_action`, `rclpy`, `nav2_msgs`, `tf2_ros`, `yaml_cpp_vendor`, `nav2_controller`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_lifecycle_manager` |
| `bumperbot_bringup` | `ament_cmake` | `bumperbot_description`, `bumperbot_controller`, `bumperbot_firmware`, `bumperbot_localization`, `bumperbot_mapping`, `bumperbot_utils`, `bumperbot_navigation` | `rviz2` |
| `bumperbot_cpp_examples`| `ament_cmake` | `bumperbot_msgs` | `rclcpp`, `rclcpp_action`, `rclcpp_lifecycle`, `tf2`, `tf2_ros`, `turtlesim` |
| `bumperbot_py_examples` | `ament_python`| `bumperbot_msgs` | `rclpy`, `tf2_ros`, `turtlesim` |

