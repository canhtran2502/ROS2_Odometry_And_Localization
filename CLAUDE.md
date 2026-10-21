# CLAUDE.md

BumperBot: a sample differential-drive mobile robot project on ROS 2 Humble / Ubuntu 22.04.
It covers robot description, `ros2_control`, odometry, sensor fusion, SLAM, localization and a real-hardware interface.
Colcon workspace, packages in `src/`. Never commit `build/`, `install/`, `log/`.

## Docs (read on demand)

Detailed docs live in `doc/` (the folder is named `doc`, not `docs`). Read only the file you need:
- `doc/01_repository_structure.md`: packages and layout
- `doc/02_system_architecture.md`, `doc/03_ros2_nodes_topics_services.md`, `doc/04_tf_and_coordinate_frames.md`: design, topics, TF
- `doc/05_robot_description_and_simulation.md`: robot model and simulation setup
- `doc/11_build_run_and_command_reference.md`: build, run and ROS 2 commands
- `doc/12_troubleshooting.md`: known issues

## Rules

- Do NOT commit or push until the user has reviewed the changes. Leave work uncommitted in the working tree and wait for approval.
- Work only on the session's assigned branch. Do not open PRs unless asked.
- All source-code comments must be in English (C++, Python, xacro, YAML, launch files).
- Keep C++ and Python versions of the same node in sync.
- Do not hardcode absolute paths in launch files.
- Keep simulation and real-robot code paths separate; changes to one must not break the other.
- Keep nodes independent of the simulator: depend only on standard ROS 2 topics, TF and `ros2_control` interfaces.
- Update `README.md` and the relevant `doc/*.md` in the same change as any behavior change.
