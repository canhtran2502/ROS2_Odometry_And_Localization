# 06 - Odometry & Motion Control

## A. Objective

This document details the kinematics, odometry integration algorithms, control architectures, and velocity multiplexing systems implemented in BumperBot. It covers both the standard `ros2_control` differential-drive implementation and custom analytical kinematic nodes in C++ and Python.

---

## B. Differential-Drive Kinematics Theory

```
            ^ +X (Heading)
            |
            |     v (Linear Velocity)
            +------------->
           / \   w (Angular Velocity)
          /   \
  Left   |     |   Right
  Wheel  |     |   Wheel
  [w_L]  +-----+   [w_R]
            L (Track Width)
```

### 1. Forward Kinematics
Given the angular velocities of the right wheel ($\omega_R$) and left wheel ($\omega_L$), the robot's linear velocity $v$ and angular velocity $\omega$ in the robot body frame are:

$$v = \frac{r}{2}(\omega_R + \omega_L)$$

$$\omega = \frac{r}{L}(\omega_R - \omega_L)$$

In matrix form:
$$\begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} \frac{r}{2} & \frac{r}{2} \\ \frac{r}{L} & -\frac{r}{L} \end{bmatrix} \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}$$

For BumperBot ($r = 0.033\text{ m}, L = 0.170\text{ m}$):
$$\begin{bmatrix} v \\ \omega \end{bmatrix} = \begin{bmatrix} 0.0165 & 0.0165 \\ 0.1941 & -0.1941 \end{bmatrix} \begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix}$$

### 2. Inverse Kinematics
Given a desired commanded velocity $[v, \omega]^T$ (from teleoperation or an autonomous planner), the required individual wheel angular velocities are:

$$\omega_R = \frac{v + \omega \frac{L}{2}}{r}$$

$$\omega_L = \frac{v - \omega \frac{L}{2}}{r}$$

In matrix form:
$$\begin{bmatrix} \omega_R \\ \omega_L \end{bmatrix} = \begin{bmatrix} \frac{1}{r} & \frac{L}{2r} \\ \frac{1}{r} & -\frac{L}{2r} \end{bmatrix} \begin{bmatrix} v \\ \omega \end{bmatrix}$$

### 3. Discrete Odometry Integration
Given encoder angle increments $\Delta \phi_R$ and $\Delta \phi_L$ over sampling period $\Delta t$:
1. Linear distance traveled by each wheel:
   $$\Delta s_R = r \cdot \Delta \phi_R, \quad \Delta s_L = r \cdot \Delta \phi_L$$
2. Robot chassis linear displacement and change in heading:
   $$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \quad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{L}$$
3. Pose update in the fixed `odom` frame using midpoint integration:
   $$\theta_{k+1} = \theta_k + \Delta \theta$$
   $$x_{k+1} = x_k + \Delta s \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)$$
   $$y_{k+1} = y_k + \Delta s \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)$$

---

## C. Control Implementations in the Repository

The workspace provides two alternative control pathways selected via launch arguments:

### 1. Production Mode: `diff_drive_controller` (`ros2_control`)
- **Configuration**: `src/bumperbot_controller/config/bumperbot_controllers.yaml`
- **Features**:
  - Open-loop acceleration and velocity limits:
    - Linear $x$: Max vel $0.7\text{ m/s}$, Max accel $0.7\text{ m/s}^2$
    - Angular $z$: Max vel $8.5\text{ rad/s}$, Max accel $5.0\text{ rad/s}^2$
  - Command timeout: $0.5\text{ s}$ (stops motors if heartbeat lost)
  - Broadcasts `odom -> base_footprint` TF directly at 50 Hz.
  - Publishes `/bumperbot_controller/odom` with covariance diagonals.

### 2. Analytical Mode: `simple_controller` (`simple_controller.cpp` / `.py`)
- **Purpose**: Demonstrates direct implementation of the kinematic equations without relying on pre-built plugins.
- **Implementation**:
  - Subscribes to `/bumperbot_controller/cmd_vel` (`TwistStamped`).
  - Computes $[\omega_R, \omega_L]$ and publishes to `/simple_velocity_controller/commands`.
  - Subscribes to `/joint_states` to read wheel positions.
  - Integrates odometry and broadcasts `odom -> base_footprint` TF.

### 3. Noise Simulation Mode: `noisy_controller` (`noisy_controller.cpp` / `.py`)
- **Purpose**: Simulates real-world mechanical degradation, encoder quantisation error, and wheel slip.
- **Implementation**:
  - Reads `/joint_states` and adds zero-mean Gaussian noise:
    $$s_{\text{noisy}} = s + \mathcal{N}(0, 0.005\text{ m})$$
    $$\theta_{\text{noisy}} = \theta + \mathcal{N}(0, 0.001\text{ rad})$$
  - Publishes to `/bumperbot_controller/odom_noisy` and broadcasts `odom -> base_footprint_noisy`.
  - Provides the corrupted measurement source used by `kalman_filter` and `robot_localization` EKF.

---

## D. Teleoperation & Priority Multiplexing (`twist_mux`)

The teleoperation architecture protects against command conflicts and operator error through priority arbitration:

```
[Linux Joystick /joy] ---> [joy_teleop] ---> [twist_relay] ---> /input_joy/cmd_vel (Priority: 99)
                                                                            |
[Keyboard Teleop]     ---------------------------------------> /key_vel    (Priority: 90)
                                                                            |
                                                                            v
[safety_stop_node]    ---------------------------------------> [twist_mux Core]
                      Publishes /safety_stop (Lock Priority: 255)           |
                                                                            v
                                                             /bumperbot_controller/cmd_vel_unstamped
                                                                            |
                                                                            v
                                                                     [twist_relay]
                                                                            |
                                                                            v
                                                             /bumperbot_controller/cmd_vel (Stamped)
                                                                            |
                                                                            v
                                                                   [diff_drive_controller]
```

### Priority Configuration Table

| Source Channel | Topic / Lock Name | Priority | Timeout | Behavior |
|---|---|---|---|---|
| **Safety Lock** | `/safety_stop` | **255** | `0.0` s | When `True`, completely disables all velocity commands |
| **Joystick** | `joy_vel` | **99** | `0.5` s | Overrides keyboard when active |
| **Keyboard** | `key_vel` | **90** | `0.5` s | Fallback manual input channel |

---

## E. Practical Verification & Testing Commands

```bash
# 1. Launch controller stack with diff_drive_controller (default)
ros2 launch bumperbot_controller controller.launch.py

# 2. Launch controller stack using custom analytical simple_controller
ros2 launch bumperbot_controller controller.launch.py use_simple_controller:=True

# 3. Launch complete teleoperation stack with joystick and twist_mux
ros2 launch bumperbot_controller joystick_teleop.launch.py

# 4. Drive the robot forward manually via CLI topic publication
ros2 topic pub --once /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_footprint'},
  twist: {linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
}"

# 5. Echo live odometry output
ros2 topic echo /bumperbot_controller/odom --field pose.pose.position
```

