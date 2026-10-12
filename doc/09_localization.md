# 09 - Robot Localization & Sensor Fusion

## A. Objective

This document covers state estimation, multi-sensor filtering, and probabilistic global localization implemented in BumperBot. It provides the mathematical theory and source code mapping for 1D Kalman Filtering, Thrun's Odometry Motion Model, Multi-Sensor EKF Fusion (`robot_localization`), and Adaptive Monte Carlo Localization (`nav2_amcl`).

---

## B. Mathematical Foundations of State Estimation

### 1. 1D Discrete Linear Kalman Filter
Implemented in `src/bumperbot_localization/src/kalman_filter.cpp` and `kalman_filter.py`, this node demonstrates optimal recursive estimation for a single degree-of-freedom state $x$ (longitudinal displacement).

#### State & Measurement Model:
$$x_k = x_{k-1} + u_k + w_k, \quad w_k \sim \mathcal{N}(0, Q)$$
$$z_k = x_k + v_k, \quad v_k \sim \mathcal{N}(0, R)$$
where:
- $u_k$: Control input displacement ($x_{k,\text{odom}} - x_{k-1,\text{odom}}$).
- $z_k$: Measurement from noisy sensor (`/bumperbot_controller/odom_noisy`).
- $Q$: Process noise variance (default: $Q = 4.0$).
- $R$: Measurement noise variance (default: $R = 9.0$).

#### Prediction Step:
$$\hat{x}_k^- = \hat{x}_{k-1} + u_k$$
$$P_k^- = P_{k-1} + Q$$

#### Update Step:
$$K_k = \frac{P_k^-}{P_k^- + R} \quad (\text{Kalman Gain})$$
$$\hat{x}_k = \hat{x}_k^- + K_k (z_k - \hat{x}_k^-) \quad (\text{State Estimate})$$
$$P_k = (1 - K_k) P_k^- \quad (\text{Error Covariance})$$

---

### 2. Thrun's Probabilistic Sample Odometry Motion Model
Implemented in `src/bumperbot_localization/src/odometry_motion_model.cpp` and `odometry_motion_model.py`, this node visualizes the stochastic probability distribution $p(x_t | u_t, x_{t-1})$ by generating $N = 300$ Monte Carlo particles.

```
       Start Pose (x, y, theta)
                 \
                  \  delta_rot1
                   v
                    +---------------> delta_trans
                                     \
                                      \  delta_rot2
                                       v
                                    End Pose (x', y', theta')
```

#### Relative Motion Decomposition:
Given start pose $\bar{x} = [x, y, \theta]^T$ and end pose $\bar{x}' = [x', y', \theta']^T$:
$$\delta_{\text{rot1}} = \text{atan2}(y' - y, x' - x) - \theta$$
$$\delta_{\text{trans}} = \sqrt{(x' - x)^2 + (y' - y)^2}$$
$$\delta_{\text{rot2}} = \theta' - \theta - \delta_{\text{rot1}}$$

#### Perturbed Odometry Sampling:
Each particle $i \in [1, N]$ samples from Gaussian noise parameterized by error parameters $\alpha_1, \alpha_2, \alpha_3, \alpha_4$:
$$\hat{\delta}_{\text{rot1}} = \delta_{\text{rot1}} - \mathcal{N}\left(0, \alpha_1 \delta_{\text{rot1}}^2 + \alpha_2 \delta_{\text{trans}}^2\right)$$
$$\hat{\delta}_{\text{trans}} = \delta_{\text{trans}} - \mathcal{N}\left(0, \alpha_3 \delta_{\text{trans}}^2 + \alpha_4 (\delta_{\text{rot1}}^2 + \delta_{\text{rot2}}^2)\right)$$
$$\hat{\delta}_{\text{rot2}} = \delta_{\text{rot2}} - \mathcal{N}\left(0, \alpha_1 \delta_{\text{rot2}}^2 + \alpha_2 \delta_{\text{trans}}^2\right)$$

#### Sample Pose Propagation:
$$x^{(i)} = x^{(i)} + \hat{\delta}_{\text{trans}} \cos\left(\theta^{(i)} + \hat{\delta}_{\text{rot1}}\right)$$
$$y^{(i)} = y^{(i)} + \hat{\delta}_{\text{trans}} \sin\left(\theta^{(i)} + \hat{\delta}_{\text{rot1}}\right)$$
$$\theta^{(i)} = \theta^{(i)} + \hat{\delta}_{\text{rot1}} + \hat{\delta}_{\text{rot2}}$$

The node publishes the particle cloud to `/odometry_motion_model/samples` (`geometry_msgs/msg/PoseArray`) for live visualization in RViz2.

---

## C. Multi-Sensor EKF Fusion (`robot_localization`)

Configured in `src/bumperbot_localization/config/ekf.yaml` and launched via `local_localization.launch.py`:

### 1. Extended Kalman Filter State Vector
The `ekf_node` maintains a 15-dimensional state vector $\mathbf{x} \in \mathbb{R}^{15}$:
$$\mathbf{x} = \begin{bmatrix} x & y & z & \text{roll} & \text{pitch} & \text{yaw} & \dot{x} & \dot{y} & \dot{z} & \dot{\text{roll}} & \dot{\text{pitch}} & \dot{\text{yaw}} & \ddot{x} & \ddot{y} & \ddot{z} \end{bmatrix}^T$$

### 2. Sensor Fusion Configuration
- **Wheel Odometry (`odom0: /bumperbot_controller/odom_noisy`)**:
  - `odom0_config`: Tracks velocities $[\dot{x}, \dot{y}, \dot{z}]$ and angular velocity $\dot{\text{yaw}}$:
    ```yaml
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    odom0_differential: true
    ```
- **IMU Sensor (`imu0: /imu_ekf`)**:
  - `imu0_config`: Tracks angular velocity $\dot{\text{yaw}}$ and linear acceleration $\ddot{x}$:
    ```yaml
    imu0_config: [false, false, false,
                  false, false, false,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
    imu0_differential: false
    ```
- **Frame Configuration**:
  - `map_frame: map`
  - `odom_frame: odom`
  - `base_link_frame: base_footprint_ekf`
  - `world_frame: odom` (Outputs continuous fused odometry in `odom`)

---

## D. Global Monte Carlo Localization (`nav2_amcl`)

Configured in `src/bumperbot_localization/config/amcl.yaml` and launched via `global_localization.launch.py`:

### 1. Adaptive Particle Filtering (KLD-Sampling)
Unlike fixed-size particle filters, `nav2_amcl` adapts the number of particles based on the Kullback-Leibler divergence (KLD) between the true belief and the particle approximation:
- `min_particles: 500`
- `max_particles: 2000`
- `pf_err: 0.05`, `pf_z: 0.99`
When the robot is well-localized, particle count drops towards 500 to save CPU. When uncertainty is high, particle count expands up to 2000.

### 2. Likelihood Field Sensor Model
For each laser beam $z_t^k$, AMCL computes the Euclidean distance $d$ from the beam endpoint to the nearest obstacle on the pre-built map:
$$p_{\text{hit}}(z_t^k | x_t, m) = \frac{1}{\sqrt{2\pi \sigma_{\text{hit}}^2}} \exp\left(-\frac{d^2}{2 \sigma_{\text{hit}}^2}\right)$$
Total measurement likelihood mixes four components:
$$p(z_t^k | x_t, m) = z_{\text{hit}} p_{\text{hit}} + z_{\text{short}} p_{\text{short}} + z_{\text{max}} p_{\text{max}} + z_{\text{rand}} p_{\text{rand}}$$
Configured parameters: $z_{\text{hit}} = 0.5$, $z_{\text{rand}} = 0.5$, $\sigma_{\text{hit}} = 0.2\text{ m}$.

### 3. Coordinate Frame Correction (`map -> odom`)
AMCL does **not** modify `/bumperbot_controller/odom`. Instead, it publishes the dynamic transform between `map` and `odom`:
$$\mathbf{T}_{\text{map}}^{\text{base}} = \mathbf{T}_{\text{map}}^{\text{odom}} \cdot \mathbf{T}_{\text{odom}}^{\text{base}}$$
Therefore:
$$\mathbf{T}_{\text{map}}^{\text{odom}} = \mathbf{T}_{\text{map}}^{\text{base}} \cdot \left(\mathbf{T}_{\text{odom}}^{\text{base}}\right)^{-1}$$
This isolates high-rate continuous motion control (which uses `odom`) from discrete global jumps caused by AMCL particle re-weighting.

---

## E. Practical Verification & Testing Commands

### 1. Testing Local Localization (EKF)
```bash
# Terminal 1: Launch simulation
ros2 launch bumperbot_bringup simulated_robot.launch.py

# Terminal 2: Launch EKF local localization
ros2 launch bumperbot_localization local_localization.launch.py

# Terminal 3: Echo fused EKF odometry
ros2 topic echo /odometry/filtered --field pose.pose.position
```

### 2. Testing Global Localization (AMCL + Pre-built Map)
```bash
# Terminal 1: Launch simulation with AMCL global localization (default use_slam:=False)
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house use_slam:=False

# Terminal 2: Set 2D Pose Estimate in RViz2 or via CLI topic
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0, 0, 0, 0, 0,
                 0, 0.25, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0.06]
  }
}"

# Terminal 3: Teleoperate to observe particle convergence
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/key_vel
```

