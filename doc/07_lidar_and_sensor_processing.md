# 07 - LiDAR & Sensor Processing

## A. Objective

This document covers sensor data structures, polar-to-Cartesian transformations, IMU reference frame conversions, and the autonomous reactive safety supervisor implemented in `bumperbot_utils`.

---

## B. 2D LiDAR Principles & `LaserScan` Processing

### 1. `sensor_msgs/msg/LaserScan` Anatomy
Planar range data is communicated via the standard ROS 2 `LaserScan` message:
- `header.stamp`: The exact time the scan was initiated.
- `header.frame_id`: Coordinate frame of the laser emitter (`laser_link`).
- `angle_min` / `angle_max`: Start and end angles of the scan arc ($[0, 2\pi]\text{ rad}$).
- `angle_increment`: Angular resolution between adjacent beams ($\Delta \theta = \frac{2\pi}{360} \approx 0.01745\text{ rad} = 1^\circ$).
- `time_increment`: Time between successive beam measurements ($\approx 0.0\text{ s}$ for simultaneous GPU simulation).
- `scan_time`: Period between complete rotations ($0.05\text{ s}$ at 20 Hz).
- `range_min` / `range_max`: Hardware cutoff limits ($0.12\text{ m}$ to $12.0\text{ m}$).
- `ranges[]`: Array of float range measurements of length $N = 360$.

### 2. Polar to Cartesian Transformation with Sensor Offsets
For beam index $i \in [0, N-1]$ with distance $r = \text{ranges}[i]$:
1. Polar beam angle in the sensor frame:
   $$\theta_L = \text{angle\_min} + i \cdot \text{angle\_increment}$$
2. Point in `laser_link` coordinates:
   $$x_L = r \cos(\theta_L), \quad y_L = r \sin(\theta_L)$$
3. Transformation to robot base frame (`base_link` or `odom`):
   Because the sensor is mounted with a $180^\circ$ ($3.14\text{ rad}$) yaw rotation on BumperBot, the orientation transforms through:
   $$x_O = r \cos(\theta_L + \text{yaw}_{\text{sensor}}) + t_x$$
   $$y_O = r \sin(\theta_L + \text{yaw}_{\text{sensor}}) + t_y$$

> [!WARNING]
> **Invalid Range Filtering**: Raw LiDAR data routinely contains non-finite values (`std::isinf` or `std::isnan`) when beams do not return (free space or sky) or when specular reflections fail. All processing algorithms must explicitly validate:
> ```cpp
> if (std::isinf(r) || std::isnan(r) || r < scan.range_min || r > scan.range_max) {
>     continue; // Skip invalid measurements
> }
> ```

---

## C. Autonomous Reactive Safety Supervisor (`safety_stop`)

Implemented in `src/bumperbot_utils/src/safety_stop.cpp` and `bumperbot_utils/safety_stop.py`, this node acts as an autonomous virtual bumper protecting the robot against collisions.

```
       +---------------------------------------------+
       |               FREE SPACE                    |
       |  (Range > 0.60m) - Normal Max Speed         |
       |                                             |
       |       +-----------------------------+       |
       |       |       WARNING ZONE          |       |
       |       |  (0.20m < Range <= 0.60m)   |       |
       |       |  Triggers /joy_turbo_decrease       |
       |       |                             |       |
       |       |       +-------------+       |       |
       |       |       | DANGER ZONE |       |       |
       |       |       | (r <= 0.20m)|       |       |
       |       |       | Hard Brake  |       |       |
       |       |       | Lock 255    |       |       |
       |       |       +-------------+       |       |
       +-------+-----------------------------+-------+
```

### 1. Finite State Machine Logic
```
          [FREE] (Range > 0.60m)
            |               ^
            | r <= 0.60m    | r > 0.60m (Calls /joy_turbo_increase)
            v               |
        [WARNING] ----------+
            |
            | r <= 0.20m
            v
         [DANGER] (Publishes /safety_stop = True -> twist_mux Lock 255)
```

- **`State::FREE`**: All obstacles $> 0.60\text{ m}$. Calls action server `/joy_turbo_increase` to restore velocity scale. Publishes `safety_stop: False`.
- **`State::WARNING`**: Any valid beam $\le 0.60\text{ m}$ but $> 0.20\text{ m}$. Asynchronously dispatches a goal to `/joy_turbo_decrease`, cutting manual teleop speed. Disks on `/zones` highlight yellow.
- **`State::DANGER`**: Any valid beam $\le 0.20\text{ m}$. Immediately publishes `True` on `/safety_stop`. `twist_mux` activates lock 255 and stops the robot. Disks on `/zones` highlight red.

### 2. RViz2 Visual Zones
The node generates two concentric visual disks using `visualization_msgs/msg/MarkerArray` on `/zones`:
- Marker ID 0: Yellow translucent cylinder representing the warning threshold ($2 \times \text{warning\_distance} = 1.2\text{ m}$ diameter).
- Marker ID 1: Red translucent cylinder representing the danger threshold ($2 \times \text{danger\_distance} = 0.4\text{ m}$ diameter).

---

## D. IMU Processing & Frame Sign Conversion (`imu_republisher`)

Implemented in `src/bumperbot_localization/src/imu_republisher.cpp` and `imu_republisher.py`:
- **The Problem**: In Ignition Gazebo Fortress, the IMU system plugin aligns with world orientation conventions where angular velocity around $z$ and linear accelerations may have opposite sign conventions compared to standard ROS ENU (East-North-Up) frames.
- **The Solution**: The `imu_republisher_node` subscribes to `/imu/out`, applies axis sign transformations:
  ```cpp
  imu_ekf.angular_velocity.z = -msg.angular_velocity.z;
  imu_ekf.linear_acceleration.y = -msg.linear_acceleration.y;
  imu_ekf.linear_acceleration.z = -msg.linear_acceleration.z;
  ```
  and publishes the corrected output to `/imu_ekf` with `header.frame_id = "imu_link_ekf"`.

---

## E. Physical MPU6050 Linux I2C Driver (`mpu6050_driver`)

For real hardware, `src/bumperbot_firmware/bumperbot_firmware/mpu6050_driver.py` directly interfaces with the MPU6050 sensor via Linux I2C userspace (`smbus` on `/dev/i2c-1`):
1. **Power Configuration**: Writes `0x00` to `PWR_MGMT_1` (register `0x6B`) to wake the sensor from sleep.
2. **Register Polling**: At 100 Hz, reads 16-bit signed registers:
   - Accel $X, Y, Z$: Registers `0x3B` through `0x40`
   - Gyro $X, Y, Z$: Registers `0x43` through `0x48`
3. **Unit Conversion**:
   $$\mathbf{a} = \frac{\text{raw\_accel}}{1670.13}\text{ m/s}^2, \quad \boldsymbol{\omega} = \frac{\text{raw\_gyro}}{7509.55}\text{ rad/s}$$
4. **Publishing**: Outputs to `/imu/out` with `qos_profile_sensor_data` (Best-Effort, Volatile).

---

## F. Practical Inspection & Testing Commands

```bash
# 1. Inspect live LiDAR scan topic metadata and publish frequency
ros2 topic hz /scan
ros2 topic info /scan

# 2. Echo a single LiDAR scan to inspect range values
ros2 topic echo --once /scan --field ranges

# 3. View live safety supervisor state changes
ros2 topic echo /safety_stop

# 4. Inspect IMU publication rates and covariance matrices
ros2 topic hz /imu/out
ros2 topic echo --once /imu/out --field linear_acceleration
```

