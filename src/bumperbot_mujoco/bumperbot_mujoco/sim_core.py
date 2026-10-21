"""MuJoCo simulation of BumperBot without any ROS dependency.

The ROS node in mujoco_sim_node.py wraps this class; keeping the physics separate allows it to be
tested and run headless without a ROS installation.
"""
import math

import mujoco
import numpy as np

WHEEL_JOINTS = ("wheel_left_joint", "wheel_right_joint")
# Only static world collision geoms (group 4) are visible to the lidar
LIDAR_GEOM_GROUPS = np.array([0, 0, 0, 0, 1, 0], dtype=np.uint8)


class BumperbotSim:
    def __init__(self, world_file, x=0.0, y=0.0, z=0.001, yaw=0.0, seed=None):
        self.model = mujoco.MjModel.from_xml_path(world_file)
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)

        self._wheel_qpos = []
        self._wheel_dof = []
        self._wheel_ctrl = []
        for joint in WHEEL_JOINTS:
            joint_id = self._id(mujoco.mjtObj.mjOBJ_JOINT, joint)
            self._wheel_qpos.append(self.model.jnt_qposadr[joint_id])
            self._wheel_dof.append(self.model.jnt_dofadr[joint_id])
            self._wheel_ctrl.append(self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, joint + "_motor"))
        self._laser_site = self._id(mujoco.mjtObj.mjOBJ_SITE, "laser_site")
        self._sensors = {name: self._id(mujoco.mjtObj.mjOBJ_SENSOR, name)
                         for name in ("imu_gyro", "imu_accel", "imu_quat")}

        self._set_base_pose(x, y, z, yaw)
        mujoco.mj_forward(self.model, self.data)

    def _id(self, obj_type, name):
        index = mujoco.mj_name2id(self.model, obj_type, name)
        if index < 0:
            raise KeyError("%s not found in the MuJoCo model" % name)
        return index

    def _set_base_pose(self, x, y, z, yaw):
        joint_id = self._id(mujoco.mjtObj.mjOBJ_JOINT, "base_freejoint")
        adr = self.model.jnt_qposadr[joint_id]
        self.data.qpos[adr:adr + 3] = (x, y, z)
        self.data.qpos[adr + 3:adr + 7] = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))

    @property
    def time(self):
        return self.data.time

    @property
    def timestep(self):
        return self.model.opt.timestep

    def set_wheel_velocities(self, left, right):
        """Target wheel velocities [rad/s] for wheel_left_joint and wheel_right_joint."""
        self.data.ctrl[self._wheel_ctrl[0]] = left
        self.data.ctrl[self._wheel_ctrl[1]] = right

    def step(self, duration):
        """Advance the simulation by (at least) the given duration [s]."""
        end = self.data.time + duration - 1e-9
        while self.data.time < end:
            mujoco.mj_step(self.model, self.data)

    def wheel_positions(self):
        return [float(self.data.qpos[adr]) for adr in self._wheel_qpos]

    def wheel_velocities(self):
        return [float(self.data.qvel[adr]) for adr in self._wheel_dof]

    def _sensor(self, name, noisy):
        sensor_id = self._sensors[name]
        adr = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        values = np.array(self.data.sensordata[adr:adr + dim])
        noise = self.model.sensor_noise[sensor_id]
        if noisy and noise > 0.0:
            values += self.rng.normal(0.0, noise, dim)
        return values

    def imu(self, noisy=True):
        """Return (orientation wxyz, angular velocity [rad/s], linear acceleration [m/s^2]) in the IMU frame."""
        return (self._sensor("imu_quat", False),
                self._sensor("imu_gyro", noisy),
                self._sensor("imu_accel", noisy))

    def scan(self, angle_min, angle_increment, samples, range_min, range_max, noise_std=0.0):
        """Cast a planar lidar scan from laser_site; misses are reported as inf (as Gazebo does)."""
        angles = angle_min + angle_increment * np.arange(samples)
        local = np.stack([np.cos(angles), np.sin(angles), np.zeros(samples)], axis=1)
        rotation = self.data.site_xmat[self._laser_site].reshape(3, 3)
        directions = (local @ rotation.T).reshape(-1)
        origin = np.array(self.data.site_xpos[self._laser_site])

        distances = np.zeros(samples)
        geom_ids = np.zeros(samples, dtype=np.int32)
        normals = np.zeros(samples * 3)
        mujoco.mj_multiRay(self.model, self.data, origin, directions, LIDAR_GEOM_GROUPS, 1, -1,
                           geom_ids, distances, normals, samples, range_max)

        ranges = np.where(distances < 0.0, np.inf, distances)
        if noise_std > 0.0:
            ranges = ranges + self.rng.normal(0.0, noise_std, samples)
        ranges[(ranges < range_min) | (ranges > range_max)] = np.inf
        return ranges

    def base_pose(self):
        """Return (x, y, yaw) of base_footprint in the world frame, useful for tests."""
        body = self._id(mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
        pos = self.data.xpos[body]
        w, x, y, z = self.data.xquat[body]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), yaw
