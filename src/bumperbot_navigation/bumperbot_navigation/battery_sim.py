#!/usr/bin/env python3
"""
SIMULATION ONLY: battery model node.

Publishes /battery_state (sensor_msgs/BatteryState). The charge drains while the robot is idle and
faster while it moves (twist of the odometry topic) and rises while /dock/status reports docked.
The model itself lives in battery_logic.py (BatteryModel). On the real robot the battery driver
publishes /battery_state instead; this node is started only by the simulation launch files.
"""
import math
import sys

import rclpy
from bumperbot_msgs.msg import DockStatus
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

from bumperbot_navigation.battery_logic import (
    STATUS_CHARGING, STATUS_DISCHARGING, STATUS_FULL, BatteryModel, BatteryParams)


class BatterySim(Node):
    def __init__(self):
        super().__init__("battery_sim")

        self.declare_parameter("initial_percentage", 1.0)
        self.declare_parameter("idle_drain_rate", 0.0005)
        self.declare_parameter("linear_drain_rate", 0.004)
        self.declare_parameter("angular_drain_rate", 0.001)
        self.declare_parameter("charge_rate", 0.01)
        self.declare_parameter("empty_voltage", 10.5)
        self.declare_parameter("full_voltage", 12.6)
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("percentage_override", -1.0)
        self.declare_parameter("odom_topic", "bumperbot_controller/odom")
        self.declare_parameter("dock_status_topic", "dock/status")
        self.declare_parameter("battery_topic", "battery_state")

        get = self.get_parameter
        params = BatteryParams(
            idle_drain_rate=float(get("idle_drain_rate").value),
            linear_drain_rate=float(get("linear_drain_rate").value),
            angular_drain_rate=float(get("angular_drain_rate").value),
            charge_rate=float(get("charge_rate").value),
            empty_voltage=float(get("empty_voltage").value),
            full_voltage=float(get("full_voltage").value))
        publish_rate = float(get("publish_rate").value)
        if not (math.isfinite(publish_rate) and publish_rate > 0.0):
            raise ValueError("parameter 'publish_rate' must be a positive number")
        for name in ("idle_drain_rate", "linear_drain_rate", "angular_drain_rate", "charge_rate"):
            value = float(get(name).value)
            if not (math.isfinite(value) and value >= 0.0):
                raise ValueError("parameter '{}' must not be negative".format(name))
        if not params.empty_voltage < params.full_voltage:
            raise ValueError("empty_voltage must be below full_voltage")

        self.model = BatteryModel(params, float(get("initial_percentage").value))
        self.linear = 0.0
        self.angular = 0.0
        self.docked = False
        self.last_time = self.get_clock().now()

        self.pub = self.create_publisher(BatteryState, get("battery_topic").value, 10)
        self.create_subscription(Odometry, get("odom_topic").value, self.odom_callback, 10)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            DockStatus, get("dock_status_topic").value, self.dock_status_callback, status_qos)
        self.add_on_set_parameters_callback(self.on_parameters)

        # Node-clock timer: a paused simulation (clock not advancing) gives no change
        self.timer = self.create_timer(1.0 / publish_rate, self.update)
        self.get_logger().info("Battery sim ready (SIMULATION ONLY), charge {:.2f}".format(
            self.model.percentage))

    def odom_callback(self, msg):
        self.linear = float(msg.twist.twist.linear.x)
        self.angular = float(msg.twist.twist.angular.z)

    def dock_status_callback(self, msg):
        self.docked = bool(msg.docked)

    def on_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name != "percentage_override":
                continue
            value = float(parameter.value)
            if not math.isfinite(value) or value > 1.0:
                return SetParametersResult(
                    successful=False, reason="percentage_override must be -1 or in [0, 1]")
            if value >= 0.0:
                self.model.set_percentage(value)
                self.get_logger().info("Charge set to {:.2f}".format(value))
        return SetParametersResult(successful=True)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        self.model.step(dt, self.linear, self.angular, self.docked)
        self.publish()

    def publish(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(self.model.voltage())
        msg.temperature = math.nan
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = float(self.model.percentage)
        status = self.model.power_supply_status(self.docked)
        msg.power_supply_status = {
            STATUS_CHARGING: BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            STATUS_FULL: BatteryState.POWER_SUPPLY_STATUS_FULL,
            STATUS_DISCHARGING: BatteryState.POWER_SUPPLY_STATUS_DISCHARGING}[status]
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BatterySim()
    except Exception as exc:  # noqa: BLE001 - any constructor failure is fatal
        rclpy.logging.get_logger("battery_sim").fatal(str(exc))
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
