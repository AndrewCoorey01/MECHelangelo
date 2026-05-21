#!/usr/bin/env python3

import math

import PyLidar3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class YDLidarX4Node(Node):
    def __init__(self):
        super().__init__('ydlidar_x4_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'base_scan')
        self.declare_parameter('scan_topic', 'scan')

        self.declare_parameter('range_min_m', 0.12)
        self.declare_parameter('range_max_m', 8.0)
        self.declare_parameter('publish_rate_hz', 7.0)

        # Use this if the physical front of the LiDAR is not aligned with robot front.
        # Example:
        #   If PyLidar3 angle 90 degrees points forward on your robot,
        #   set angle_offset_deg to -90.
        self.declare_parameter('angle_offset_deg', 0.0)

        # Use this if left/right appears mirrored.
        self.declare_parameter('invert_scan', False)

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.range_min_m = float(self.get_parameter('range_min_m').value)
        self.range_max_m = float(self.get_parameter('range_max_m').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)
        self.invert_scan = bool(self.get_parameter('invert_scan').value)

        self.num_readings = 360

        self.scan_pub = self.create_publisher(
            LaserScan,
            self.scan_topic,
            10,
        )

        self.lidar = PyLidar3.YdLidarX4(self.port)

        self.get_logger().info(f'Connecting to YDLIDAR X4 on {self.port}')

        if not self.lidar.Connect():
            self.get_logger().error('Failed to connect to YDLIDAR X4')
            raise RuntimeError('Failed to connect to YDLIDAR X4')

        self.get_logger().info(f'Connected: {self.lidar.GetDeviceInfo()}')
        self.get_logger().info(f'Health OK: {self.lidar.GetHealthStatus()}')

        self.scan_generator = self.lidar.StartScanning()

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Publishing LaserScan on /{self.scan_topic}')
        self.get_logger().info(f'Laser frame_id: {self.frame_id}')
        self.get_logger().info('LaserScan angle range: -pi to +pi, with 0 rad at robot front')

    def normalise_angle_deg_0_to_360(self, angle_deg):
        """
        Converts any angle into the range [0, 360).
        """
        return angle_deg % 360.0

    def timer_callback(self):
        try:
            raw_scan = next(self.scan_generator)
        except Exception as exc:
            self.get_logger().warn(f'Failed to read LiDAR scan: {exc}')
            return

        msg = LaserScan()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Important:
        # This is the corrected ROS-friendly angle convention.
        #
        # Index 0     = -180 degrees
        # Index 180   = 0 degrees, straight ahead
        # Index 359   = about +179 degrees
        #
        # This allows behaviour code to check:
        #   -15 degrees to +15 degrees
        # as the front safety cone.
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / self.num_readings

        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.publish_rate_hz

        msg.range_min = self.range_min_m
        msg.range_max = self.range_max_m

        ranges = [float('inf')] * self.num_readings

        for angle_deg, distance_mm in raw_scan.items():
            try:
                raw_angle_deg = float(angle_deg)
                distance_m = float(distance_mm) / 1000.0
            except (TypeError, ValueError):
                continue

            if distance_m < self.range_min_m or distance_m > self.range_max_m:
                continue

            # Apply physical mounting offset.
            corrected_angle_deg = raw_angle_deg + self.angle_offset_deg

            # Optionally mirror scan if left/right are flipped.
            if self.invert_scan:
                corrected_angle_deg = -corrected_angle_deg

            # Convert into [-180, +180).
            corrected_angle_deg = self.normalise_angle_deg_0_to_360(corrected_angle_deg)

            if corrected_angle_deg >= 180.0:
                corrected_angle_deg -= 360.0

            corrected_angle_rad = math.radians(corrected_angle_deg)

            # Convert angle into LaserScan index.
            index = int(round((corrected_angle_rad - msg.angle_min) / msg.angle_increment))

            if 0 <= index < self.num_readings:
                ranges[index] = distance_m

        msg.ranges = ranges
        msg.intensities = []

        self.scan_pub.publish(msg)

    def shutdown_lidar(self):
        self.get_logger().info('Stopping YDLIDAR X4')

        try:
            self.lidar.StopScanning()
        except Exception:
            pass

        try:
            self.lidar.Disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = YDLidarX4Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.shutdown_lidar()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()