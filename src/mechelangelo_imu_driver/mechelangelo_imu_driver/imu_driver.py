#!/usr/bin/env python3

import math
import os
import shutil
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu, Temperature, MagneticField, FluidPressure as Pressure, RelativeHumidity as Humidity
from std_msgs.msg import Header

from sense_hat import SenseHat


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list:
    """Convert Euler angles (RPY, radians) to [x, y, z, w] quaternion.

    Equivalent to tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    using the default sxyz (intrinsic XYZ) convention.
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    ]


def _ensure_rtimu_config() -> None:
    """Copy the bundled RTIMULib.ini to ~/.config/sense_hat/ if not already there.

    sense_hat.py only searches ~/.config/sense_hat/ and /etc/ for the calibration
    file. This function seeds the home location from the package-shipped file so
    compass, accel, and gyro calibration values are not lost on first boot.
    """
    home_path = os.path.join(os.path.expanduser('~'), '.config', 'sense_hat')
    os.makedirs(home_path, exist_ok=True)

    dest = os.path.join(home_path, 'RTIMULib.ini')
    if os.path.isfile(dest):
        return

    try:
        pkg_share = get_package_share_directory('mechelangelo_imu_driver')
        src = os.path.join(pkg_share, 'config', 'RTIMULib.ini')
        if os.path.isfile(src):
            shutil.copyfile(src, dest)
    except Exception:
        pass


class SenseHatImuNode(Node):
    def __init__(self):
        super().__init__('sensehat_imu')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('en_imu', True),
                ('en_mag', True),
                ('en_pressure', True),
                ('en_humidity', True),
                ('en_temp', True),
                ('frame_id', 'sensehat_frame'),
                ('timer_period', 0.02),
            ]
        )

        _ensure_rtimu_config()

        self._sensehat = SenseHat()
        self._sensehat.set_rotation(180)

        en_imu = self.get_parameter('en_imu').value
        en_mag = self.get_parameter('en_mag').value
        en_pressure = self.get_parameter('en_pressure').value
        en_humidity = self.get_parameter('en_humidity').value
        en_temp = self.get_parameter('en_temp').value

        self._sensehat.set_imu_config(en_mag and en_imu, en_imu, en_imu)

        qos = qos_profile_sensor_data

        self._imu_pub = self.create_publisher(Imu, 'imu', qos) if en_imu else None
        self._mag_pub = self.create_publisher(MagneticField, 'mag', qos) if en_mag else None
        self._pressure_pub = self.create_publisher(Pressure, 'pressure', qos) if en_pressure else None
        self._humidity_pub = self.create_publisher(Humidity, 'humidity', qos) if en_humidity else None
        self._temp_p_pub = self.create_publisher(Temperature, 'temp_p', qos) if (en_temp and en_pressure) else None
        self._temp_h_pub = self.create_publisher(Temperature, 'temp_h', qos) if (en_temp and en_humidity) else None

        if en_temp and not en_pressure and not en_humidity:
            self.get_logger().warn('Temperature enabled but pressure and humidity both disabled — nothing to publish.')

        timer_period = self.get_parameter('timer_period').value
        self._timer: Optional[Timer] = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info('SenseHat IMU driver started')

    def _header(self):
        return Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.get_parameter('frame_id').value,
        )

    def _timer_callback(self):
        try:
            if self._imu_pub:
                gyro = self._sensehat.get_gyroscope_raw()   # rad/s
                accel = self._sensehat.get_accelerometer_raw()  # g

                msg = Imu()
                msg.header = self._header()

                # Negate Y to convert left-hand → right-hand coordinate convention
                msg.angular_velocity = Vector3(
                    x=gyro['x'], y=-gyro['y'], z=gyro['z'])
                msg.angular_velocity_covariance = [
                    0.01, 0.0, 0.0,
                    0.0, 0.01, 0.0,
                    0.0, 0.0, 0.01,
                ]
                msg.linear_acceleration = Vector3(
                    x=accel['x'] * 9.81, y=-accel['y'] * 9.81, z=accel['z'] * 9.81)
                msg.linear_acceleration_covariance = [
                    0.01, 0.0, 0.0,
                    0.0, 0.01, 0.0,
                    0.0, 0.0, 0.01,
                ]

                if self._mag_pub:
                    orientation = self._sensehat.get_orientation_radians()
                    quat = _euler_to_quaternion(
                        orientation['roll'], -orientation['pitch'], orientation['yaw'])
                    msg.orientation = Quaternion(
                        x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                    msg.orientation_covariance = [
                        0.01, 0.0, 0.0,
                        0.0, 0.01, 0.0,
                        0.0, 0.0, 0.01,
                    ]

                self._imu_pub.publish(msg)

            if self._mag_pub:
                mag = self._sensehat.get_compass_raw()  # µT
                msg = MagneticField()
                msg.header = self._header()
                msg.magnetic_field = Vector3(
                    x=mag['x'] / 1e6, y=-mag['y'] / 1e6, z=mag['z'] / 1e6)  # µT → T
                self._mag_pub.publish(msg)

            if self._pressure_pub:
                msg = Pressure()
                msg.header = self._header()
                msg.fluid_pressure = self._sensehat.get_pressure() * 100.0  # mBar → Pa
                self._pressure_pub.publish(msg)

            if self._humidity_pub:
                msg = Humidity()
                msg.header = self._header()
                msg.relative_humidity = self._sensehat.get_humidity() / 100.0
                self._humidity_pub.publish(msg)

            if self._temp_p_pub:
                msg = Temperature()
                msg.header = self._header()
                msg.temperature = float(self._sensehat.get_temperature_from_pressure())
                self._temp_p_pub.publish(msg)

            if self._temp_h_pub:
                msg = Temperature()
                msg.header = self._header()
                msg.temperature = float(self._sensehat.get_temperature_from_humidity())
                self._temp_h_pub.publish(msg)

        except OSError as exc:
            self.get_logger().error(f'Sensor read failed: {exc}')

    def shutdown(self):
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        self.get_logger().info('SenseHat IMU driver stopped')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SenseHatImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
