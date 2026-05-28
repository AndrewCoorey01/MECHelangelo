#!/usr/bin/env python3
"""
MECHelangelo ThunderBorg base driver.

This node replaces the GPIO PWM/direction base driver when the robot is using
PiBorg ThunderBorg motor control hardware.

It subscribes to:

    /cmd_vel
        geometry_msgs/msg/Twist

It drives:

    ThunderBorg Motor 1 and Motor 2

Optional future extension:

    Wheel encoders can be added later for closed-loop odometry.

The purpose of this node is to make the real robot behave like the simulated
robot. Anything that publishes /cmd_vel, such as keyboard teleop, Nav2, or the
MECHelangelo behaviour node, can command the robot without directly touching the
ThunderBorg API.
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import ThunderBorg


def constrain(value, low, high):
    return max(low, min(high, value))


class MechelangeloThunderBorgDriver(Node):
    def __init__(self):
        super().__init__('mechelangelo_base_driver')

        self.load_parameters()

        self.target_linear_mps = 0.0
        self.target_angular_radps = 0.0
        self.last_cmd_time = time.time()
        self.last_status_time = 0.0

        self.tb = ThunderBorg.ThunderBorg()
        self.tb.Init()

        if not self.tb.foundChip:
            boards = ThunderBorg.ScanForThunderBorg()
            self.get_logger().error('No ThunderBorg found.')
            self.get_logger().error(f'ThunderBorg boards found: {boards}')
            raise RuntimeError('ThunderBorg not found')

        self.get_logger().info('ThunderBorg connected.')

        try:
            battery_voltage = self.tb.GetBatteryReading()
            self.get_logger().info(f'ThunderBorg battery voltage: {battery_voltage:.2f} V')
        except Exception as exc:
            self.get_logger().warn(f'Could not read ThunderBorg battery voltage: {exc}')

        self.configure_thunderborg_failsafe()

        self.tb.MotorsOff()

        if self.show_battery_led:
            self.tb.SetLedShowBattery(True)

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.get_logger().info('MECHelangelo ThunderBorg base driver started.')
        self.get_logger().info(f'Subscribing to: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'Motor mapping: Motor1 is {"right" if self.motor1_is_right else "left"}, '
            f'Motor2 is {"left" if self.motor1_is_right else "right"}'
        )
        self.get_logger().info(
            f'Limits: max_linear={self.max_linear_vel_mps:.3f} m/s, '
            f'max_angular={self.max_angular_vel_radps:.3f} rad/s, '
            f'max_power={self.max_power:.2f}'
        )

    def load_parameters(self):
        """
        Declare and load ROS parameters.

        These can be overridden from a YAML file or launch file.
        """

        # ROS interface
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')

        # Robot geometry
        self.declare_parameter('wheel_separation_m', 0.50)

        # Velocity limits accepted from /cmd_vel
        self.declare_parameter('max_linear_vel_mps', 0.15)
        self.declare_parameter('max_angular_vel_radps', 0.60)

        # ThunderBorg output scaling
        self.declare_parameter('max_power', 0.75)
        self.declare_parameter('deadband', 0.03)

        # Motor mapping and direction correction
        # PiBorg's tbJoystick.py commonly treats:
        #   Motor 1 = right wheel
        #   Motor 2 = left wheel
        self.declare_parameter('motor1_is_right', True)
        self.declare_parameter('left_motor_invert', False)
        self.declare_parameter('right_motor_invert', False)

        # Control/safety settings
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('status_period', 0.5)
        self.declare_parameter('enable_comms_failsafe', True)
        self.declare_parameter('show_battery_led', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.wheel_separation_m = float(self.get_parameter('wheel_separation_m').value)
        self.max_linear_vel_mps = float(self.get_parameter('max_linear_vel_mps').value)
        self.max_angular_vel_radps = float(self.get_parameter('max_angular_vel_radps').value)
        self.max_power = float(self.get_parameter('max_power').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.motor1_is_right = bool(self.get_parameter('motor1_is_right').value)
        self.left_motor_invert = bool(self.get_parameter('left_motor_invert').value)
        self.right_motor_invert = bool(self.get_parameter('right_motor_invert').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.status_period = float(self.get_parameter('status_period').value)
        self.enable_comms_failsafe = bool(self.get_parameter('enable_comms_failsafe').value)
        self.show_battery_led = bool(self.get_parameter('show_battery_led').value)

        self.max_power = constrain(self.max_power, 0.0, 1.0)

    def configure_thunderborg_failsafe(self):
        """
        Enable the ThunderBorg communication failsafe if requested.

        ThunderBorg can turn motors off if motor commands stop arriving. This is
        separate from this node's /cmd_vel timeout and gives another safety layer.
        """

        if not self.enable_comms_failsafe:
            self.get_logger().warn('ThunderBorg comms failsafe disabled by parameter.')
            return

        failsafe_ok = False

        for _ in range(5):
            try:
                self.tb.SetCommsFailsafe(True)
                time.sleep(0.02)
                if self.tb.GetCommsFailsafe():
                    failsafe_ok = True
                    break
            except Exception:
                pass

        if failsafe_ok:
            self.get_logger().info('ThunderBorg comms failsafe enabled.')
        else:
            self.get_logger().warn('Could not confirm ThunderBorg comms failsafe.')

    def cmd_vel_callback(self, msg):
        self.target_linear_mps = constrain(
            msg.linear.x,
            -self.max_linear_vel_mps,
            self.max_linear_vel_mps,
        )

        self.target_angular_radps = constrain(
            msg.angular.z,
            -self.max_angular_vel_radps,
            self.max_angular_vel_radps,
        )

        self.last_cmd_time = time.time()

    def apply_deadband(self, value):
        if abs(value) < self.deadband:
            return 0.0
        return value

    def twist_to_wheel_commands(self, linear_mps, angular_radps):
        """
        Convert robot linear/angular velocity into left/right wheel commands.

        Differential drive kinematics:

            left_velocity  = v - omega * wheel_separation / 2
            right_velocity = v + omega * wheel_separation / 2

        These wheel velocities are then normalised into motor commands in the
        range -1.0 to +1.0 before being scaled by max_power.
        """

        left_velocity = linear_mps - angular_radps * (self.wheel_separation_m / 2.0)
        right_velocity = linear_mps + angular_radps * (self.wheel_separation_m / 2.0)

        # Normalise relative to the configured maximums.
        # This keeps the output proportional while preventing either wheel from
        # exceeding the allowed command range.
        max_wheel_velocity = self.max_linear_vel_mps + (
            self.max_angular_vel_radps * self.wheel_separation_m / 2.0
        )

        if max_wheel_velocity <= 0.0:
            return 0.0, 0.0

        left_command = left_velocity / max_wheel_velocity
        right_command = right_velocity / max_wheel_velocity

        scale = max(1.0, abs(left_command), abs(right_command))
        left_command /= scale
        right_command /= scale

        return left_command, right_command

    def send_motor_command(self, left_command, right_command):
        """
        Send signed left/right motor commands to the ThunderBorg.

        Command convention:
            + = forward
            - = reverse
             0 = stopped
        """

        left_command = self.apply_deadband(left_command)
        right_command = self.apply_deadband(right_command)

        if self.left_motor_invert:
            left_command = -left_command
        if self.right_motor_invert:
            right_command = -right_command

        left_output = constrain(left_command, -1.0, 1.0) * self.max_power
        right_output = constrain(right_command, -1.0, 1.0) * self.max_power

        if self.motor1_is_right:
            self.tb.SetMotor1(right_output)
            self.tb.SetMotor2(left_output)
        else:
            self.tb.SetMotor1(left_output)
            self.tb.SetMotor2(right_output)

        return left_output, right_output

    def control_loop(self):
        now = time.time()

        if now - self.last_cmd_time > self.cmd_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.target_linear_mps
            angular = self.target_angular_radps

        left_command, right_command = self.twist_to_wheel_commands(linear, angular)
        left_output, right_output = self.send_motor_command(left_command, right_command)

        if now - self.last_status_time > self.status_period:
            self.last_status_time = now
            self.print_status(linear, angular, left_output, right_output)

    def print_status(self, linear, angular, left_output, right_output):
        try:
            fault1 = self.tb.GetDriveFault1()
            fault2 = self.tb.GetDriveFault2()
        except Exception:
            fault1 = 'unknown'
            fault2 = 'unknown'

        self.get_logger().info(
            f'cmd v={linear:+.3f} m/s, w={angular:+.3f} rad/s | '
            f'L out={left_output:+.2f}, R out={right_output:+.2f} | '
            f'Fault1={fault1}, Fault2={fault2}'
        )

    def stop_all(self):
        try:
            self.tb.MotorsOff()
        except AttributeError:
            pass
        except Exception as exc:
            self.get_logger().warn(f'Could not stop ThunderBorg motors cleanly: {exc}')

    def close_hardware(self):
        self.stop_all()

        try:
            self.tb.SetCommsFailsafe(False)
        except Exception:
            pass

        try:
            self.tb.SetLedShowBattery(False)
            self.tb.SetLeds(0.2, 0.0, 0.0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = MechelangeloThunderBorgDriver()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if rclpy.ok():
            print(f'MECHelangelo ThunderBorg driver error: {exc}')
        raise

    finally:
        if node is not None:
            node.get_logger().info('Stopping ThunderBorg motors and closing driver.')
            node.close_hardware()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
