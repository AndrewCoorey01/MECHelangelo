#!/usr/bin/env python3
"""
MECHelangelo real base driver.

This node subscribes to:

    /cmd_vel
        geometry_msgs/msg/Twist

It drives:

    Raspberry Pi GPIO -> motor driver PWM/direction pins

It reads:

    Left and right quadrature wheel encoders

It publishes:

    /odom
        nav_msgs/msg/Odometry

Optional TF:

    odom -> base_link

The purpose of this node is to make the real robot behave like the simulated
robot. Anything that publishes /cmd_vel, such as keyboard teleop or Nav2, can
command the robot without directly touching the motor GPIO pins.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster

from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from gpiozero import InputDevice


def constrain(value, low, high):
    return max(low, min(high, value))


def sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


class QuadratureEncoder:
    """
    4x quadrature decoder for a two-channel encoder.

    Each wheel encoder should have two signal wires:
        channel A
        channel B

    If the wheel physically spins forward but the measured speed is negative,
    flip that wheel's encoder_invert parameter.
    """

    _FORWARD_TRANSITIONS = {
        0b0001,
        0b0111,
        0b1110,
        0b1000,
    }

    _REVERSE_TRANSITIONS = {
        0b0010,
        0b1011,
        0b1101,
        0b0100,
    }

    def __init__(self, pin_a, pin_b, pull_up=False, invert=False, name='encoder'):
        self.name = name
        self.invert = invert

        self.a = InputDevice(pin_a, pull_up=pull_up)
        self.b = InputDevice(pin_b, pull_up=pull_up)

        self.count = 0
        self.lock = threading.Lock()

        self.last_state = self._read_state()

        self.a.when_activated = self._edge
        self.a.when_deactivated = self._edge
        self.b.when_activated = self._edge
        self.b.when_deactivated = self._edge

    def _read_state(self):
        return (int(self.a.value) << 1) | int(self.b.value)

    def _edge(self, device=None):
        new_state = self._read_state()
        transition = (self.last_state << 2) | new_state

        delta = 0

        if transition in self._FORWARD_TRANSITIONS:
            delta = 1
        elif transition in self._REVERSE_TRANSITIONS:
            delta = -1

        if self.invert:
            delta = -delta

        with self.lock:
            self.count += delta
            self.last_state = new_state

    def get_count(self):
        with self.lock:
            return self.count

    def close(self):
        self.a.close()
        self.b.close()


class Motor:
    """
    Signed motor driver wrapper.

    Command convention:
        +1.0 = full forward
        -1.0 = full reverse
         0.0 = stopped

    DRI0002 / MD1.3 style motor driver convention:
        E pin = PWM speed control
        M pin = direction control

    If a wheel spins backward when commanded forward, flip that wheel's
    motor_invert parameter.
    """

    def __init__(
        self,
        pwm_pin,
        dir_pin,
        pwm_frequency,
        max_duty,
        invert=False,
        name='motor',
    ):
        self.name = name
        self.invert = invert
        self.max_duty = max_duty

        self.pwm = PWMOutputDevice(
            pwm_pin,
            frequency=pwm_frequency,
            initial_value=0.0,
        )

        self.direction = DigitalOutputDevice(
            dir_pin,
            initial_value=False,
        )

        self.last_command = 0.0

    def set_duty_signed(self, command):
        command = constrain(command, -1.0, 1.0)

        if self.invert:
            command = -command

        self.last_command = command

        if abs(command) < 0.001:
            self.pwm.value = 0.0
            return

        duty = constrain(abs(command), 0.0, self.max_duty)

        if command > 0.0:
            self.direction.off()
        else:
            self.direction.on()

        self.pwm.value = duty

    def stop(self):
        self.last_command = 0.0
        self.pwm.value = 0.0

    def close(self):
        self.stop()
        self.pwm.close()
        self.direction.close()


class WheelSpeedController:
    """
    Closed-loop PI speed controller for one wheel.

    The target and measured values are in encoder ticks per second.
    The controller outputs a signed PWM duty command.
    """

    def __init__(
        self,
        motor,
        encoder,
        max_ticks_per_sec,
        max_duty,
        min_moving_duty,
        kp,
        ki,
        integral_limit,
        speed_filter_alpha,
        name='wheel',
    ):
        self.name = name
        self.motor = motor
        self.encoder = encoder

        self.max_ticks_per_sec = max_ticks_per_sec
        self.max_duty = max_duty
        self.min_moving_duty = min_moving_duty

        self.kp = kp
        self.ki = ki
        self.integral_limit = integral_limit
        self.speed_filter_alpha = speed_filter_alpha

        self.last_count = encoder.get_count()
        self.filtered_ticks_per_sec = 0.0
        self.integral = 0.0
        self.last_target_direction = 0.0

    def update(self, target_ticks_per_sec, dt):
        current_count = self.encoder.get_count()
        delta_count = current_count - self.last_count
        self.last_count = current_count

        raw_ticks_per_sec = delta_count / dt if dt > 0.0 else 0.0

        self.filtered_ticks_per_sec = (
            self.speed_filter_alpha * raw_ticks_per_sec
            + (1.0 - self.speed_filter_alpha) * self.filtered_ticks_per_sec
        )

        target_direction = sign(target_ticks_per_sec)

        if target_direction == 0.0:
            self.integral = 0.0
            self.last_target_direction = 0.0
            self.motor.stop()
            return 0.0, self.filtered_ticks_per_sec, current_count, delta_count

        if target_direction != self.last_target_direction:
            self.integral = 0.0
            self.last_target_direction = target_direction

        measured_along_target = self.filtered_ticks_per_sec * target_direction
        speed_error = abs(target_ticks_per_sec) - measured_along_target

        self.integral = constrain(
            self.integral + speed_error * dt,
            -self.integral_limit,
            self.integral_limit,
        )

        if self.max_ticks_per_sec <= 0.0:
            feedforward = 0.0
        else:
            feedforward = (
                abs(target_ticks_per_sec) / self.max_ticks_per_sec
            ) * self.max_duty

        correction = self.kp * speed_error + self.ki * self.integral

        duty = feedforward + correction
        duty = constrain(duty, self.min_moving_duty, self.max_duty)

        signed_duty = target_direction * duty
        self.motor.set_duty_signed(signed_duty)

        return signed_duty, self.filtered_ticks_per_sec, current_count, delta_count

    def stop(self):
        self.integral = 0.0
        self.motor.stop()


class MechelangeloBaseDriver(Node):
    def __init__(self):
        super().__init__('mechelangelo_base_driver')

        self._hardware_closed = False

        self.load_parameters()

        self.left_motor = Motor(
            pwm_pin=self.left_pwm_pin,
            dir_pin=self.left_dir_pin,
            pwm_frequency=self.pwm_frequency,
            max_duty=self.max_duty,
            invert=self.left_motor_invert,
            name='left_motor',
        )

        self.right_motor = Motor(
            pwm_pin=self.right_pwm_pin,
            dir_pin=self.right_dir_pin,
            pwm_frequency=self.pwm_frequency,
            max_duty=self.max_duty,
            invert=self.right_motor_invert,
            name='right_motor',
        )

        self.left_encoder = QuadratureEncoder(
            pin_a=self.left_enc_a_pin,
            pin_b=self.left_enc_b_pin,
            pull_up=self.encoder_pull_up,
            invert=self.left_encoder_invert,
            name='left_encoder',
        )

        self.right_encoder = QuadratureEncoder(
            pin_a=self.right_enc_a_pin,
            pin_b=self.right_enc_b_pin,
            pull_up=self.encoder_pull_up,
            invert=self.right_encoder_invert,
            name='right_encoder',
        )

        self.left_controller = WheelSpeedController(
            motor=self.left_motor,
            encoder=self.left_encoder,
            max_ticks_per_sec=self.max_ticks_per_sec,
            max_duty=self.max_duty,
            min_moving_duty=self.min_moving_duty,
            kp=self.kp,
            ki=self.ki,
            integral_limit=self.integral_limit,
            speed_filter_alpha=self.speed_filter_alpha,
            name='left',
        )

        self.right_controller = WheelSpeedController(
            motor=self.right_motor,
            encoder=self.right_encoder,
            max_ticks_per_sec=self.max_ticks_per_sec,
            max_duty=self.max_duty,
            min_moving_duty=self.min_moving_duty,
            kp=self.kp,
            ki=self.ki,
            integral_limit=self.integral_limit,
            speed_filter_alpha=self.speed_filter_alpha,
            name='right',
        )

        self.target_linear_mps = 0.0
        self.target_angular_radps = 0.0

        self.last_cmd_time = time.time()
        self.last_loop_time = time.time()
        self.last_status_time = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_left_actual_tps = 0.0
        self.last_right_actual_tps = 0.0

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )

        if self.publish_odom:
            self.odom_pub = self.create_publisher(
                Odometry,
                self.odom_topic,
                10,
            )
        else:
            self.odom_pub = None

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.get_logger().info('MECHelangelo base driver started.')
        self.get_logger().info(f'Subscribing to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Publishing odometry: {self.publish_odom}')
        self.get_logger().info(f'Publishing odom -> base TF: {self.publish_tf}')
        self.get_logger().info(
            f'Wheel diameter: {self.wheel_diameter_m:.4f} m, '
            f'wheel separation: {self.wheel_separation_m:.4f} m'
        )
        self.get_logger().info(
            f'Max wheel speed estimate: {self.max_wheel_rpm:.1f} rpm, '
            f'{self.max_ticks_per_sec:.1f} ticks/s'
        )

    def load_parameters(self):
        """
        Declare and load ROS parameters.

        These can be overridden from config/base_driver.yaml.
        """

        # GPIO pins, BCM numbering
        self.declare_parameter('right_pwm_pin', 12)
        self.declare_parameter('right_dir_pin', 23)
        self.declare_parameter('left_pwm_pin', 13)
        self.declare_parameter('left_dir_pin', 24)

        self.declare_parameter('left_enc_a_pin', 5)
        self.declare_parameter('left_enc_b_pin', 6)
        self.declare_parameter('right_enc_a_pin', 16)
        self.declare_parameter('right_enc_b_pin', 20)

        # Motor and encoder direction correction
        self.declare_parameter('left_motor_invert', False)
        self.declare_parameter('right_motor_invert', False)
        self.declare_parameter('left_encoder_invert', False)
        self.declare_parameter('right_encoder_invert', False)
        self.declare_parameter('encoder_pull_up', False)

        # Motor settings
        self.declare_parameter('pwm_frequency', 1000)
        self.declare_parameter('max_duty', 0.35)
        self.declare_parameter('min_moving_duty', 0.18)
        self.declare_parameter('max_wheel_rpm', 60.0)

        # Robot geometry
        self.declare_parameter('wheel_diameter_m', 0.1524)
        self.declare_parameter('wheel_separation_m', 0.50)

        # Encoder settings
        self.declare_parameter('encoder_ticks_per_rev', 144.0)

        # Velocity limits
        self.declare_parameter('max_linear_vel_mps', 0.15)
        self.declare_parameter('max_angular_vel_radps', 0.60)

        # Control settings
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('kp', 0.0010)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('integral_limit', 150.0)
        self.declare_parameter('speed_filter_alpha', 0.35)

        # ROS interface
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('status_period', 0.5)

        self.right_pwm_pin = self.get_parameter('right_pwm_pin').value
        self.right_dir_pin = self.get_parameter('right_dir_pin').value
        self.left_pwm_pin = self.get_parameter('left_pwm_pin').value
        self.left_dir_pin = self.get_parameter('left_dir_pin').value

        self.left_enc_a_pin = self.get_parameter('left_enc_a_pin').value
        self.left_enc_b_pin = self.get_parameter('left_enc_b_pin').value
        self.right_enc_a_pin = self.get_parameter('right_enc_a_pin').value
        self.right_enc_b_pin = self.get_parameter('right_enc_b_pin').value

        self.left_motor_invert = self.get_parameter('left_motor_invert').value
        self.right_motor_invert = self.get_parameter('right_motor_invert').value
        self.left_encoder_invert = self.get_parameter('left_encoder_invert').value
        self.right_encoder_invert = self.get_parameter('right_encoder_invert').value
        self.encoder_pull_up = self.get_parameter('encoder_pull_up').value

        self.pwm_frequency = self.get_parameter('pwm_frequency').value
        self.max_duty = self.get_parameter('max_duty').value
        self.min_moving_duty = self.get_parameter('min_moving_duty').value
        self.max_wheel_rpm = self.get_parameter('max_wheel_rpm').value

        self.wheel_diameter_m = self.get_parameter('wheel_diameter_m').value
        self.wheel_separation_m = self.get_parameter('wheel_separation_m').value
        self.encoder_ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value

        self.max_linear_vel_mps = self.get_parameter('max_linear_vel_mps').value
        self.max_angular_vel_radps = self.get_parameter('max_angular_vel_radps').value

        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.control_period = self.get_parameter('control_period').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.speed_filter_alpha = self.get_parameter('speed_filter_alpha').value

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_odom = self.get_parameter('publish_odom').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.status_period = self.get_parameter('status_period').value

        self.wheel_circumference_m = math.pi * self.wheel_diameter_m

        self.max_ticks_per_sec = (
            self.encoder_ticks_per_rev * self.max_wheel_rpm / 60.0
        )

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

    def wheel_velocity_to_ticks_per_sec(self, wheel_velocity_mps):
        if self.wheel_circumference_m <= 0.0:
            return 0.0

        wheel_rev_per_sec = wheel_velocity_mps / self.wheel_circumference_m
        return wheel_rev_per_sec * self.encoder_ticks_per_rev

    def ticks_to_distance_m(self, ticks):
        if self.encoder_ticks_per_rev <= 0.0:
            return 0.0

        wheel_revs = ticks / self.encoder_ticks_per_rev
        return wheel_revs * self.wheel_circumference_m

    def control_loop(self):
        now = time.time()
        dt = max(now - self.last_loop_time, 0.001)
        self.last_loop_time = now

        if now - self.last_cmd_time > self.cmd_timeout:
            linear = 0.0
            angular = 0.0
        else:
            linear = self.target_linear_mps
            angular = self.target_angular_radps

        # Differential drive inverse kinematics:
        #
        # left wheel velocity  = v - omega * wheel_separation / 2
        # right wheel velocity = v + omega * wheel_separation / 2

        left_velocity_mps = linear - angular * (self.wheel_separation_m / 2.0)
        right_velocity_mps = linear + angular * (self.wheel_separation_m / 2.0)

        left_target_tps = self.wheel_velocity_to_ticks_per_sec(left_velocity_mps)
        right_target_tps = self.wheel_velocity_to_ticks_per_sec(right_velocity_mps)

        left_target_tps = constrain(
            left_target_tps,
            -self.max_ticks_per_sec,
            self.max_ticks_per_sec,
        )

        right_target_tps = constrain(
            right_target_tps,
            -self.max_ticks_per_sec,
            self.max_ticks_per_sec,
        )

        left_duty, left_actual_tps, left_count, left_delta_count = (
            self.left_controller.update(left_target_tps, dt)
        )

        right_duty, right_actual_tps, right_count, right_delta_count = (
            self.right_controller.update(right_target_tps, dt)
        )

        self.last_left_actual_tps = left_actual_tps
        self.last_right_actual_tps = right_actual_tps

        if self.publish_odom:
            self.update_and_publish_odometry(
                left_delta_count,
                right_delta_count,
                dt,
            )

        if now - self.last_status_time > self.status_period:
            self.last_status_time = now
            self.print_status(
                linear,
                angular,
                left_target_tps,
                right_target_tps,
                left_actual_tps,
                right_actual_tps,
                left_duty,
                right_duty,
                left_count,
                right_count,
            )

    def update_and_publish_odometry(self, left_delta_count, right_delta_count, dt):
        left_distance = self.ticks_to_distance_m(left_delta_count)
        right_distance = self.ticks_to_distance_m(right_delta_count)

        centre_distance = (right_distance + left_distance) / 2.0

        if self.wheel_separation_m <= 0.0:
            delta_theta = 0.0
        else:
            delta_theta = (right_distance - left_distance) / self.wheel_separation_m

        theta_mid = self.theta + (delta_theta / 2.0)

        self.x += centre_distance * math.cos(theta_mid)
        self.y += centre_distance * math.sin(theta_mid)
        self.theta += delta_theta

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        linear_velocity = centre_distance / dt if dt > 0.0 else 0.0
        angular_velocity = delta_theta / dt if dt > 0.0 else 0.0

        now_msg = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        # Simple covariance placeholders.
        # These can be tuned later for robot_localization/Nav2.
        odom_msg.pose.covariance[0] = 0.02
        odom_msg.pose.covariance[7] = 0.02
        odom_msg.pose.covariance[35] = 0.05

        odom_msg.twist.covariance[0] = 0.05
        odom_msg.twist.covariance[7] = 0.05
        odom_msg.twist.covariance[35] = 0.1

        self.odom_pub.publish(odom_msg)

        if self.publish_tf and self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = now_msg
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id

            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0

            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(transform)

    def print_status(
        self,
        linear,
        angular,
        left_target_tps,
        right_target_tps,
        left_actual_tps,
        right_actual_tps,
        left_duty,
        right_duty,
        left_count,
        right_count,
    ):
        left_rpm = (left_actual_tps / self.encoder_ticks_per_rev) * 60.0
        right_rpm = (right_actual_tps / self.encoder_ticks_per_rev) * 60.0

        self.get_logger().info(
            f'cmd v={linear:+.3f} m/s, w={angular:+.3f} rad/s | '
            f'L tgt={left_target_tps:+6.1f} act={left_actual_tps:+6.1f} t/s '
            f'rpm={left_rpm:+5.1f} duty={left_duty:+.2f} count={left_count:+7d} | '
            f'R tgt={right_target_tps:+6.1f} act={right_actual_tps:+6.1f} t/s '
            f'rpm={right_rpm:+5.1f} duty={right_duty:+.2f} count={right_count:+7d}'
        )

    def stop_all(self):
        try:
            self.left_controller.stop()
            self.right_controller.stop()
        except AttributeError:
            pass

    def close_hardware(self):
        if self._hardware_closed:
            return

        self._hardware_closed = True

        self.stop_all()

        try:
            self.left_encoder.close()
            self.right_encoder.close()
            self.left_motor.close()
            self.right_motor.close()
        except AttributeError:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = MechelangeloBaseDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.get_logger().info('Stopping motors and closing GPIO.')
        node.close_hardware()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()