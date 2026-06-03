# #!/usr/bin/env python3
# """
# MECHelangelo ThunderBorg base driver.

# This node replaces the GPIO PWM/direction base driver when the robot is using
# PiBorg ThunderBorg motor control hardware.

# It subscribes to:

#     /cmd_vel
#         geometry_msgs/msg/Twist

# It drives:

#     ThunderBorg Motor 1 and Motor 2

# Optional future extension:

#     Wheel encoders can be added later for closed-loop odometry.

# The purpose of this node is to make the real robot behave like the simulated
# robot. Anything that publishes /cmd_vel, such as keyboard teleop, Nav2, or the
# MECHelangelo behaviour node, can command the robot without directly touching the
# ThunderBorg API.
# """

# import math
# import time

# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist

# from mechelangelo_base_driver import ThunderBorg

# import threading
# from gpiozero import DigitalInputDevice


# def constrain(value, low, high):
#     return max(low, min(high, value))

# class QuadratureEncoder:
#     """
#     4x quadrature decoder for one Parallax encoder.

#     Uses two GPIO inputs:
#         A = encoder channel 1
#         B = encoder channel 2
#     """

#     FORWARD_TRANSITIONS = {0b0001, 0b0111, 0b1110, 0b1000}
#     REVERSE_TRANSITIONS = {0b0010, 0b1011, 0b1101, 0b0100}

#     def __init__(self, pin_a, pin_b, pull_up=False, invert=False, name='encoder'):
#         self.name = name
#         self.invert = invert
#         self.a = DigitalInputDevice(pin_a, pull_up=pull_up)
#         self.b = DigitalInputDevice(pin_b, pull_up=pull_up)

#         self.count = 0
#         self.lock = threading.Lock()
#         self.last_state = self._read_state()

#         self.a.when_activated = self._edge
#         self.a.when_deactivated = self._edge
#         self.b.when_activated = self._edge
#         self.b.when_deactivated = self._edge

#     def _read_state(self):
#         return (int(self.a.value) << 1) | int(self.b.value)

#     def _edge(self, device=None):
#         new_state = self._read_state()
#         transition = (self.last_state << 2) | new_state

#         delta = 0
#         if transition in self.FORWARD_TRANSITIONS:
#             delta = 1
#         elif transition in self.REVERSE_TRANSITIONS:
#             delta = -1

#         if self.invert:
#             delta = -delta

#         with self.lock:
#             self.count += delta
#             self.last_state = new_state

#     def get_count(self):
#         with self.lock:
#             return self.count

#     def close(self):
#         self.a.close()
#         self.b.close()


# class MechelangeloThunderBorgDriver(Node):
#     def __init__(self):
#         super().__init__('mechelangelo_base_driver')

#         self.load_parameters()

#         self.target_linear_mps = 0.0
#         self.target_angular_radps = 0.0
#         self.last_cmd_time = time.time()
#         self.last_status_time = 0.0

#         ##added for encoder implementation
#         self.encoder = None
#         self.last_encoder_count = 0
#         self.last_encoder_time = time.time()

#         if self.enable_encoder:
#             self.encoder = QuadratureEncoder(
#                 self.encoder_a_pin,
#                 self.encoder_b_pin,
#                 pull_up=self.encoder_pull_up,
#                 invert=self.encoder_invert,
#                 name='wheel_encoder',
#             )

#             self.last_encoder_count = self.encoder.get_count()

#             self.get_logger().info(
#                 f'Encoder enabled on GPIO{self.encoder_a_pin} and GPIO{self.encoder_b_pin}'
#             )
#         ####

#         self.tb = ThunderBorg.ThunderBorg()
#         self.tb.Init()

#         if not self.tb.foundChip:
#             boards = ThunderBorg.ScanForThunderBorg()
#             self.get_logger().error('No ThunderBorg found.')
#             self.get_logger().error(f'ThunderBorg boards found: {boards}')
#             raise RuntimeError('ThunderBorg not found')

#         self.get_logger().info('ThunderBorg connected.')

#         try:
#             battery_voltage = self.tb.GetBatteryReading()
#             self.get_logger().info(f'ThunderBorg battery voltage: {battery_voltage:.2f} V')
#         except Exception as exc:
#             self.get_logger().warn(f'Could not read ThunderBorg battery voltage: {exc}')

#         self.configure_thunderborg_failsafe()

#         self.tb.MotorsOff()

#         if self.show_battery_led:
#             self.tb.SetLedShowBattery(True)

#         self.cmd_sub = self.create_subscription(
#             Twist,
#             self.cmd_vel_topic,
#             self.cmd_vel_callback,
#             10,
#         )

#         self.control_timer = self.create_timer(
#             self.control_period,
#             self.control_loop,
#         )

#         self.get_logger().info('MECHelangelo ThunderBorg base driver started.')
#         self.get_logger().info(f'Subscribing to: {self.cmd_vel_topic}')
#         self.get_logger().info(
#             f'Motor mapping: Motor1 is {"right" if self.motor1_is_right else "left"}, '
#             f'Motor2 is {"left" if self.motor1_is_right else "right"}'
#         )
#         self.get_logger().info(
#             f'Limits: max_linear={self.max_linear_vel_mps:.3f} m/s, '
#             f'max_angular={self.max_angular_vel_radps:.3f} rad/s, '
#             f'max_power={self.max_power:.2f}'
#         )

#     def load_parameters(self):
#         """
#         Declare and load ROS parameters.

#         These can be overridden from a YAML file or launch file.
#         """

#         # ROS interface
#         self.declare_parameter('cmd_vel_topic', 'cmd_vel')

#         # Robot geometry
#         self.declare_parameter('wheel_separation_m', 0.50)

#         # Velocity limits accepted from /cmd_vel
#         self.declare_parameter('max_linear_vel_mps', 0.15)
#         self.declare_parameter('max_angular_vel_radps', 0.60)

#         # ThunderBorg output scaling
#         self.declare_parameter('max_power', 0.75)
#         self.declare_parameter('deadband', 0.03)

#         # Motor mapping and direction correction
#         # PiBorg's tbJoystick.py commonly treats:
#         #   Motor 1 = right wheel
#         #   Motor 2 = left wheel
#         self.declare_parameter('motor1_is_right', True)
#         self.declare_parameter('left_motor_invert', False)
#         self.declare_parameter('right_motor_invert', False)

#         # Control/safety settings
#         self.declare_parameter('cmd_timeout', 0.5)
#         self.declare_parameter('control_period', 0.05)
#         self.declare_parameter('status_period', 0.5)
#         self.declare_parameter('enable_comms_failsafe', True)
#         self.declare_parameter('show_battery_led', True)

#         # Single encoder debug input ##added for encoder implementation
#         self.declare_parameter('enable_encoder', True)
#         self.declare_parameter('encoder_a_pin', 24)
#         self.declare_parameter('encoder_b_pin', 25)
#         self.declare_parameter('encoder_pull_up', False)
#         self.declare_parameter('encoder_invert', False)
#         self.declare_parameter('encoder_ticks_per_rev', 144.0)

#         self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
#         self.wheel_separation_m = float(self.get_parameter('wheel_separation_m').value)
#         self.max_linear_vel_mps = float(self.get_parameter('max_linear_vel_mps').value)
#         self.max_angular_vel_radps = float(self.get_parameter('max_angular_vel_radps').value)
#         self.max_power = float(self.get_parameter('max_power').value)
#         self.deadband = float(self.get_parameter('deadband').value)
#         self.motor1_is_right = bool(self.get_parameter('motor1_is_right').value)
#         self.left_motor_invert = bool(self.get_parameter('left_motor_invert').value)
#         self.right_motor_invert = bool(self.get_parameter('right_motor_invert').value)
#         self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
#         self.control_period = float(self.get_parameter('control_period').value)
#         self.status_period = float(self.get_parameter('status_period').value)
#         self.enable_comms_failsafe = bool(self.get_parameter('enable_comms_failsafe').value)
#         self.show_battery_led = bool(self.get_parameter('show_battery_led').value)
#         self.enable_encoder = bool(self.get_parameter('enable_encoder').value)
#         self.encoder_a_pin = int(self.get_parameter('encoder_a_pin').value) ##added for encoder implementation
#         self.encoder_b_pin = int(self.get_parameter('encoder_b_pin').value)
#         self.encoder_pull_up = bool(self.get_parameter('encoder_pull_up').value)
#         self.encoder_invert = bool(self.get_parameter('encoder_invert').value)
#         self.encoder_ticks_per_rev = float(self.get_parameter('encoder_ticks_per_rev').value)

#         self.max_power = constrain(self.max_power, 0.0, 1.0)

#     def configure_thunderborg_failsafe(self):
#         """
#         Enable the ThunderBorg communication failsafe if requested.

#         ThunderBorg can turn motors off if motor commands stop arriving. This is
#         separate from this node's /cmd_vel timeout and gives another safety layer.
#         """

#         if not self.enable_comms_failsafe:
#             self.get_logger().warn('ThunderBorg comms failsafe disabled by parameter.')
#             return

#         failsafe_ok = False

#         for _ in range(5):
#             try:
#                 self.tb.SetCommsFailsafe(True)
#                 time.sleep(0.02)
#                 if self.tb.GetCommsFailsafe():
#                     failsafe_ok = True
#                     break
#             except Exception:
#                 pass

#         if failsafe_ok:
#             self.get_logger().info('ThunderBorg comms failsafe enabled.')
#         else:
#             self.get_logger().warn('Could not confirm ThunderBorg comms failsafe.')

#     def cmd_vel_callback(self, msg):
#         self.target_linear_mps = constrain(
#             msg.linear.x,
#             -self.max_linear_vel_mps,
#             self.max_linear_vel_mps,
#         )

#         self.target_angular_radps = constrain(
#             msg.angular.z,
#             -self.max_angular_vel_radps,
#             self.max_angular_vel_radps,
#         )

#         self.last_cmd_time = time.time()

#     def apply_deadband(self, value):
#         if abs(value) < self.deadband:
#             return 0.0
#         return value

#     def twist_to_wheel_commands(self, linear_mps, angular_radps):
#         """
#         Convert robot linear/angular velocity into left/right wheel commands.

#         Differential drive kinematics:

#             left_velocity  = v - omega * wheel_separation / 2
#             right_velocity = v + omega * wheel_separation / 2

#         These wheel velocities are then normalised into motor commands in the
#         range -1.0 to +1.0 before being scaled by max_power.
#         """

#         left_velocity = linear_mps - angular_radps * (self.wheel_separation_m / 2.0)
#         right_velocity = linear_mps + angular_radps * (self.wheel_separation_m / 2.0)

#         # Normalise relative to the configured maximums.
#         # This keeps the output proportional while preventing either wheel from
#         # exceeding the allowed command range.
#         max_wheel_velocity = self.max_linear_vel_mps + (
#             self.max_angular_vel_radps * self.wheel_separation_m / 2.0
#         )

#         if max_wheel_velocity <= 0.0:
#             return 0.0, 0.0

#         left_command = left_velocity / max_wheel_velocity
#         right_command = right_velocity / max_wheel_velocity

#         scale = max(1.0, abs(left_command), abs(right_command))
#         left_command /= scale
#         right_command /= scale

#         return left_command, right_command

#     def send_motor_command(self, left_command, right_command):
#         """
#         Send signed left/right motor commands to the ThunderBorg.

#         Command convention:
#             + = forward
#             - = reverse
#              0 = stopped
#         """

#         left_command = self.apply_deadband(left_command)
#         right_command = self.apply_deadband(right_command)

#         if self.left_motor_invert:
#             left_command = -left_command
#         if self.right_motor_invert:
#             right_command = -right_command

#         left_output = constrain(left_command, -1.0, 1.0) * self.max_power
#         right_output = constrain(right_command, -1.0, 1.0) * self.max_power

#         if self.motor1_is_right:
#             self.tb.SetMotor1(right_output)
#             self.tb.SetMotor2(left_output)
#         else:
#             self.tb.SetMotor1(left_output)
#             self.tb.SetMotor2(right_output)

#         return left_output, right_output

#     def control_loop(self):
#         now = time.time()

#         if now - self.last_cmd_time > self.cmd_timeout:
#             linear = 0.0
#             angular = 0.0
#         else:
#             linear = self.target_linear_mps
#             angular = self.target_angular_radps

#         left_command, right_command = self.twist_to_wheel_commands(linear, angular)
#         left_output, right_output = self.send_motor_command(left_command, right_command)

#         if now - self.last_status_time > self.status_period:
#             self.last_status_time = now
#             self.print_status(linear, angular, left_output, right_output)

#     def print_status(self, linear, angular, left_output, right_output):  ##replaced for encoder implementation
#         try:
#             fault1 = self.tb.GetDriveFault1()
#             fault2 = self.tb.GetDriveFault2()
#         except Exception:
#             fault1 = 'unknown'
#             fault2 = 'unknown'

#         encoder_text = ''

#         if self.encoder is not None:
#             now = time.time()
#             count = self.encoder.get_count()

#             dt = max(now - self.last_encoder_time, 1e-3)
#             delta = count - self.last_encoder_count

#             ticks_per_sec = delta / dt
#             rpm = (ticks_per_sec / self.encoder_ticks_per_rev) * 60.0

#             self.last_encoder_count = count
#             self.last_encoder_time = now

#             encoder_text = (
#                 f' | ENC count={count:+7d}, '
#                 f'speed={ticks_per_sec:+7.1f} ticks/s, '
#                 f'rpm={rpm:+6.1f}'
#             )

#         self.get_logger().info(
#             f'cmd v={linear:+.3f} m/s, w={angular:+.3f} rad/s | '
#             f'L out={left_output:+.2f}, R out={right_output:+.2f} | '
#             f'Fault1={fault1}, Fault2={fault2}'
#             f'{encoder_text}'
#         )
#     def stop_all(self):
#         try:
#             self.tb.MotorsOff()
#         except AttributeError:
#             pass
#         except Exception as exc:
#             self.get_logger().warn(f'Could not stop ThunderBorg motors cleanly: {exc}')

#     def close_hardware(self):
#         self.stop_all()

#         try: ##added for encoder implementation
#             if self.encoder is not None:
#                 self.encoder.close()
#         except Exception as exc:
#             self.get_logger().warn(f'Could not close encoder GPIO cleanly: {exc}')

#         try:
#             self.tb.SetCommsFailsafe(False)
#         except Exception:
#             pass

#         try:
#             self.tb.SetLedShowBattery(False)
#             self.tb.SetLeds(0.2, 0.0, 0.0)
#         except Exception:
#             pass


# def main(args=None):
#     rclpy.init(args=args)

#     node = None

#     try:
#         node = MechelangeloThunderBorgDriver()
#         rclpy.spin(node)

#     except KeyboardInterrupt:
#         pass

#     except Exception as exc:
#         if rclpy.ok():
#             print(f'MECHelangelo ThunderBorg driver error: {exc}')
#         raise

#     finally:
#         if node is not None:
#             node.get_logger().info('Stopping ThunderBorg motors and closing driver.')
#             node.close_hardware()
#             node.destroy_node()

#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
"""
MECHelangelo ThunderBorg base driver with dual wheel encoder debug feedback.

This node subscribes to:

    /cmd_vel
        geometry_msgs/msg/Twist

It drives:

    ThunderBorg Motor 1 and Motor 2

It reads:

    Left wheel quadrature encoder
    Right wheel quadrature encoder

Important:
    This version still uses open-loop ThunderBorg motor control.
    The encoders are read and printed for debugging only.
    They are not yet used for closed-loop speed control or odometry.
"""

import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from gpiozero import DigitalInputDevice

from mechelangelo_base_driver import ThunderBorg


def constrain(value, low, high):
    return max(low, min(high, value))


class QuadratureEncoder:
    """
    4x quadrature decoder for one wheel encoder.

    Each encoder uses two GPIO inputs:
        A = encoder channel A
        B = encoder channel B
    """

    FORWARD_TRANSITIONS = {0b0001, 0b0111, 0b1110, 0b1000}
    REVERSE_TRANSITIONS = {0b0010, 0b1011, 0b1101, 0b0100}

    def __init__(self, pin_a, pin_b, pull_up=True, invert=False, name='encoder'):
        self.name = name
        self.invert = invert

        self.a = DigitalInputDevice(pin_a, pull_up=pull_up)
        self.b = DigitalInputDevice(pin_b, pull_up=pull_up)

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

        if transition in self.FORWARD_TRANSITIONS:
            delta = 1
        elif transition in self.REVERSE_TRANSITIONS:
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


class MechelangeloThunderBorgDriver(Node):
    def __init__(self):
        super().__init__('mechelangelo_base_driver')

        self.load_parameters()

        self.target_linear_mps = 0.0
        self.target_angular_radps = 0.0
        self.last_cmd_time = time.time()
        self.last_status_time = 0.0

        # ============================================================
        # Dual encoder setup
        # ============================================================

        self.left_encoder = None
        self.right_encoder = None

        self.last_left_encoder_count = 0
        self.last_right_encoder_count = 0

        self.last_left_encoder_time = time.time()
        self.last_right_encoder_time = time.time()

        if self.enable_encoders:
            self.left_encoder = QuadratureEncoder(
                self.left_encoder_a_pin,
                self.left_encoder_b_pin,
                pull_up=self.encoder_pull_up,
                invert=self.left_encoder_invert,
                name='left_encoder',
            )

            self.right_encoder = QuadratureEncoder(
                self.right_encoder_a_pin,
                self.right_encoder_b_pin,
                pull_up=self.encoder_pull_up,
                invert=self.right_encoder_invert,
                name='right_encoder',
            )

            self.last_left_encoder_count = self.left_encoder.get_count()
            self.last_right_encoder_count = self.right_encoder.get_count()

            self.get_logger().info(
                f'Left encoder enabled on GPIO{self.left_encoder_a_pin} '
                f'and GPIO{self.left_encoder_b_pin}'
            )

            self.get_logger().info(
                f'Right encoder enabled on GPIO{self.right_encoder_a_pin} '
                f'and GPIO{self.right_encoder_b_pin}'
            )

        # ============================================================
        # ThunderBorg setup
        # ============================================================

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
            self.get_logger().info(
                f'ThunderBorg battery voltage: {battery_voltage:.2f} V'
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Could not read ThunderBorg battery voltage: {exc}'
            )

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
        self.declare_parameter('wheel_separation_m', 0.42)

        # Velocity limits accepted from /cmd_vel
        self.declare_parameter('max_linear_vel_mps', 0.15)
        self.declare_parameter('max_angular_vel_radps', 0.60)

        # ThunderBorg output scaling
        self.declare_parameter('max_power', 1.0)
        self.declare_parameter('deadband', 0.03)

        # Motor mapping and direction correction
        self.declare_parameter('motor1_is_right', True)
        self.declare_parameter('left_motor_invert', True)
        self.declare_parameter('right_motor_invert', True)

        # Control/safety settings
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('status_period', 0.5)
        self.declare_parameter('enable_comms_failsafe', True)
        self.declare_parameter('show_battery_led', True)

        # Dual encoder debug inputs
        self.declare_parameter('enable_encoders', True)

        self.declare_parameter('left_encoder_a_pin', 23)
        self.declare_parameter('left_encoder_b_pin', 22)

        self.declare_parameter('right_encoder_a_pin', 24)
        self.declare_parameter('right_encoder_b_pin', 25)

        self.declare_parameter('encoder_pull_up', True)

        self.declare_parameter('left_encoder_invert', False)
        self.declare_parameter('right_encoder_invert', False)

        self.declare_parameter('encoder_ticks_per_rev', 144.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.wheel_separation_m = float(
            self.get_parameter('wheel_separation_m').value
        )

        self.max_linear_vel_mps = float(
            self.get_parameter('max_linear_vel_mps').value
        )

        self.max_angular_vel_radps = float(
            self.get_parameter('max_angular_vel_radps').value
        )

        self.max_power = float(self.get_parameter('max_power').value)
        self.deadband = float(self.get_parameter('deadband').value)

        self.motor1_is_right = bool(
            self.get_parameter('motor1_is_right').value
        )

        self.left_motor_invert = bool(
            self.get_parameter('left_motor_invert').value
        )

        self.right_motor_invert = bool(
            self.get_parameter('right_motor_invert').value
        )

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.control_period = float(self.get_parameter('control_period').value)
        self.status_period = float(self.get_parameter('status_period').value)

        self.enable_comms_failsafe = bool(
            self.get_parameter('enable_comms_failsafe').value
        )

        self.show_battery_led = bool(
            self.get_parameter('show_battery_led').value
        )

        self.enable_encoders = bool(
            self.get_parameter('enable_encoders').value
        )

        self.left_encoder_a_pin = int(
            self.get_parameter('left_encoder_a_pin').value
        )

        self.left_encoder_b_pin = int(
            self.get_parameter('left_encoder_b_pin').value
        )

        self.right_encoder_a_pin = int(
            self.get_parameter('right_encoder_a_pin').value
        )

        self.right_encoder_b_pin = int(
            self.get_parameter('right_encoder_b_pin').value
        )

        self.encoder_pull_up = bool(
            self.get_parameter('encoder_pull_up').value
        )

        self.left_encoder_invert = bool(
            self.get_parameter('left_encoder_invert').value
        )

        self.right_encoder_invert = bool(
            self.get_parameter('right_encoder_invert').value
        )

        self.encoder_ticks_per_rev = float(
            self.get_parameter('encoder_ticks_per_rev').value
        )

        self.max_power = constrain(self.max_power, 0.0, 1.0)

    def configure_thunderborg_failsafe(self):
        """
        Enable the ThunderBorg communication failsafe if requested.
        """

        if not self.enable_comms_failsafe:
            self.get_logger().warn(
                'ThunderBorg comms failsafe disabled by parameter.'
            )
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
            self.get_logger().warn(
                'Could not confirm ThunderBorg comms failsafe.'
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

    def apply_deadband(self, value):
        if abs(value) < self.deadband:
            return 0.0
        return value

    def twist_to_wheel_commands(self, linear_mps, angular_radps):
        """
        Convert robot linear/angular velocity into left/right wheel commands.

        Differential-drive kinematics:

            left_velocity  = v - omega * wheel_separation / 2
            right_velocity = v + omega * wheel_separation / 2
        """

        left_velocity = (
            linear_mps - angular_radps * (self.wheel_separation_m / 2.0)
        )

        right_velocity = (
            linear_mps + angular_radps * (self.wheel_separation_m / 2.0)
        )

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

        left_command, right_command = self.twist_to_wheel_commands(
            linear,
            angular,
        )

        left_output, right_output = self.send_motor_command(
            left_command,
            right_command,
        )

        if now - self.last_status_time > self.status_period:
            self.last_status_time = now
            self.print_status(linear, angular, left_output, right_output)

    def encoder_status_text(self):
        """
        Build status string for both encoders.
        """

        text = ''
        now = time.time()

        if self.left_encoder is not None:
            left_count = self.left_encoder.get_count()
            left_dt = max(now - self.last_left_encoder_time, 1e-3)
            left_delta = left_count - self.last_left_encoder_count

            left_tps = left_delta / left_dt
            left_rpm = (left_tps / self.encoder_ticks_per_rev) * 60.0

            self.last_left_encoder_count = left_count
            self.last_left_encoder_time = now

            # text += (
            #     f' | L ENC count={left_count:+7d}, '
            #     f'speed={left_tps:+7.1f} ticks/s, '
            #     f'rpm={left_rpm:+6.1f}'
            # )

        if self.right_encoder is not None:
            right_count = self.right_encoder.get_count()
            right_dt = max(now - self.last_right_encoder_time, 1e-3)
            right_delta = right_count - self.last_right_encoder_count

            right_tps = right_delta / right_dt
            right_rpm = (right_tps / self.encoder_ticks_per_rev) * 60.0

            self.last_right_encoder_count = right_count
            self.last_right_encoder_time = now

            # text += (
            #     f' | R ENC count={right_count:+7d}, '
            #     f'speed={right_tps:+7.1f} ticks/s, '
            #     f'rpm={right_rpm:+6.1f}'
            # )

        return text

    def print_status(self, linear, angular, left_output, right_output):
        try:
            fault1 = self.tb.GetDriveFault1()
            fault2 = self.tb.GetDriveFault2()
        except Exception:
            fault1 = 'unknown'
            fault2 = 'unknown'

        encoder_text = self.encoder_status_text()

        # self.get_logger().info(
        #     f'cmd v={linear:+.3f} m/s, w={angular:+.3f} rad/s | '
        #     f'L out={left_output:+.2f}, R out={right_output:+.2f} | '
        #     f'Fault1={fault1}, Fault2={fault2}'
        #     f'{encoder_text}'
        # )

    def stop_all(self):
        try:
            self.tb.MotorsOff()
        except AttributeError:
            pass
        except Exception as exc:
            self.get_logger().warn(
                f'Could not stop ThunderBorg motors cleanly: {exc}'
            )

    def close_hardware(self):
        self.stop_all()

        try:
            if self.left_encoder is not None:
                self.left_encoder.close()

            if self.right_encoder is not None:
                self.right_encoder.close()

        except Exception as exc:
            self.get_logger().warn(
                f'Could not close encoder GPIO cleanly: {exc}'
            )

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
            node.get_logger().info(
                'Stopping ThunderBorg motors and closing driver.'
            )
            node.close_hardware()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()