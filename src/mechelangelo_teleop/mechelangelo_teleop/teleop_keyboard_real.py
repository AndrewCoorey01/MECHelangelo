# #!/usr/bin/env python3

# import os
# import sys
# import time
# import select

# from gpiozero import PWMOutputDevice, DigitalOutputDevice

# if os.name != "nt":
#     import termios
#     import tty

# # Everything up here is what needs to change, Pin definitions, power and speed settings.

# # ============================================================
# # GPIO PIN CONFIGURATION FOR DRI0002 / MD1.3 MOTOR DRIVER
# # ============================================================
# # These are Raspberry Pi BCM GPIO numbers, matching IO labels on the HAT.

# RIGHT_PWM_PIN = 12   # Driver E1
# RIGHT_DIR_PIN = 23   # Driver M1

# LEFT_PWM_PIN = 13    # Driver E2
# LEFT_DIR_PIN = 24    # Driver M2


# # ============================================================
# # SAFETY / CONTROL SETTINGS
# # ============================================================

# # MAX_DUTY = 0.25 #Power control for the motors (25% start for testing) PWM duty cycle range 0.0-1.0.

# # DUTY_STEP = 0.02 #how much each press of the keyboard increases speed
# # TURN_STEP = 0.02 #how much each press of the keyboard increases turn
# # RAMP_STEP = 0.01 #how much the motor speed changes per iteration used to reduce jerk and smooth accerleration
# MAX_DUTY = 1.0 #Power control for the motors (25% start for testing) PWM duty cycle range 0.0-1.0.

# DUTY_STEP = 0.5 #how much each press of the keyboard increases speed
# TURN_STEP = 0.5 #how much each press of the keyboard increases turn
# RAMP_STEP = 0.2 #how much the motor speed changes per iteration used to reduce jerk and smooth accerleration

# MIN_MOVING_DUTY = 0.20 # Minimum duty cycle to overcome static friction and start moving. Adjust based on your motors and load.

# PWM_FREQUENCY = 1000

# # If no key command is received for this long, stop motors.
# COMMAND_TIMEOUT = 5.0

# #terminal printout instructions
# MSG = """
# MECHelangelo Physical Wheel Test
# --------------------------------

# Movement:
#         w
#    a    s    d
#         x

# w : increase forward speed
# x : increase reverse speed
# a : increase left turn
# d : increase right turn

# space or s : stop motors

# CTRL-C : quit

# Important:
# - Start with wheels lifted off the ground.
# - Keep MAX_DUTY low for first test.
# - Have a battery disconnect or emergency stop ready.
# """


# def get_key(settings):
#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

#     if rlist:
#         key = sys.stdin.read(1)
#     else:
#         key = ""

#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key


# def constrain(value, low, high):
#     return max(low, min(high, value))


# def ramp_value(current, target, step):
#     if target > current:
#         return min(target, current + step)
#     if target < current:
#         return max(target, current - step)
#     return current


# def print_status(target_linear, target_turn, left_cmd, right_cmd):
#     print(
#         f"target linear: {target_linear:+.2f} | "
#         f"target turn: {target_turn:+.2f} | "
#         f"left motor: {left_cmd:+.2f} | "
#         f"right motor: {right_cmd:+.2f}"
#     )

# class Motor:
#     def __init__(self, pwm_pin, dir_pin, invert=False):
#         self.pwm = PWMOutputDevice(
#             pwm_pin,
#             frequency=PWM_FREQUENCY,
#             initial_value=0.0
#         )

#         self.direction = DigitalOutputDevice(
#             dir_pin,
#             initial_value=False
#         )

#         self.invert = invert

#     # def set_speed(self, command):
#     #     """
#     #     command:
#     #         + value = forward
#     #         - value = reverse
#     #         0       = stop

#     #     DRI0002 logic:
#     #         Enable pin E = PWM speed
#     #         Direction pin M = LOW/HIGH direction
#     #     """

#     #     command = constrain(command, -1.0, 1.0)

#     #     if self.invert:
#     #         command = -command

#     #     if command > 0.0:
#     #         # Forward
#     #         # Datasheet says M = LOW gives forward.
#     #         self.direction.off()
#     #         self.pwm.value = abs(command)

#     #     elif command < 0.0:
#     #         # Reverse
#     #         # Datasheet says M = HIGH gives back direction.
#     #         self.direction.on()
#     #         self.pwm.value = abs(command)

#     #     else:
#     #         # Stop
#     #         self.pwm.value = 0.0

#     def set_speed(self, command):
#         command = constrain(command, -1.0, 1.0)
    
#         if self.invert:
#             command = -command
    
#         if abs(command) < 0.001:
#             self.pwm.value = 0.0
#             return
    
#         duty = max(abs(command), MIN_MOVING_DUTY)
    
#         if command > 0.0:
#             self.direction.off()
#             self.pwm.value = duty
    
#         else:
#             self.direction.on()
#             self.pwm.value = duty

#     def stop(self):
#         self.pwm.value = 0.0


# def main():
#     if os.name == "nt":
#         print("This script is intended to run on Raspberry Pi Linux.")
#         return

#     settings = termios.tcgetattr(sys.stdin)

#     # If a wheel spins backwards when pressing w, change its invert value.
#     left_motor = Motor(
#         LEFT_PWM_PIN,
#         LEFT_DIR_PIN,
#         invert=False
#     )
    
#     right_motor = Motor(
#         RIGHT_PWM_PIN,
#         RIGHT_DIR_PIN,
#         invert=False
#     )

#     target_linear = 0.0
#     target_turn = 0.0

#     control_linear = 0.0
#     control_turn = 0.0

#     last_command_time = time.time()

#     print(MSG)

#     try:
#         while True:
#             key = get_key(settings)

#             if key == "w":
#                 target_linear = constrain(
#                     target_linear + DUTY_STEP,
#                     -MAX_DUTY,
#                     MAX_DUTY
#                 )
#                 last_command_time = time.time()

#             elif key == "x":
#                 target_linear = constrain(
#                     target_linear - DUTY_STEP,
#                     -MAX_DUTY,
#                     MAX_DUTY
#                 )
#                 last_command_time = time.time()

#             elif key == "a":
#                 target_turn = constrain(
#                     target_turn + TURN_STEP,
#                     -MAX_DUTY,
#                     MAX_DUTY
#                 )
#                 last_command_time = time.time()

#             elif key == "d":
#                 target_turn = constrain(
#                     target_turn - TURN_STEP,
#                     -MAX_DUTY,
#                     MAX_DUTY
#                 )
#                 last_command_time = time.time()

#             elif key == " " or key == "s":
#                 target_linear = 0.0
#                 target_turn = 0.0
#                 control_linear = 0.0
#                 control_turn = 0.0
#                 left_motor.stop()
#                 right_motor.stop()
#                 last_command_time = time.time()
#                 print("STOP")

#             elif key == "\x03":
#                 break

#             # Safety timeout
#             if time.time() - last_command_time > COMMAND_TIMEOUT:
#                 target_linear = 0.0
#                 target_turn = 0.0

#             control_linear = ramp_value(
#                 control_linear,
#                 target_linear,
#                 RAMP_STEP
#             )

#             control_turn = ramp_value(
#                 control_turn,
#                 target_turn,
#                 RAMP_STEP
#             )

#             # Differential drive mixing
#             left_cmd = control_linear - control_turn
#             right_cmd = control_linear + control_turn

#             left_cmd = constrain(left_cmd, -MAX_DUTY, MAX_DUTY)
#             right_cmd = constrain(right_cmd, -MAX_DUTY, MAX_DUTY)

#             left_motor.set_speed(left_cmd)
#             right_motor.set_speed(right_cmd)

#             if key in ["w", "x", "a", "d", " ", "s"]:
#                 print_status(
#                     target_linear,
#                     target_turn,
#                     left_cmd,
#                     right_cmd
#                 )

#             time.sleep(0.02)

#     except Exception as e:
#         print(f"Error: {e}")

#     finally:
#         left_motor.stop()
#         right_motor.stop()

#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#         print("\nMotors stopped. Exiting safely.")


# if __name__ == "__main__":
#     main()

##Code with encoder feedback and closed-loop speed control.


#!/usr/bin/env python3
"""
MECHelangelo real-wheel teleop with quadrature encoder closed-loop speed control.

This script directly drives the physical motors from keyboard input. It reads each
wheel encoder, estimates signed wheel speed in encoder ticks/second, and adjusts
PWM duty using a small PI loop so the left and right wheels track the requested
speed instead of blindly receiving the same duty cycle.

IMPORTANT GPIO NOTES
- Pin numbers are BCM GPIO numbers, not physical header pin numbers.
- On most screw-terminal Raspberry Pi HATs, labels like IO12 / GPIO12 mean BCM 12.
- The encoder outputs must not exceed 3.3 V on Raspberry Pi GPIO pins. Power the
  Parallax encoder boards from 3.3 V, or use level shifting if powered from 5 V.
"""

import os
import sys
import time
import select
import threading

# from gpiozero import PWMOutputDevice, DigitalOutputDevice, InputDevice
from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice

if os.name != "nt":
    import termios
    import tty


# ============================================================
# GPIO PIN CONFIGURATION
# ============================================================
# These are Raspberry Pi BCM GPIO numbers.
# Match them to the IO/GPIO labels on your screw-terminal HAT.

# DRI0002 / MD1.3 motor driver pins
RIGHT_PWM_PIN = 12   # Driver E1, PWM speed control
RIGHT_DIR_PIN = 23   # Driver M1, direction

LEFT_PWM_PIN = 13    # Driver E2, PWM speed control
LEFT_DIR_PIN = 24    # Driver M2, direction

# Quadrature encoder pins
# Each encoder board has two white signal wires: channel A and channel B.
# Change these to match where you connect the two white wires for each wheel.
LEFT_ENC_A_PIN = 5
LEFT_ENC_B_PIN = 6
# RIGHT_ENC_A_PIN = 16
# RIGHT_ENC_B_PIN = 20
RIGHT_ENC_A_PIN = 20
RIGHT_ENC_B_PIN = 16

# ============================================================
# MOTOR / ENCODER SETTINGS
# ============================================================
PWM_FREQUENCY = 1000

# Start conservative while tuning. Increase only after the robot is safe on blocks.
MAX_DUTY = 0.60
MIN_MOVING_DUTY = 0.18

# The Parallax kit gives 144 positions/counts per full tyre rotation if using
# both encoder channels with 4x quadrature decoding.
ENCODER_TICKS_PER_REV = 144.0

# Product guide states approx. 100 rpm no-load at 12 V. Loaded speed will be lower.
MAX_WHEEL_RPM = 100.0
MAX_TICKS_PER_SEC = ENCODER_TICKS_PER_REV * MAX_WHEEL_RPM / 60.0

# Keyboard commands are normalized wheel-speed commands, not raw PWM duty.
SPEED_STEP = 0.10 ##how much each press of the keyboard increases forward/reverse speed
TURN_STEP = 0.10 ##how much each press of the keyboard increases turn
RAMP_STEP = 0.03 ##how much the motor speed changes per iteration used to reduce jerk and smooth acceleration. Increase for more responsiveness, decrease for smoother but slower response.

CONTROL_PERIOD = 0.05       # 20 Hz closed-loop update
COMMAND_TIMEOUT = 2.0       # Stop if keyboard input is inactive this long
STATUS_PERIOD = 0.25        # Print status while moving

# PI speed controller gains.
# If it reacts too weakly, increase KP slightly. If it oscillates, lower KP/KI.
KP = 0.0015                 # duty per ticks/sec error
KI = 0.00035                # duty per accumulated ticks/sec error
INTEGRAL_LIMIT = 250.0

# Set these after the first encoder test.
# If pressing w makes the wheel move forward but measured ticks/sec is negative,
# flip that wheel's encoder invert value.
LEFT_MOTOR_INVERT = False ##if pressing w makes the wheel spin backwards, set this to True to flip the motor direction.
RIGHT_MOTOR_INVERT = False
LEFT_ENCODER_INVERT = False ##if wheel goes forward but ticks/sec is negative, set this to True to flip the sign of the encoder reading for that wheel.
RIGHT_ENCODER_INVERT = False

# Parallax encoder outputs are active driven when powered correctly, so pull-ups
# are usually not needed. Set True only if your signal floats when disconnected.
ENCODER_PULL_UP = False


MSG = """
MECHelangelo Physical Wheel Test - encoder closed loop
------------------------------------------------------

Movement:
        w
   a    s    d
        x

w : increase forward speed
x : increase reverse speed
a : increase left turn
d : increase right turn

space or s : stop motors
CTRL-C     : quit

Start with the wheels lifted off the ground.
"""


def get_key(settings, timeout=CONTROL_PERIOD):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)

    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def constrain(value, low, high):
    return max(low, min(high, value))


def ramp_value(current, target, step):
    if target > current:
        return min(target, current + step)
    if target < current:
        return max(target, current - step)
    return current


def sign(value):
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


class QuadratureEncoder:
    """4x quadrature decoder using two GPIO input pins."""

    # Transition lookup table for a standard quadrature sequence.
    # If your encoder sign is backwards, use invert=True instead of changing this.
    _FORWARD_TRANSITIONS = {0b0001, 0b0111, 0b1110, 0b1000}
    _REVERSE_TRANSITIONS = {0b0010, 0b1011, 0b1101, 0b0100}

    def __init__(self, pin_a, pin_b, invert=False, name="encoder"):
        self.name = name
        # self.a = InputDevice(pin_a, pull_up=ENCODER_PULL_UP)
        # self.b = InputDevice(pin_b, pull_up=ENCODER_PULL_UP)
        self.a = DigitalInputDevice(pin_a, pull_up=ENCODER_PULL_UP)
        self.b = DigitalInputDevice(pin_b, pull_up=ENCODER_PULL_UP)
        self.invert = invert
        self.count = 0
        self.lock = threading.Lock()

        self.last_state = self._read_state()

        self.a.when_activated = self._edge
        self.a.when_deactivated = self._edge
        self.b.when_activated = self._edge
        self.b.when_deactivated = self._edge

    def _read_state(self):
        return (int(self.a.value) << 1) | int(self.b.value)

    def _edge(self):
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
    def __init__(self, pwm_pin, dir_pin, invert=False, name="motor"):
        self.name = name
        self.pwm = PWMOutputDevice(
            pwm_pin,
            frequency=PWM_FREQUENCY,
            initial_value=0.0,
        )
        self.direction = DigitalOutputDevice(dir_pin, initial_value=False)
        self.invert = invert
        self.last_command = 0.0

    def set_duty_signed(self, command):
        """
        command range: -1.0 to +1.0
        + = forward, - = reverse, 0 = stop.
        DRI0002 logic: E pin is PWM speed; M pin LOW/HIGH controls direction.
        """
        command = constrain(command, -1.0, 1.0)

        if self.invert:
            command = -command

        self.last_command = command

        if abs(command) < 0.001:
            self.pwm.value = 0.0
            return

        duty = constrain(abs(command), 0.0, MAX_DUTY)

        if command > 0.0:
            self.direction.off()   # M = LOW, forward for DRI0002
        else:
            self.direction.on()    # M = HIGH, reverse for DRI0002

        self.pwm.value = duty

    def stop(self):
        self.last_command = 0.0
        self.pwm.value = 0.0

    def close(self):
        self.stop()
        self.pwm.close()
        self.direction.close()


class WheelSpeedController:
    """Closed-loop wheel speed controller for one wheel."""

    def __init__(self, motor, encoder, name):
        self.motor = motor
        self.encoder = encoder
        self.name = name
        self.last_count = encoder.get_count()
        self.filtered_ticks_per_sec = 0.0
        self.integral = 0.0
        self.last_target_direction = 0.0

    def update(self, target_ticks_per_sec, dt):
        current_count = self.encoder.get_count()
        delta_count = current_count - self.last_count
        self.last_count = current_count

        raw_ticks_per_sec = delta_count / dt if dt > 0.0 else 0.0

        # Light smoothing makes the PI loop less jumpy at low speeds.
        alpha = 0.35
        self.filtered_ticks_per_sec = (
            alpha * raw_ticks_per_sec
            + (1.0 - alpha) * self.filtered_ticks_per_sec
        )

        target_direction = sign(target_ticks_per_sec)

        if target_direction == 0.0:
            self.integral = 0.0
            self.last_target_direction = 0.0
            self.motor.stop()
            return 0.0, self.filtered_ticks_per_sec, current_count

        if target_direction != self.last_target_direction:
            self.integral = 0.0
            self.last_target_direction = target_direction

        measured_along_target = self.filtered_ticks_per_sec * target_direction
        speed_error = abs(target_ticks_per_sec) - measured_along_target

        self.integral = constrain(
            self.integral + speed_error * dt,
            -INTEGRAL_LIMIT,
            INTEGRAL_LIMIT,
        )

        feedforward = (abs(target_ticks_per_sec) / MAX_TICKS_PER_SEC) * MAX_DUTY
        correction = KP * speed_error + KI * self.integral
        duty = feedforward + correction

        duty = constrain(duty, MIN_MOVING_DUTY, MAX_DUTY)
        signed_duty = target_direction * duty

        self.motor.set_duty_signed(signed_duty)
        return signed_duty, self.filtered_ticks_per_sec, current_count

    def stop(self):
        self.integral = 0.0
        self.motor.stop()


def print_status(target_linear, target_turn, left_target, right_target,
                 left_actual, right_actual, left_duty, right_duty,
                 left_count, right_count):
    left_rpm = (left_actual / ENCODER_TICKS_PER_REV) * 60.0
    right_rpm = (right_actual / ENCODER_TICKS_PER_REV) * 60.0

    print(
        f"lin {target_linear:+.2f} turn {target_turn:+.2f} | "
        f"L tgt {left_target:+6.1f} t/s act {left_actual:+6.1f} t/s "
        f"({left_rpm:+5.1f} rpm) duty {left_duty:+.2f} cnt {left_count:+7d} | "
        f"R tgt {right_target:+6.1f} t/s act {right_actual:+6.1f} t/s "
        f"({right_rpm:+5.1f} rpm) duty {right_duty:+.2f} cnt {right_count:+7d}"
    )


def main():
    if os.name == "nt":
        print("This script is intended to run on Raspberry Pi Linux.")
        return

    settings = termios.tcgetattr(sys.stdin)

    left_motor = Motor(
        LEFT_PWM_PIN,
        LEFT_DIR_PIN,
        invert=LEFT_MOTOR_INVERT,
        name="left motor",
    )
    right_motor = Motor(
        RIGHT_PWM_PIN,
        RIGHT_DIR_PIN,
        invert=RIGHT_MOTOR_INVERT,
        name="right motor",
    )

    left_encoder = QuadratureEncoder(
        LEFT_ENC_A_PIN,
        LEFT_ENC_B_PIN,
        invert=LEFT_ENCODER_INVERT,
        name="left encoder",
    )
    right_encoder = QuadratureEncoder(
        RIGHT_ENC_A_PIN,
        RIGHT_ENC_B_PIN,
        invert=RIGHT_ENCODER_INVERT,
        name="right encoder",
    )

    left_controller = WheelSpeedController(left_motor, left_encoder, "left")
    right_controller = WheelSpeedController(right_motor, right_encoder, "right")

    target_linear = 0.0
    target_turn = 0.0
    control_linear = 0.0
    control_turn = 0.0

    last_command_time = time.time()
    last_loop_time = time.time()
    last_status_time = 0.0

    print(MSG)
    print(f"Max target speed: {MAX_TICKS_PER_SEC:.1f} ticks/s per wheel")

    try:
        while True:
            key = get_key(settings)
            now = time.time()
            dt = max(now - last_loop_time, 0.001)
            last_loop_time = now

            if key == "w":
                target_linear = constrain(target_linear + SPEED_STEP, -1.0, 1.0)
                last_command_time = now
            elif key == "x":
                target_linear = constrain(target_linear - SPEED_STEP, -1.0, 1.0)
                last_command_time = now
            elif key == "a":
                target_turn = constrain(target_turn + TURN_STEP, -1.0, 1.0)
                last_command_time = now
            elif key == "d":
                target_turn = constrain(target_turn - TURN_STEP, -1.0, 1.0)
                last_command_time = now
            elif key == " " or key == "s":
                target_linear = 0.0
                target_turn = 0.0
                control_linear = 0.0
                control_turn = 0.0
                left_controller.stop()
                right_controller.stop()
                last_command_time = now
                print("STOP")
            elif key == "\x03":
                break

            if now - last_command_time > COMMAND_TIMEOUT:
                target_linear = 0.0
                target_turn = 0.0

            control_linear = ramp_value(control_linear, target_linear, RAMP_STEP)
            control_turn = ramp_value(control_turn, target_turn, RAMP_STEP)

            # Differential-drive mixing in normalized wheel-speed space.
            left_fraction = constrain(control_linear - control_turn, -1.0, 1.0)
            right_fraction = constrain(control_linear + control_turn, -1.0, 1.0)

            left_target_tps = left_fraction * MAX_TICKS_PER_SEC
            right_target_tps = right_fraction * MAX_TICKS_PER_SEC

            left_duty, left_actual_tps, left_count = left_controller.update(left_target_tps, dt)
            right_duty, right_actual_tps, right_count = right_controller.update(right_target_tps, dt)

            if key in ["w", "x", "a", "d", " ", "s"] or now - last_status_time > STATUS_PERIOD:
                if abs(target_linear) > 0.0 or abs(target_turn) > 0.0 or key in [" ", "s"]:
                    print_status(
                        target_linear,
                        target_turn,
                        left_target_tps,
                        right_target_tps,
                        left_actual_tps,
                        right_actual_tps,
                        left_duty,
                        right_duty,
                        left_count,
                        right_count,
                    )
                last_status_time = now

    except Exception as e:
        print(f"Error: {e}")

    finally:
        left_controller.stop()
        right_controller.stop()
        left_encoder.close()
        right_encoder.close()
        left_motor.close()
        right_motor.close()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nMotors stopped. Exiting safely.")


if __name__ == "__main__":
    main()