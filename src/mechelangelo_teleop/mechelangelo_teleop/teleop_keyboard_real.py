#!/usr/bin/env python3
"""
MECHelangelo real robot keyboard teleop for MakerStore 24V 7A dual motor driver.

Driver type:
    MakerStore 24V 7A Dual Channel DC Motor Driver / XY-160D style board.

Control interface:
    LEFT MOTOR CHANNEL:
        ENA = PWM speed
        IN1 = direction input
        IN2 = direction input

    RIGHT MOTOR CHANNEL:
        ENB = PWM speed
        IN3 = direction input
        IN4 = direction input

Important wiring notes:
    - GPIO numbers below are BCM GPIO numbers, not physical header pin numbers.
    - On most Raspberry Pi screw-terminal HATs, labels like IO12 mean BCM GPIO12.
    - Pi GND must connect to motor driver GND.
    - Motor battery negative must also share GND with the Pi/motor driver logic ground.
    - If the board has ENA/ENB jumpers installed, remove them if you want Pi PWM speed control.
    - Do not power the motors from the Raspberry Pi 5V pin.

Keyboard controls:
    w : increase forward speed
    x : increase reverse speed
    a : increase left turn
    d : increase right turn
    r : clear turn command only
    f : clear forward/reverse command only
    s or space : stop
    q or Ctrl-C : quit

This script uses direct open-loop PWM for motor control. Encoders are read and printed for
basic debugging, but they are not yet used for closed-loop speed control.
"""

from __future__ import annotations

import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice


# =============================================================================
# GPIO PIN CONFIGURATION - EDIT THIS SECTION IF YOU CHANGE WIRING
# =============================================================================
# BCM GPIO numbering, not physical header numbering.
# Example: screw terminal labelled IO12 usually means BCM GPIO12.

# Left motor connected to driver OUT1 / OUT2
LEFT_PWM_PIN = 21      # Pi GPIO21 -> driver ENA
LEFT_IN1_PIN = 13      # Pi GPIO13 -> driver IN1
LEFT_IN2_PIN = 26      # Pi GPIO26 -> driver IN2

# Right motor connected to driver OUT3 / OUT4
RIGHT_PWM_PIN = 27     # Pi GPIO27 -> driver ENB
RIGHT_IN1_PIN = 4     # Pi GPIO4 -> driver IN3
RIGHT_IN2_PIN = 23     # Pi GPIO23 -> driver IN4

# Encoder input pins. These are only used for debug feedback in this script.
LEFT_ENC_A_PIN = 6
LEFT_ENC_B_PIN = 5
RIGHT_ENC_A_PIN = 20
RIGHT_ENC_B_PIN = 16


# =============================================================================
# MOTOR DRIVER SETTINGS
# =============================================================================

PWM_FREQUENCY_HZ = 1000

# Absolute safety cap. Keep at or below 1.0.
MAX_DUTY = 1.00

# Normal command scaling.
MAX_LINEAR_DUTY = 0.85
MAX_TURN_DUTY = 0.65

# Minimum duty to overcome motor static friction.
# If the robot jerks too aggressively at low speed, reduce this.
# If the motors hum but do not move at low speed, increase this slightly.
MIN_MOVING_DUTY = 0.18

# Left/right trim for straight-line correction.
# If the robot drifts left, reduce RIGHT_TRIM slightly or increase LEFT_TRIM slightly.
# If the robot drifts right, reduce LEFT_TRIM slightly or increase RIGHT_TRIM slightly.
LEFT_TRIM = 1.00
RIGHT_TRIM = 1.00

# Flip these if a wheel spins backwards for a forward command.
LEFT_MOTOR_INVERT = True
RIGHT_MOTOR_INVERT = True

# If encoders count backwards relative to forward motor motion, flip these.
LEFT_ENCODER_INVERT = False
RIGHT_ENCODER_INVERT = False

# Most push-pull encoder outputs should use pull_up=False.
# If your encoder signal floats when disconnected or counts randomly, try True.
ENCODER_PULL_UP = False

# Encoder ticks per wheel revolution. This is only used to estimate RPM in debug output.
# Change this once the exact encoder resolution and gearbox relationship are confirmed.
ENCODER_TICKS_PER_REV = 144.0


# =============================================================================
# TELEOP SETTINGS
# =============================================================================

SPEED_STEP = 0.10          # w/x increment
TURN_STEP = 0.08           # a/d increment
RAMP_STEP = 0.04           # command smoothing per loop
CONTROL_PERIOD = 0.05      # seconds, 20 Hz
STATUS_PERIOD = 0.25       # seconds
COMMAND_TIMEOUT = 20.0     # seconds without keypress before target command returns to zero


HELP_TEXT = """
MECHelangelo Real Teleop - MakerStore 24V 7A Driver
---------------------------------------------------

Controls:
        w
   a    s    d
        x

w : increase forward speed
x : increase reverse speed
a : increase left turn
d : increase right turn
r : clear turn only
f : clear forward/reverse only
s or space : stop motors
q or Ctrl-C : quit

Start with the robot lifted off the ground.
"""


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def ramp(current: float, target: float, step: float) -> float:
    if current < target:
        return min(current + step, target)
    if current > target:
        return max(current - step, target)
    return current


def apply_min_duty(command: float) -> float:
    """Apply a minimum non-zero PWM duty so low commands still move the motor."""
    if abs(command) < 1e-4:
        return 0.0

    direction = sign(command)
    duty = max(abs(command), MIN_MOVING_DUTY)
    duty = clamp(duty, 0.0, MAX_DUTY)
    return direction * duty


def read_key(timeout: float) -> str:
    """Read one keyboard character without blocking forever."""
    readable, _, _ = select.select([sys.stdin], [], [], timeout)
    if readable:
        return sys.stdin.read(1)
    return ""


def mix_differential_drive(linear: float, turn: float) -> tuple[float, float]:
    """
    Convert linear and turn commands into left and right signed motor duties.

    linear:
        +1 = forward
        -1 = reverse

    turn:
        +1 = turn left
        -1 = turn right
    """
    linear_cmd = linear * MAX_LINEAR_DUTY
    turn_cmd = turn * MAX_TURN_DUTY

    # Differential drive mixing.
    # For a left turn, left wheel slows/reverses and right wheel speeds up.
    left = linear_cmd - turn_cmd
    right = linear_cmd + turn_cmd

    left *= LEFT_TRIM
    right *= RIGHT_TRIM

    left = clamp(left, -MAX_DUTY, MAX_DUTY)
    right = clamp(right, -MAX_DUTY, MAX_DUTY)

    left = apply_min_duty(left)
    right = apply_min_duty(right)

    return left, right


# =============================================================================
# HARDWARE CLASSES
# =============================================================================

class MotorChannel:
    """One motor channel on an EN + IN1 + IN2 style H-bridge driver."""

    def __init__(self, pwm_pin: int, in1_pin: int, in2_pin: int, *, invert: bool, name: str):
        self.name = name
        self.invert = invert
        self.last_command = 0.0

        self.pwm = PWMOutputDevice(
            pwm_pin,
            frequency=PWM_FREQUENCY_HZ,
            initial_value=0.0,
        )
        self.in1 = DigitalOutputDevice(in1_pin, initial_value=False)
        self.in2 = DigitalOutputDevice(in2_pin, initial_value=False)

        self.stop()

    def set_signed_duty(self, command: float) -> None:
        """
        Set motor command.

        command > 0: forward
        command < 0: reverse
        command = 0: coast/stop
        """
        command = clamp(command, -MAX_DUTY, MAX_DUTY)

        if self.invert:
            command = -command

        self.last_command = command

        if abs(command) < 1e-4:
            self.stop()
            return

        duty = clamp(abs(command), 0.0, MAX_DUTY)

        if command > 0.0:
            self.in1.on()
            self.in2.off()
        else:
            self.in1.off()
            self.in2.on()

        self.pwm.value = duty

    def stop(self) -> None:
        self.last_command = 0.0
        self.pwm.value = 0.0
        self.in1.off()
        self.in2.off()

    def brake(self) -> None:
        """Active brake. Use briefly only if you specifically want braking."""
        self.last_command = 0.0
        self.pwm.value = 1.0
        self.in1.on()
        self.in2.on()

    def close(self) -> None:
        self.stop()
        self.pwm.close()
        self.in1.close()
        self.in2.close()


class PolledQuadratureEncoder:
    """
    Simple polled quadrature decoder.

    This avoids gpiozero callback/event setup issues and is good enough for debugging.
    It can miss counts at high motor speeds, so do not use this exact class for final
    closed-loop velocity control without testing the count accuracy.
    """

    FORWARD_TRANSITIONS = {0b0001, 0b0111, 0b1110, 0b1000}
    REVERSE_TRANSITIONS = {0b0010, 0b1011, 0b1101, 0b0100}

    def __init__(self, pin_a: int, pin_b: int, *, invert: bool, name: str):
        self.name = name
        self.invert = invert
        self.a = DigitalInputDevice(pin_a, pull_up=ENCODER_PULL_UP)
        self.b = DigitalInputDevice(pin_b, pull_up=ENCODER_PULL_UP)
        self.count = 0
        self.last_state = self._read_state()

    def _read_state(self) -> int:
        return (int(self.a.value) << 1) | int(self.b.value)

    def update(self) -> int:
        new_state = self._read_state()
        transition = (self.last_state << 2) | new_state

        delta = 0
        if transition in self.FORWARD_TRANSITIONS:
            delta = 1
        elif transition in self.REVERSE_TRANSITIONS:
            delta = -1

        if self.invert:
            delta = -delta

        self.count += delta
        self.last_state = new_state
        return self.count

    def get_count(self) -> int:
        return self.count

    def close(self) -> None:
        self.a.close()
        self.b.close()


@dataclass
class TeleopState:
    target_linear: float = 0.0
    target_turn: float = 0.0
    control_linear: float = 0.0
    control_turn: float = 0.0


# =============================================================================
# STATUS / DEBUG OUTPUT
# =============================================================================

def print_status(
    state: TeleopState,
    left_cmd: float,
    right_cmd: float,
    left_count: int,
    right_count: int,
    prev_left_count: int,
    prev_right_count: int,
    dt: float,
) -> None:
    left_delta = left_count - prev_left_count
    right_delta = right_count - prev_right_count

    left_tps = left_delta / dt if dt > 0.0 else 0.0
    right_tps = right_delta / dt if dt > 0.0 else 0.0

    left_rpm = (left_tps / ENCODER_TICKS_PER_REV) * 60.0
    right_rpm = (right_tps / ENCODER_TICKS_PER_REV) * 60.0

    print(
        f"target lin={state.target_linear:+.2f} turn={state.target_turn:+.2f} | "
        f"control lin={state.control_linear:+.2f} turn={state.control_turn:+.2f} | "
        f"L duty={left_cmd:+.2f} count={left_count:+7d} "
        f"speed={left_tps:+7.1f} ticks/s rpm={left_rpm:+6.1f} | "
        f"R duty={right_cmd:+.2f} count={right_count:+7d} "
        f"speed={right_tps:+7.1f} ticks/s rpm={right_rpm:+6.1f}"
    )


# =============================================================================
# MAIN TELEOP LOOP
# =============================================================================

def main() -> int:
    if os.name == "nt":
        print("This script must run on the Raspberry Pi, not Windows.")
        return 1

    terminal_settings = termios.tcgetattr(sys.stdin)

    left_motor: MotorChannel | None = None
    right_motor: MotorChannel | None = None
    left_encoder: PolledQuadratureEncoder | None = None
    right_encoder: PolledQuadratureEncoder | None = None

    try:
        left_motor = MotorChannel(
            LEFT_PWM_PIN,
            LEFT_IN1_PIN,
            LEFT_IN2_PIN,
            invert=LEFT_MOTOR_INVERT,
            name="left motor",
        )
        right_motor = MotorChannel(
            RIGHT_PWM_PIN,
            RIGHT_IN1_PIN,
            RIGHT_IN2_PIN,
            invert=RIGHT_MOTOR_INVERT,
            name="right motor",
        )

        left_encoder = PolledQuadratureEncoder(
            LEFT_ENC_A_PIN,
            LEFT_ENC_B_PIN,
            invert=LEFT_ENCODER_INVERT,
            name="left encoder",
        )
        right_encoder = PolledQuadratureEncoder(
            RIGHT_ENC_A_PIN,
            RIGHT_ENC_B_PIN,
            invert=RIGHT_ENCODER_INVERT,
            name="right encoder",
        )

        tty.setcbreak(sys.stdin.fileno())

        state = TeleopState()
        last_key_time = time.time()
        last_status_time = time.time()
        last_status_count_time = time.time()
        prev_left_count = left_encoder.get_count()
        prev_right_count = right_encoder.get_count()

        print(HELP_TEXT)
        print("GPIO mapping:")
        print(f"  LEFT : ENA GPIO{LEFT_PWM_PIN}, IN1 GPIO{LEFT_IN1_PIN}, IN2 GPIO{LEFT_IN2_PIN}")
        print(f"  RIGHT: ENB GPIO{RIGHT_PWM_PIN}, IN3 GPIO{RIGHT_IN1_PIN}, IN4 GPIO{RIGHT_IN2_PIN}")
        print(f"  LEFT encoder : A GPIO{LEFT_ENC_A_PIN}, B GPIO{LEFT_ENC_B_PIN}")
        print(f"  RIGHT encoder: A GPIO{RIGHT_ENC_A_PIN}, B GPIO{RIGHT_ENC_B_PIN}")
        print("\nReady. Press w/a/s/d/x, space to stop, q to quit.\n")

        while True:
            loop_start = time.time()

            key = read_key(CONTROL_PERIOD)
            now = time.time()

            # Always poll encoders once per loop.
            left_encoder.update()
            right_encoder.update()

            if key:
                last_key_time = now

            if key == "w":
                state.target_linear = clamp(state.target_linear + SPEED_STEP, -1.0, 1.0)
            elif key == "x":
                state.target_linear = clamp(state.target_linear - SPEED_STEP, -1.0, 1.0)
            elif key == "a":
                state.target_turn = clamp(state.target_turn + TURN_STEP, -1.0, 1.0)
            elif key == "d":
                state.target_turn = clamp(state.target_turn - TURN_STEP, -1.0, 1.0)
            elif key == "r":
                state.target_turn = 0.0
                state.control_turn = 0.0
                print("Turn command cleared")
            elif key == "f":
                state.target_linear = 0.0
                state.control_linear = 0.0
                print("Linear command cleared")
            elif key in ("s", " "):
                state = TeleopState()
                left_motor.stop()
                right_motor.stop()
                print("STOP")
            elif key in ("q", "\x03"):
                break

            # Dead-man timeout: after no keypresses for a while, ramp down to zero.
            if now - last_key_time > COMMAND_TIMEOUT:
                state.target_linear = 0.0
                state.target_turn = 0.0

            # Smooth commands.
            state.control_linear = ramp(state.control_linear, state.target_linear, RAMP_STEP)
            state.control_turn = ramp(state.control_turn, state.target_turn, RAMP_STEP)

            # Mix and apply motor commands.
            left_cmd, right_cmd = mix_differential_drive(state.control_linear, state.control_turn)
            left_motor.set_signed_duty(left_cmd)
            right_motor.set_signed_duty(right_cmd)

            moving = (
                abs(state.target_linear) > 1e-4
                or abs(state.target_turn) > 1e-4
                or abs(state.control_linear) > 1e-4
                or abs(state.control_turn) > 1e-4
            )

            if moving and now - last_status_time >= STATUS_PERIOD:
                left_count = left_encoder.get_count()
                right_count = right_encoder.get_count()
                dt_status = max(now - last_status_count_time, 1e-3)

                print_status(
                    state,
                    left_cmd,
                    right_cmd,
                    left_count,
                    right_count,
                    prev_left_count,
                    prev_right_count,
                    dt_status,
                )

                prev_left_count = left_count
                prev_right_count = right_count
                last_status_count_time = now
                last_status_time = now

            # Keep the loop from running faster than intended if a key was pressed immediately.
            elapsed = time.time() - loop_start
            remaining = CONTROL_PERIOD - elapsed
            if remaining > 0.0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\nERROR: {exc}")
        return 1
    finally:
        if left_motor is not None:
            left_motor.stop()
        if right_motor is not None:
            right_motor.stop()
        if left_encoder is not None:
            left_encoder.close()
        if right_encoder is not None:
            right_encoder.close()
        if left_motor is not None:
            left_motor.close()
        if right_motor is not None:
            right_motor.close()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, terminal_settings)
        print("\nMotors stopped. GPIO released.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


# #!/usr/bin/env python3
# """
# MECHelangelo real-wheel teleop, direct motor PWM version.

# This script directly drives the physical left and right motors from keyboard input.

# Control behaviour is intentionally similar to TurtleBot teleop:
# - w/x adjust forward/reverse command
# - a/d adjust turn command
# - pressing w then d keeps the forward command and adds rotation
# - pressing s or space stops everything

# This version uses open-loop PWM duty commands rather than encoder closed-loop speed
# control, because the encoder counts have been reading as zero during testing.

# GPIO NOTES
# - Pin numbers are BCM GPIO numbers, not physical header pin numbers.
# - On most screw-terminal Raspberry Pi HATs, labels like IO12/GPIO12 mean BCM 12.
# - Encoder inputs are still read and printed, but are not used for control yet.
# """

# import os
# import sys
# import time
# import select
# import threading

# from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice

# if os.name != "nt":
#     import termios
#     import tty


# # ============================================================
# # GPIO PIN CONFIGURATION
# # ============================================================
# # These are Raspberry Pi BCM GPIO numbers.

# # DRI0002 / MD1.3 motor driver pins
# RIGHT_PWM_PIN = 12   # Driver E1, PWM speed control
# RIGHT_DIR_PIN = 23   # Driver M1, direction

# LEFT_PWM_PIN = 13    # Driver E2, PWM speed control
# LEFT_DIR_PIN = 24    # Driver M2, direction

# # Quadrature encoder pins
# # These are still read for debugging, but not used for motor control yet.
# LEFT_ENC_A_PIN = 6
# LEFT_ENC_B_PIN = 5

# RIGHT_ENC_A_PIN = 20
# RIGHT_ENC_B_PIN = 16


# # ============================================================
# # MOTOR SETTINGS
# # ============================================================

# PWM_FREQUENCY = 1000

# # Hard electrical limit. Never set above 1.0.
# MAX_DUTY = 1.0

# # Behaviour limits.
# # Forward/back works best for your robot at full duty.
# MAX_LINEAR_DUTY = 1.00

# # Turning works best around 0.7 to 0.8.
# MAX_TURN_DUTY = 0.75

# # Minimum duty to overcome static friction.
# # This is only applied when a command is non-zero.
# MIN_MOVING_DUTY = 0.18

# # Optional trim if the robot drifts.
# # If robot drifts left, reduce RIGHT_TRIM slightly.
# # If robot drifts right, reduce LEFT_TRIM slightly.
# LEFT_TRIM = 1.00
# RIGHT_TRIM = 1.00


# # ============================================================
# # TELEOP SETTINGS
# # ============================================================

# # These are normalized command steps, not direct PWM values.
# # target_linear and target_turn both range from -1.0 to +1.0.
# SPEED_STEP = 0.10
# TURN_STEP = 0.05

# # Smoothing per loop.
# # Increase for snappier response, decrease for smoother response.
# RAMP_STEP = 0.03

# CONTROL_PERIOD = 0.05        # 20 Hz loop
# COMMAND_TIMEOUT = 20.0       # Stop if no keyboard input for this long
# STATUS_PERIOD = 0.25         # Print status while moving


# # ============================================================
# # ENCODER SETTINGS
# # ============================================================

# ENCODER_TICKS_PER_REV = 144.0

# LEFT_ENCODER_INVERT = False
# RIGHT_ENCODER_INVERT = False

# # Try True later if encoder input floats.
# ENCODER_PULL_UP = False


# # ============================================================
# # MOTOR DIRECTION SETTINGS
# # ============================================================

# # You found forward/reverse were inverted, so these are True.
# LEFT_MOTOR_INVERT = True
# RIGHT_MOTOR_INVERT = True


# MSG = """
# MECHelangelo Physical Teleop - direct PWM
# -----------------------------------------

# Movement:
#         w
#    a    s    d
#         x

# w : increase forward speed
# x : increase reverse speed
# a : increase left turn
# d : increase right turn

# Important:
# - w then d = forward while turning right
# - w then a = forward while turning left
# - x then d = reverse while turning right
# - x then a = reverse while turning left

# r : clear turn, keep forward/reverse speed
# f : clear forward/reverse speed, keep turn
# space or s : stop motors
# CTRL-C     : quit

# Start with the wheels lifted off the ground for first test.
# """


# # ============================================================
# # HELPER FUNCTIONS
# # ============================================================

# def get_key(settings, timeout=CONTROL_PERIOD):
#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], timeout)

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


# def sign(value):
#     if value > 0.0:
#         return 1.0
#     if value < 0.0:
#         return -1.0
#     return 0.0


# def apply_minimum_duty(command):
#     """
#     Applies MIN_MOVING_DUTY to non-zero motor commands.

#     Example:
#     command = +0.05 becomes +0.18
#     command = -0.05 becomes -0.18
#     command =  0.00 stays  0.00
#     """
#     if abs(command) < 0.001:
#         return 0.0

#     direction = sign(command)
#     duty = max(abs(command), MIN_MOVING_DUTY)
#     duty = constrain(duty, 0.0, MAX_DUTY)

#     return direction * duty


# def mix_drive_commands(linear, turn):
#     """
#     Convert normalized linear and turn commands into left/right motor duty.

#     linear:
#         +1.0 = full forward
#         -1.0 = full reverse

#     turn:
#         +1.0 = left turn
#         -1.0 = right turn

#     This keeps TurtleBot-style behaviour:
#     - linear and angular commands are independent
#     - pressing w then d keeps forward speed and adds right turn
#     """

#     linear_cmd = linear * MAX_LINEAR_DUTY
#     turn_cmd = turn * MAX_TURN_DUTY

#     # Differential drive mixing.
#     # Positive turn means turn left:
#     # left wheel slows/reverses, right wheel speeds up.
#     left_cmd = linear_cmd - turn_cmd
#     right_cmd = linear_cmd + turn_cmd

#     # Apply trim before final constraining.
#     left_cmd *= LEFT_TRIM
#     right_cmd *= RIGHT_TRIM

#     # Keep inside valid duty range.
#     left_cmd = constrain(left_cmd, -MAX_DUTY, MAX_DUTY)
#     right_cmd = constrain(right_cmd, -MAX_DUTY, MAX_DUTY)

#     # Make very small non-zero commands strong enough to move the robot.
#     left_cmd = apply_minimum_duty(left_cmd)
#     right_cmd = apply_minimum_duty(right_cmd)

#     return left_cmd, right_cmd


# # ============================================================
# # ENCODER CLASS
# # ============================================================

# class QuadratureEncoder:
#     """4x quadrature decoder using two GPIO input pins."""

#     _FORWARD_TRANSITIONS = {0b0001, 0b0111, 0b1110, 0b1000}
#     _REVERSE_TRANSITIONS = {0b0010, 0b1011, 0b1101, 0b0100}

#     def __init__(self, pin_a, pin_b, invert=False, name="encoder"):
#         self.name = name
#         self.a = DigitalInputDevice(pin_a, pull_up=ENCODER_PULL_UP)
#         self.b = DigitalInputDevice(pin_b, pull_up=ENCODER_PULL_UP)
#         self.invert = invert
#         self.count = 0
#         self.lock = threading.Lock()

#         self.last_state = self._read_state()

#         self.a.when_activated = self._edge
#         self.a.when_deactivated = self._edge
#         self.b.when_activated = self._edge
#         self.b.when_deactivated = self._edge

#     def _read_state(self):
#         return (int(self.a.value) << 1) | int(self.b.value)

#     def _edge(self):
#         new_state = self._read_state()
#         transition = (self.last_state << 2) | new_state

#         delta = 0
#         if transition in self._FORWARD_TRANSITIONS:
#             delta = 1
#         elif transition in self._REVERSE_TRANSITIONS:
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


# # ============================================================
# # MOTOR CLASS
# # ============================================================

# class Motor:
#     def __init__(self, pwm_pin, dir_pin, invert=False, name="motor"):
#         self.name = name
#         self.pwm = PWMOutputDevice(
#             pwm_pin,
#             frequency=PWM_FREQUENCY,
#             initial_value=0.0,
#         )
#         self.direction = DigitalOutputDevice(dir_pin, initial_value=False)
#         self.invert = invert
#         self.last_command = 0.0

#     def set_duty_signed(self, command):
#         """
#         command range:
#             +1.0 = full forward
#             -1.0 = full reverse
#              0.0 = stop

#         DRI0002 / MD1.3:
#             E pin = PWM speed
#             M pin = direction
#         """
#         command = constrain(command, -MAX_DUTY, MAX_DUTY)

#         if self.invert:
#             command = -command

#         self.last_command = command

#         if abs(command) < 0.001:
#             self.pwm.value = 0.0
#             return

#         duty = constrain(abs(command), 0.0, MAX_DUTY)

#         if command > 0.0:
#             self.direction.off()
#         else:
#             self.direction.on()

#         self.pwm.value = duty

#     def stop(self):
#         self.last_command = 0.0
#         self.pwm.value = 0.0

#     def close(self):
#         self.stop()
#         self.pwm.close()
#         self.direction.close()


# # ============================================================
# # STATUS PRINTING
# # ============================================================

# def print_status(target_linear, target_turn,
#                  control_linear, control_turn,
#                  left_cmd, right_cmd,
#                  left_count, right_count,
#                  last_left_count, last_right_count,
#                  dt):
#     left_delta = left_count - last_left_count
#     right_delta = right_count - last_right_count

#     left_tps = left_delta / dt if dt > 0.0 else 0.0
#     right_tps = right_delta / dt if dt > 0.0 else 0.0

#     left_rpm = (left_tps / ENCODER_TICKS_PER_REV) * 60.0
#     right_rpm = (right_tps / ENCODER_TICKS_PER_REV) * 60.0

#     print(
#         f"target lin {target_linear:+.2f} turn {target_turn:+.2f} | "
#         f"control lin {control_linear:+.2f} turn {control_turn:+.2f} | "
#         f"L duty {left_cmd:+.2f} cnt {left_count:+7d} "
#         f"est {left_tps:+6.1f} t/s ({left_rpm:+5.1f} rpm) | "
#         f"R duty {right_cmd:+.2f} cnt {right_count:+7d} "
#         f"est {right_tps:+6.1f} t/s ({right_rpm:+5.1f} rpm)"
#     )


# # ============================================================
# # MAIN PROGRAM
# # ============================================================

# def main():
#     if os.name == "nt":
#         print("This script is intended to run on Raspberry Pi Linux.")
#         return

#     settings = termios.tcgetattr(sys.stdin)

#     left_motor = None
#     right_motor = None
#     left_encoder = None
#     right_encoder = None

#     try:
#         left_motor = Motor(
#             LEFT_PWM_PIN,
#             LEFT_DIR_PIN,
#             invert=LEFT_MOTOR_INVERT,
#             name="left motor",
#         )

#         right_motor = Motor(
#             RIGHT_PWM_PIN,
#             RIGHT_DIR_PIN,
#             invert=RIGHT_MOTOR_INVERT,
#             name="right motor",
#         )

#         left_encoder = QuadratureEncoder(
#             LEFT_ENC_A_PIN,
#             LEFT_ENC_B_PIN,
#             invert=LEFT_ENCODER_INVERT,
#             name="left encoder",
#         )

#         right_encoder = QuadratureEncoder(
#             RIGHT_ENC_A_PIN,
#             RIGHT_ENC_B_PIN,
#             invert=RIGHT_ENCODER_INVERT,
#             name="right encoder",
#         )

#         target_linear = 0.0
#         target_turn = 0.0
#         control_linear = 0.0
#         control_turn = 0.0

#         last_command_time = time.time()
#         last_loop_time = time.time()
#         last_status_time = 0.0

#         last_left_count_for_status = left_encoder.get_count()
#         last_right_count_for_status = right_encoder.get_count()
#         last_status_count_time = time.time()

#         print(MSG)
#         print(
#             f"Settings: MAX_LINEAR_DUTY={MAX_LINEAR_DUTY:.2f}, "
#             f"MAX_TURN_DUTY={MAX_TURN_DUTY:.2f}, "
#             f"MIN_MOVING_DUTY={MIN_MOVING_DUTY:.2f}"
#         )

#         while True:
#             key = get_key(settings)
#             now = time.time()
#             dt = max(now - last_loop_time, 0.001)
#             last_loop_time = now

#             if key == "w":
#                 target_linear = constrain(
#                     target_linear + SPEED_STEP,
#                     -1.0,
#                     1.0,
#                 )
#                 last_command_time = now

#             elif key == "x":
#                 target_linear = constrain(
#                     target_linear - SPEED_STEP,
#                     -1.0,
#                     1.0,
#                 )
#                 last_command_time = now

#             elif key == "a":
#                 target_turn = constrain(
#                     target_turn + TURN_STEP,
#                     -1.0,
#                     1.0,
#                 )
#                 last_command_time = now

#             elif key == "d":
#                 target_turn = constrain(
#                     target_turn - TURN_STEP,
#                     -1.0,
#                     1.0,
#                 )
#                 last_command_time = now

#             elif key == "r":
#                 # Reset turn only.
#                 target_turn = 0.0
#                 control_turn = 0.0
#                 last_command_time = now
#                 print("TURN CLEARED")

#             elif key == "f":
#                 # Reset forward/reverse only.
#                 target_linear = 0.0
#                 control_linear = 0.0
#                 last_command_time = now
#                 print("LINEAR CLEARED")

#             elif key == " " or key == "s":
#                 target_linear = 0.0
#                 target_turn = 0.0
#                 control_linear = 0.0
#                 control_turn = 0.0
#                 left_motor.stop()
#                 right_motor.stop()
#                 last_command_time = now
#                 print("STOP")

#             elif key == "\x03":
#                 break

#             # Safety timeout
#             if now - last_command_time > COMMAND_TIMEOUT:
#                 target_linear = 0.0
#                 target_turn = 0.0

#             # Smooth target commands like TurtleBot teleop does.
#             control_linear = ramp_value(
#                 control_linear,
#                 target_linear,
#                 RAMP_STEP,
#             )

#             control_turn = ramp_value(
#                 control_turn,
#                 target_turn,
#                 RAMP_STEP,
#             )

#             # Convert linear + turn into direct motor duty commands.
#             left_cmd, right_cmd = mix_drive_commands(
#                 control_linear,
#                 control_turn,
#             )

#             # Direct open-loop motor control.
#             left_motor.set_duty_signed(left_cmd)
#             right_motor.set_duty_signed(right_cmd)

#             # Status printing.
#             should_print = (
#                 key in ["w", "x", "a", "d", "r", "f", " ", "s"]
#                 or now - last_status_time > STATUS_PERIOD
#             )

#             moving_or_commanded = (
#                 abs(target_linear) > 0.0
#                 or abs(target_turn) > 0.0
#                 or key in [" ", "s", "r", "f"]
#             )

#             if should_print and moving_or_commanded:
#                 current_left_count = left_encoder.get_count()
#                 current_right_count = right_encoder.get_count()

#                 status_dt = max(now - last_status_count_time, 0.001)

#                 print_status(
#                     target_linear,
#                     target_turn,
#                     control_linear,
#                     control_turn,
#                     left_cmd,
#                     right_cmd,
#                     current_left_count,
#                     current_right_count,
#                     last_left_count_for_status,
#                     last_right_count_for_status,
#                     status_dt,
#                 )

#                 last_left_count_for_status = current_left_count
#                 last_right_count_for_status = current_right_count
#                 last_status_count_time = now
#                 last_status_time = now

#             time.sleep(0.001)

#     except Exception as e:
#         print(f"Error: {e}")

#     finally:
#         if left_motor is not None:
#             left_motor.stop()
#         if right_motor is not None:
#             right_motor.stop()

#         if left_encoder is not None:
#             left_encoder.close()
#         if right_encoder is not None:
#             right_encoder.close()

#         if left_motor is not None:
#             left_motor.close()
#         if right_motor is not None:
#             right_motor.close()

#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#         print("\nMotors stopped. Exiting safely.")


# if __name__ == "__main__":
#     main()


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

# ##Code with encoder feedback and closed-loop speed control.


# #!/usr/bin/env python3
# """
# MECHelangelo real-wheel teleop with quadrature encoder closed-loop speed control.

# This script directly drives the physical motors from keyboard input. It reads each
# wheel encoder, estimates signed wheel speed in encoder ticks/second, and adjusts
# PWM duty using a small PI loop so the left and right wheels track the requested
# speed instead of blindly receiving the same duty cycle.

# IMPORTANT GPIO NOTES
# - Pin numbers are BCM GPIO numbers, not physical header pin numbers.
# - On most screw-terminal Raspberry Pi HATs, labels like IO12 / GPIO12 mean BCM 12.
# - The encoder outputs must not exceed 3.3 V on Raspberry Pi GPIO pins. Power the
#   Parallax encoder boards from 3.3 V, or use level shifting if powered from 5 V.
# """

# import os
# import sys
# import time
# import select
# import threading

# # from gpiozero import PWMOutputDevice, DigitalOutputDevice, InputDevice
# from gpiozero import PWMOutputDevice, DigitalOutputDevice, DigitalInputDevice

# if os.name != "nt":
#     import termios
#     import tty


# # ============================================================
# # GPIO PIN CONFIGURATION
# # ============================================================
# # These are Raspberry Pi BCM GPIO numbers.
# # Match them to the IO/GPIO labels on your screw-terminal HAT.

# # DRI0002 / MD1.3 motor driver pins
# RIGHT_PWM_PIN = 12   # Driver E1, PWM speed control
# RIGHT_DIR_PIN = 23   # Driver M1, direction

# LEFT_PWM_PIN = 13    # Driver E2, PWM speed control
# LEFT_DIR_PIN = 24    # Driver M2, direction

# # Quadrature encoder pins
# # Each encoder board has two white signal wires: channel A and channel B.
# # Change these to match where you connect the two white wires for each wheel.
# LEFT_ENC_A_PIN = 6 #swapped these
# LEFT_ENC_B_PIN = 5
# # RIGHT_ENC_A_PIN = 16
# # RIGHT_ENC_B_PIN = 20
# RIGHT_ENC_A_PIN = 20
# RIGHT_ENC_B_PIN = 16

# # ============================================================
# # MOTOR / ENCODER SETTINGS
# # ============================================================
# PWM_FREQUENCY = 1000

# # Start conservative while tuning. Increase only after the robot is safe on blocks.
# # MAX_DUTY = 1.0
# # MIN_MOVING_DUTY = 0.18

# # Hard electrical safety limit. Do not exceed 1.0.
# MAX_DUTY = 1.0

# # Separate behaviour limits
# MAX_LINEAR_DUTY = 1.00   # forward/back driving power
# MAX_TURN_DUTY = 0.75     # pure turning power, tune 0.70 to 0.80

# MIN_MOVING_DUTY = 0.18

# # The Parallax kit gives 144 positions/counts per full tyre rotation if using
# # both encoder channels with 4x quadrature decoding.
# ENCODER_TICKS_PER_REV = 144.0

# # Product guide states approx. 100 rpm no-load at 12 V. Loaded speed will be lower.
# MAX_WHEEL_RPM = 100.0
# MAX_TICKS_PER_SEC = ENCODER_TICKS_PER_REV * MAX_WHEEL_RPM / 60.0

# # Keyboard commands are normalized wheel-speed commands, not raw PWM duty.
# SPEED_STEP = 0.10 ##how much each press of the keyboard increases forward/reverse speed
# TURN_STEP = 0.05 ##how much each press of the keyboard increases turn
# RAMP_STEP = 0.03 ##how much the motor speed changes per iteration used to reduce jerk and smooth acceleration. Increase for more responsiveness, decrease for smoother but slower response.

# CONTROL_PERIOD = 0.05       # 20 Hz closed-loop update
# COMMAND_TIMEOUT = 20.0       # Stop if keyboard input is inactive this long
# STATUS_PERIOD = 0.25        # Print status while moving

# # PI speed controller gains.
# # If it reacts too weakly, increase KP slightly. If it oscillates, lower KP/KI.
# # KP = 0.0015                 # duty per ticks/sec error
# # KI = 0.00035                # duty per accumulated ticks/sec error
# KP = 0.0
# KI = 0.0
# INTEGRAL_LIMIT = 250.0

# # Set these after the first encoder test.
# # If pressing w makes the wheel move forward but measured ticks/sec is negative,
# # flip that wheel's encoder invert value.
# LEFT_MOTOR_INVERT = True ##if pressing w makes the wheel spin backwards, set this to True to flip the motor direction.
# RIGHT_MOTOR_INVERT = True
# LEFT_ENCODER_INVERT = False ##if wheel goes forward but ticks/sec is negative, set this to True to flip the sign of the encoder reading for that wheel.
# RIGHT_ENCODER_INVERT = False

# # Parallax encoder outputs are active driven when powered correctly, so pull-ups
# # are usually not needed. Set True only if your signal floats when disconnected.
# ENCODER_PULL_UP = False


# MSG = """
# MECHelangelo Physical Wheel Test - encoder closed loop
# ------------------------------------------------------

# Movement:
#         w
#    a    s    d
#         x

# w : increase forward speed
# x : increase reverse speed
# a : increase left turn
# d : increase right turn

# space or s : stop motors
# CTRL-C     : quit

# Start with the wheels lifted off the ground.
# """


# def get_key(settings, timeout=CONTROL_PERIOD):
#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], timeout)

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


# def sign(value):
#     if value > 0.0:
#         return 1.0
#     if value < 0.0:
#         return -1.0
#     return 0.0


# class QuadratureEncoder:
#     """4x quadrature decoder using two GPIO input pins."""

#     # Transition lookup table for a standard quadrature sequence.
#     # If your encoder sign is backwards, use invert=True instead of changing this.
#     _FORWARD_TRANSITIONS = {0b0001, 0b0111, 0b1110, 0b1000}
#     _REVERSE_TRANSITIONS = {0b0010, 0b1011, 0b1101, 0b0100}

#     def __init__(self, pin_a, pin_b, invert=False, name="encoder"):
#         self.name = name
#         # self.a = InputDevice(pin_a, pull_up=ENCODER_PULL_UP)
#         # self.b = InputDevice(pin_b, pull_up=ENCODER_PULL_UP)
#         self.a = DigitalInputDevice(pin_a, pull_up=ENCODER_PULL_UP)
#         self.b = DigitalInputDevice(pin_b, pull_up=ENCODER_PULL_UP)
#         self.invert = invert
#         self.count = 0
#         self.lock = threading.Lock()

#         self.last_state = self._read_state()

#         self.a.when_activated = self._edge
#         self.a.when_deactivated = self._edge
#         self.b.when_activated = self._edge
#         self.b.when_deactivated = self._edge

#     def _read_state(self):
#         return (int(self.a.value) << 1) | int(self.b.value)

#     def _edge(self):
#         new_state = self._read_state()
#         transition = (self.last_state << 2) | new_state

#         delta = 0
#         if transition in self._FORWARD_TRANSITIONS:
#             delta = 1
#         elif transition in self._REVERSE_TRANSITIONS:
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


# class Motor:
#     def __init__(self, pwm_pin, dir_pin, invert=False, name="motor"):
#         self.name = name
#         self.pwm = PWMOutputDevice(
#             pwm_pin,
#             frequency=PWM_FREQUENCY,
#             initial_value=0.0,
#         )
#         self.direction = DigitalOutputDevice(dir_pin, initial_value=False)
#         self.invert = invert
#         self.last_command = 0.0

#     def set_duty_signed(self, command):
#         """
#         command range: -1.0 to +1.0
#         + = forward, - = reverse, 0 = stop.
#         DRI0002 logic: E pin is PWM speed; M pin LOW/HIGH controls direction.
#         """
#         command = constrain(command, -1.0, 1.0)

#         if self.invert:
#             command = -command

#         self.last_command = command

#         if abs(command) < 0.001:
#             self.pwm.value = 0.0
#             return

#         duty = constrain(abs(command), 0.0, MAX_DUTY)

#         if command > 0.0:
#             self.direction.off()   # M = LOW, forward for DRI0002
#         else:
#             self.direction.on()    # M = HIGH, reverse for DRI0002

#         self.pwm.value = duty

#     def stop(self):
#         self.last_command = 0.0
#         self.pwm.value = 0.0

#     def close(self):
#         self.stop()
#         self.pwm.close()
#         self.direction.close()


# class WheelSpeedController:
#     """Closed-loop wheel speed controller for one wheel."""

#     def __init__(self, motor, encoder, name):
#         self.motor = motor
#         self.encoder = encoder
#         self.name = name
#         self.last_count = encoder.get_count()
#         self.filtered_ticks_per_sec = 0.0
#         self.integral = 0.0
#         self.last_target_direction = 0.0

#     def update(self, target_ticks_per_sec, dt):
#         current_count = self.encoder.get_count()
#         delta_count = current_count - self.last_count
#         self.last_count = current_count

#         raw_ticks_per_sec = delta_count / dt if dt > 0.0 else 0.0

#         # Light smoothing makes the PI loop less jumpy at low speeds.
#         alpha = 0.35
#         self.filtered_ticks_per_sec = (
#             alpha * raw_ticks_per_sec
#             + (1.0 - alpha) * self.filtered_ticks_per_sec
#         )

#         target_direction = sign(target_ticks_per_sec)

#         if target_direction == 0.0:
#             self.integral = 0.0
#             self.last_target_direction = 0.0
#             self.motor.stop()
#             return 0.0, self.filtered_ticks_per_sec, current_count

#         if target_direction != self.last_target_direction:
#             self.integral = 0.0
#             self.last_target_direction = target_direction

#         measured_along_target = self.filtered_ticks_per_sec * target_direction
#         speed_error = abs(target_ticks_per_sec) - measured_along_target

#         self.integral = constrain(
#             self.integral + speed_error * dt,
#             -INTEGRAL_LIMIT,
#             INTEGRAL_LIMIT,
#         )

#         feedforward = (abs(target_ticks_per_sec) / MAX_TICKS_PER_SEC) * MAX_DUTY
#         correction = KP * speed_error + KI * self.integral
#         duty = feedforward + correction

#         duty = constrain(duty, MIN_MOVING_DUTY, MAX_DUTY)
#         signed_duty = target_direction * duty

#         self.motor.set_duty_signed(signed_duty)
#         return signed_duty, self.filtered_ticks_per_sec, current_count

#     def stop(self):
#         self.integral = 0.0
#         self.motor.stop()


# def print_status(target_linear, target_turn, left_target, right_target,
#                  left_actual, right_actual, left_duty, right_duty,
#                  left_count, right_count):
#     left_rpm = (left_actual / ENCODER_TICKS_PER_REV) * 60.0
#     right_rpm = (right_actual / ENCODER_TICKS_PER_REV) * 60.0

#     print(
#         f"lin {target_linear:+.2f} turn {target_turn:+.2f} | "
#         f"L tgt {left_target:+6.1f} t/s act {left_actual:+6.1f} t/s "
#         f"({left_rpm:+5.1f} rpm) duty {left_duty:+.2f} cnt {left_count:+7d} | "
#         f"R tgt {right_target:+6.1f} t/s act {right_actual:+6.1f} t/s "
#         f"({right_rpm:+5.1f} rpm) duty {right_duty:+.2f} cnt {right_count:+7d}"
#     )


# def main():
#     if os.name == "nt":
#         print("This script is intended to run on Raspberry Pi Linux.")
#         return

#     settings = termios.tcgetattr(sys.stdin)

#     left_motor = Motor(
#         LEFT_PWM_PIN,
#         LEFT_DIR_PIN,
#         invert=LEFT_MOTOR_INVERT,
#         name="left motor",
#     )
#     right_motor = Motor(
#         RIGHT_PWM_PIN,
#         RIGHT_DIR_PIN,
#         invert=RIGHT_MOTOR_INVERT,
#         name="right motor",
#     )

#     left_encoder = QuadratureEncoder(
#         LEFT_ENC_A_PIN,
#         LEFT_ENC_B_PIN,
#         invert=LEFT_ENCODER_INVERT,
#         name="left encoder",
#     )
#     right_encoder = QuadratureEncoder(
#         RIGHT_ENC_A_PIN,
#         RIGHT_ENC_B_PIN,
#         invert=RIGHT_ENCODER_INVERT,
#         name="right encoder",
#     )

#     left_controller = WheelSpeedController(left_motor, left_encoder, "left")
#     right_controller = WheelSpeedController(right_motor, right_encoder, "right")

#     target_linear = 0.0
#     target_turn = 0.0
#     control_linear = 0.0
#     control_turn = 0.0

#     last_command_time = time.time()
#     last_loop_time = time.time()
#     last_status_time = 0.0

#     print(MSG)
#     print(f"Max target speed: {MAX_TICKS_PER_SEC:.1f} ticks/s per wheel")

#     try:
#         while True:
#             key = get_key(settings)
#             now = time.time()
#             dt = max(now - last_loop_time, 0.001)
#             last_loop_time = now

#             if key == "w":
#                 target_linear = constrain(target_linear + SPEED_STEP, -1.0, 1.0)
#                 last_command_time = now
#             elif key == "x":
#                 target_linear = constrain(target_linear - SPEED_STEP, -1.0, 1.0)
#                 last_command_time = now
#             elif key == "a":
#                 target_turn = constrain(target_turn + TURN_STEP, -1.0, 1.0)
#                 last_command_time = now
#             elif key == "d":
#                 target_turn = constrain(target_turn - TURN_STEP, -1.0, 1.0)
#                 last_command_time = now
#             elif key == " " or key == "s":
#                 target_linear = 0.0
#                 target_turn = 0.0
#                 control_linear = 0.0
#                 control_turn = 0.0
#                 left_controller.stop()
#                 right_controller.stop()
#                 last_command_time = now
#                 print("STOP")
#             elif key == "\x03":
#                 break

#             if now - last_command_time > COMMAND_TIMEOUT:
#                 target_linear = 0.0
#                 target_turn = 0.0

#             control_linear = ramp_value(control_linear, target_linear, RAMP_STEP)
#             control_turn = ramp_value(control_turn, target_turn, RAMP_STEP)

#             # Differential-drive mixing in normalized wheel-speed space.
#             left_fraction = constrain(control_linear - control_turn, -1.0, 1.0)
#             right_fraction = constrain(control_linear + control_turn, -1.0, 1.0)

#             left_target_tps = left_fraction * MAX_TICKS_PER_SEC
#             right_target_tps = right_fraction * MAX_TICKS_PER_SEC

#             left_duty, left_actual_tps, left_count = left_controller.update(left_target_tps, dt)
#             right_duty, right_actual_tps, right_count = right_controller.update(right_target_tps, dt)

#             if key in ["w", "x", "a", "d", " ", "s"] or now - last_status_time > STATUS_PERIOD:
#                 if abs(target_linear) > 0.0 or abs(target_turn) > 0.0 or key in [" ", "s"]:
#                     print_status(
#                         target_linear,
#                         target_turn,
#                         left_target_tps,
#                         right_target_tps,
#                         left_actual_tps,
#                         right_actual_tps,
#                         left_duty,
#                         right_duty,
#                         left_count,
#                         right_count,
#                     )
#                 last_status_time = now

#     except Exception as e:
#         print(f"Error: {e}")

#     finally:
#         left_controller.stop()
#         right_controller.stop()
#         left_encoder.close()
#         right_encoder.close()
#         left_motor.close()
#         right_motor.close()

#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#         print("\nMotors stopped. Exiting safely.")


# if __name__ == "__main__":
#     main()
