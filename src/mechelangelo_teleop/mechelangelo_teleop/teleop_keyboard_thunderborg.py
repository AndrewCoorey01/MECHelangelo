#!/usr/bin/env python3
"""
MECHelangelo ThunderBorg keyboard teleop with dual encoder debug feedback.

This script directly commands the ThunderBorg from keyboard input.

Important:
    This is for bench testing.
    It is not a ROS /cmd_vel teleop publisher.
    Do not run this at the same time as the ROS base driver.
"""

import select
import sys
import termios
import threading
import time
import tty

from gpiozero import DigitalInputDevice

from mechelangelo_base_driver import ThunderBorg



# ============================================================
# ThunderBorg / motor settings
# ============================================================

# PiBorg's examples commonly treat:
#   Motor 1 = right wheel
#   Motor 2 = left wheel
MOTOR1_IS_RIGHT = True

# Flip these if one wheel spins the wrong direction.
LEFT_INVERT = False
RIGHT_INVERT = False

# Start conservative.
MAX_POWER = 0.75

# Keyboard increments.
SPEED_STEP = 0.05
TURN_STEP = 0.05

# Maximum turn command.
MAX_TURN = 0.8

# Loop period. Keep fast enough for ThunderBorg failsafe.
LOOP_DT = 0.05

# Small motor commands below this are treated as zero.
DEADBAND = 0.03


# ============================================================
# Encoder settings
# ============================================================
# BCM GPIO numbering / screw terminal IO labels.
#
# Left encoder:
#   white A -> IO23
#   white B -> IO22
#
# Right encoder:
#   white A -> IO24
#   white B -> IO25

LEFT_ENCODER_A_PIN = 23
LEFT_ENCODER_B_PIN = 22

RIGHT_ENCODER_A_PIN = 24
RIGHT_ENCODER_B_PIN = 25

ENCODER_PULL_UP = True

# Flip these if wheel moves forward but encoder speed/count is negative.
LEFT_ENCODER_INVERT = False
RIGHT_ENCODER_INVERT = False

# Parallax quadrature encoder resolution.
ENCODER_TICKS_PER_REV = 144.0


# ============================================================
# Helper functions
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadband(value):
    if abs(value) < DEADBAND:
        return 0.0
    return value


def get_key(timeout):
    """
    Non-blocking keyboard read.
    Returns one character, or None if no key was pressed.
    """

    readable, _, _ = select.select([sys.stdin], [], [], timeout)

    if readable:
        return sys.stdin.read(1)

    return None


def arcade_to_tank(speed, turn):
    """
    Convert forward speed + steering into left/right wheel commands.

    speed:
        + = forward
        - = reverse

    turn:
        + = turn right
        - = turn left
    """

    left = speed + turn
    right = speed - turn

    scale = max(1.0, abs(left), abs(right))

    left /= scale
    right /= scale

    return left, right


def send_motor_command(tb, left, right):
    """
    Send left/right wheel commands to the ThunderBorg.
    """

    left = apply_deadband(left)
    right = apply_deadband(right)

    if LEFT_INVERT:
        left = -left

    if RIGHT_INVERT:
        right = -right

    left = clamp(left, -1.0, 1.0) * MAX_POWER
    right = clamp(right, -1.0, 1.0) * MAX_POWER

    if MOTOR1_IS_RIGHT:
        tb.SetMotor1(right)
        tb.SetMotor2(left)
    else:
        tb.SetMotor1(left)
        tb.SetMotor2(right)

    return left, right


# ============================================================
# Encoder class
# ============================================================

class QuadratureEncoder:
    """
    4x quadrature decoder for one wheel encoder.
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


# ============================================================
# User interface
# ============================================================

def print_help():
    print()
    print("ThunderBorg keyboard teleop with dual encoder debug")
    print("---------------------------------------------------")
    print("w : increase forward speed")
    print("s : increase reverse speed")
    print("a : turn left")
    print("d : turn right")
    print("space or x : stop")
    print("q : quit")
    print()
    print(f"MAX_POWER = {MAX_POWER:.2f}")
    print()
    print("Start with the robot lifted on blocks.")
    print()


# ============================================================
# Main
# ============================================================

def main():
    tb = None
    left_encoder = None
    right_encoder = None
    old_terminal_settings = None

    try:
        tb = ThunderBorg.ThunderBorg()
        tb.Init()

        if not tb.foundChip:
            print("No ThunderBorg found.")
            boards = ThunderBorg.ScanForThunderBorg()
            print("Boards found:", boards)
            return 1

        print("ThunderBorg connected.")

        try:
            battery = tb.GetBatteryReading()
            print(f"Battery voltage: {battery:.2f} V")
        except Exception:
            print("Warning: Could not read battery voltage.")

        failsafe_ok = False

        for _ in range(5):
            tb.SetCommsFailsafe(True)
            time.sleep(0.02)

            if tb.GetCommsFailsafe():
                failsafe_ok = True
                break

        if failsafe_ok:
            print("ThunderBorg comms failsafe enabled.")
        else:
            print("Warning: Could not confirm ThunderBorg comms failsafe.")

        tb.MotorsOff()
        tb.SetLedShowBattery(True)

        left_encoder = QuadratureEncoder(
            LEFT_ENCODER_A_PIN,
            LEFT_ENCODER_B_PIN,
            pull_up=ENCODER_PULL_UP,
            invert=LEFT_ENCODER_INVERT,
            name='left_encoder',
        )

        right_encoder = QuadratureEncoder(
            RIGHT_ENCODER_A_PIN,
            RIGHT_ENCODER_B_PIN,
            pull_up=ENCODER_PULL_UP,
            invert=RIGHT_ENCODER_INVERT,
            name='right_encoder',
        )

        print(
            f"Left encoder enabled on GPIO{LEFT_ENCODER_A_PIN} "
            f"and GPIO{LEFT_ENCODER_B_PIN}"
        )

        print(
            f"Right encoder enabled on GPIO{RIGHT_ENCODER_A_PIN} "
            f"and GPIO{RIGHT_ENCODER_B_PIN}"
        )

        speed = 0.0
        turn = 0.0

        last_left_encoder_count = left_encoder.get_count()
        last_right_encoder_count = right_encoder.get_count()
        last_encoder_time = time.time()

        last_status_time = 0.0

        old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print_help()
        print("Ready.")

        while True:
            key = get_key(LOOP_DT)

            if key is not None:
                key = key.lower()

                if key == "w":
                    speed += SPEED_STEP

                elif key == "s":
                    speed -= SPEED_STEP

                elif key == "a":
                    turn -= TURN_STEP

                elif key == "d":
                    turn += TURN_STEP

                elif key == " " or key == "x":
                    speed = 0.0
                    turn = 0.0

                elif key == "q":
                    break

                speed = clamp(speed, -1.0, 1.0)
                turn = clamp(turn, -MAX_TURN, MAX_TURN)

            left_cmd, right_cmd = arcade_to_tank(speed, turn)
            left_out, right_out = send_motor_command(tb, left_cmd, right_cmd)

            now = time.time()

            if now - last_status_time > 0.5:
                try:
                    fault1 = tb.GetDriveFault1()
                    fault2 = tb.GetDriveFault2()
                except Exception:
                    fault1 = "unknown"
                    fault2 = "unknown"

                left_count = left_encoder.get_count()
                right_count = right_encoder.get_count()

                dt = max(now - last_encoder_time, 1e-3)

                left_delta = left_count - last_left_encoder_count
                right_delta = right_count - last_right_encoder_count

                left_tps = left_delta / dt
                right_tps = right_delta / dt

                left_rpm = (left_tps / ENCODER_TICKS_PER_REV) * 60.0
                right_rpm = (right_tps / ENCODER_TICKS_PER_REV) * 60.0

                last_left_encoder_count = left_count
                last_right_encoder_count = right_count
                last_encoder_time = now

                print(
                    f"\rSpeed: {speed:+.2f}  Turn: {turn:+.2f}  "
                    f"Left out: {left_out:+.2f}  Right out: {right_out:+.2f}  "
                    f"L count: {left_count:+7d}  "
                    f"L speed: {left_tps:+7.1f} t/s  "
                    f"L rpm: {left_rpm:+6.1f}  "
                    f"R count: {right_count:+7d}  "
                    f"R speed: {right_tps:+7.1f} t/s  "
                    f"R rpm: {right_rpm:+6.1f}  "
                    f"Fault1: {fault1}  Fault2: {fault2}      ",
                    end="",
                    flush=True,
                )

                last_status_time = now

        return 0

    except KeyboardInterrupt:
        return 0

    except Exception as exc:
        print(f"\nERROR: {exc}")
        return 1

    finally:
        if old_terminal_settings is not None:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                old_terminal_settings,
            )

        if tb is not None:
            try:
                tb.MotorsOff()
            except Exception:
                pass

            try:
                tb.SetCommsFailsafe(False)
            except Exception:
                pass

            try:
                tb.SetLedShowBattery(False)
                tb.SetLeds(0.2, 0.0, 0.0)
            except Exception:
                pass

        if left_encoder is not None:
            try:
                left_encoder.close()
            except Exception:
                pass

        if right_encoder is not None:
            try:
                right_encoder.close()
            except Exception:
                pass

        print("\nMotors off. Exiting.")


if __name__ == "__main__":
    raise SystemExit(main())