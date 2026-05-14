#!/usr/bin/env python3

import sys
import time
import termios
import tty
import select
import ThunderBorg


# ============================================================
# User settings
# ============================================================

# PiBorg's tbJoystick.py treats:
#   Motor 1 = right wheel
#   Motor 2 = left wheel
MOTOR1_IS_RIGHT = True

# Change these if one wheel spins the wrong way
LEFT_INVERT = False
RIGHT_INVERT = False

# Start safely. Increase later after testing.
MAX_POWER = 0.35

# How much W/S changes forward speed each keypress
SPEED_STEP = 0.05

# How much A/D changes turning each keypress
TURN_STEP = 0.05

# Maximum steering command before scaling by MAX_POWER
MAX_TURN = 0.8

# Send commands faster than the ThunderBorg failsafe timeout.
# ThunderBorg failsafe turns motors off if commands stop for ~0.25 s.
LOOP_DT = 0.05

# Small values below this are treated as zero
DEADBAND = 0.03


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

    # Normalize so neither side exceeds +/- 1.0
    scale = max(1.0, abs(left), abs(right))
    left /= scale
    right /= scale

    return left, right


def send_motor_command(TB, left, right):
    """
    Send left/right wheel commands to the ThunderBorg.

    Based on tbJoystick.py:
        Motor 1 = right
        Motor 2 = left
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
        TB.SetMotor1(right)
        TB.SetMotor2(left)
    else:
        TB.SetMotor1(left)
        TB.SetMotor2(right)

    return left, right


def print_help():
    print()
    print("ThunderBorg keyboard teleop")
    print("---------------------------")
    print("w : increase forward speed")
    print("s : increase reverse speed")
    print("a : turn left")
    print("d : turn right")
    print("space or x : stop")
    print("q : quit")
    print()
    print(f"MAX_POWER = {MAX_POWER:.2f}")
    print()


# ============================================================
# Main
# ============================================================

def main():
    TB = ThunderBorg.ThunderBorg()
    TB.Init()

    if not TB.foundChip:
        print("No ThunderBorg found.")
        boards = ThunderBorg.ScanForThunderBorg()
        print("Boards found:", boards)
        sys.exit(1)

    print("ThunderBorg connected.")

    try:
        battery = TB.GetBatteryReading()
        print(f"Battery voltage: {battery:.2f} V")
    except Exception:
        print("Warning: Could not read battery voltage.")

    # Enable failsafe so motors stop if the script crashes or communication stops.
    failsafe_ok = False
    for _ in range(5):
        TB.SetCommsFailsafe(True)
        time.sleep(0.02)
        if TB.GetCommsFailsafe():
            failsafe_ok = True
            break

    if not failsafe_ok:
        print("Warning: Could not confirm ThunderBorg comms failsafe.")
        print("Continuing, but be careful.")

    TB.MotorsOff()
    TB.SetLedShowBattery(True)

    speed = 0.0
    turn = 0.0

    old_terminal_settings = termios.tcgetattr(sys.stdin)

    print_help()
    print("Ready. Put the robot on blocks for first test.")

    try:
        tty.setcbreak(sys.stdin.fileno())

        last_status_time = 0.0

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
            left_out, right_out = send_motor_command(TB, left_cmd, right_cmd)

            now = time.time()
            if now - last_status_time > 0.5:
                fault1 = TB.GetDriveFault1()
                fault2 = TB.GetDriveFault2()
                print(
                    f"\rSpeed: {speed:+.2f}  Turn: {turn:+.2f}  "
                    f"Left: {left_out:+.2f}  Right: {right_out:+.2f}  "
                    f"Fault1: {fault1}  Fault2: {fault2}      ",
                    end="",
                    flush=True,
                )
                last_status_time = now

    except KeyboardInterrupt:
        pass

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)
        TB.MotorsOff()
        TB.SetCommsFailsafe(False)
        TB.SetLedShowBattery(False)
        TB.SetLeds(0.2, 0.0, 0.0)
        print("\nMotors off. Exiting.")


if __name__ == "__main__":
    main()
