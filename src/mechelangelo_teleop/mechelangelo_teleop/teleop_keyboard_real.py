#!/usr/bin/env python3

import os
import sys
import time
import select

from gpiozero import PWMOutputDevice, DigitalOutputDevice

if os.name != "nt":
    import termios
    import tty

# Everything up here is what needs to change, Pin definitions, power and speed settings.

# ============================================================
# GPIO PIN CONFIGURATION FOR DRI0002 / MD1.3 MOTOR DRIVER
# ============================================================
# These are Raspberry Pi BCM GPIO numbers, matching IO labels on the HAT.

RIGHT_PWM_PIN = 12   # Driver E1
RIGHT_DIR_PIN = 23   # Driver M1

LEFT_PWM_PIN = 13    # Driver E2
LEFT_DIR_PIN = 24    # Driver M2


# ============================================================
# SAFETY / CONTROL SETTINGS
# ============================================================

MAX_DUTY = 0.25 #Power control for the motors (25% start for testing) PWM duty cycle range 0.0-1.0.

DUTY_STEP = 0.02 #how much each press of the keyboard increases speed
TURN_STEP = 0.02 #how much each press of the keyboard increases turn
RAMP_STEP = 0.01 #how much the motor speed changes per iteration used to reduce jerk and smooth accerleration

PWM_FREQUENCY = 1000

# If no key command is received for this long, stop motors.
COMMAND_TIMEOUT = 0.5

#terminal printout instructions
MSG = """
MECHelangelo Physical Wheel Test
--------------------------------

Movement:
        w
   a    s    d
        x

w : increase forward speed
x : increase reverse speed
a : increase left turn
d : increase right turn

space or s : stop motors

CTRL-C : quit

Important:
- Start with wheels lifted off the ground.
- Keep MAX_DUTY low for first test.
- Have a battery disconnect or emergency stop ready.
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)

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


def print_status(target_linear, target_turn, left_cmd, right_cmd):
    print(
        f"target linear: {target_linear:+.2f} | "
        f"target turn: {target_turn:+.2f} | "
        f"left motor: {left_cmd:+.2f} | "
        f"right motor: {right_cmd:+.2f}"
    )

class Motor:
    def __init__(self, pwm_pin, dir_pin, invert=False):
        self.pwm = PWMOutputDevice(
            pwm_pin,
            frequency=PWM_FREQUENCY,
            initial_value=0.0
        )

        self.direction = DigitalOutputDevice(
            dir_pin,
            initial_value=False
        )

        self.invert = invert

    def set_speed(self, command):
        """
        command:
            + value = forward
            - value = reverse
            0       = stop

        DRI0002 logic:
            Enable pin E = PWM speed
            Direction pin M = LOW/HIGH direction
        """

        command = constrain(command, -1.0, 1.0)

        if self.invert:
            command = -command

        if command > 0.0:
            # Forward
            # Datasheet says M = LOW gives forward.
            self.direction.off()
            self.pwm.value = abs(command)

        elif command < 0.0:
            # Reverse
            # Datasheet says M = HIGH gives back direction.
            self.direction.on()
            self.pwm.value = abs(command)

        else:
            # Stop
            self.pwm.value = 0.0

    def stop(self):
        self.pwm.value = 0.0


def main():
    if os.name == "nt":
        print("This script is intended to run on Raspberry Pi Linux.")
        return

    settings = termios.tcgetattr(sys.stdin)

    # If a wheel spins backwards when pressing w, change its invert value.
    left_motor = Motor(
        LEFT_PWM_PIN,
        LEFT_DIR_PIN,
        invert=False
    )
    
    right_motor = Motor(
        RIGHT_PWM_PIN,
        RIGHT_DIR_PIN,
        invert=False
    )

    target_linear = 0.0
    target_turn = 0.0

    control_linear = 0.0
    control_turn = 0.0

    last_command_time = time.time()

    print(MSG)

    try:
        while True:
            key = get_key(settings)

            if key == "w":
                target_linear = constrain(
                    target_linear + DUTY_STEP,
                    -MAX_DUTY,
                    MAX_DUTY
                )
                last_command_time = time.time()

            elif key == "x":
                target_linear = constrain(
                    target_linear - DUTY_STEP,
                    -MAX_DUTY,
                    MAX_DUTY
                )
                last_command_time = time.time()

            elif key == "a":
                target_turn = constrain(
                    target_turn + TURN_STEP,
                    -MAX_DUTY,
                    MAX_DUTY
                )
                last_command_time = time.time()

            elif key == "d":
                target_turn = constrain(
                    target_turn - TURN_STEP,
                    -MAX_DUTY,
                    MAX_DUTY
                )
                last_command_time = time.time()

            elif key == " " or key == "s":
                target_linear = 0.0
                target_turn = 0.0
                control_linear = 0.0
                control_turn = 0.0
                left_motor.stop()
                right_motor.stop()
                last_command_time = time.time()
                print("STOP")

            elif key == "\x03":
                break

            # Safety timeout
            if time.time() - last_command_time > COMMAND_TIMEOUT:
                target_linear = 0.0
                target_turn = 0.0

            control_linear = ramp_value(
                control_linear,
                target_linear,
                RAMP_STEP
            )

            control_turn = ramp_value(
                control_turn,
                target_turn,
                RAMP_STEP
            )

            # Differential drive mixing
            left_cmd = control_linear - control_turn
            right_cmd = control_linear + control_turn

            left_cmd = constrain(left_cmd, -MAX_DUTY, MAX_DUTY)
            right_cmd = constrain(right_cmd, -MAX_DUTY, MAX_DUTY)

            left_motor.set_speed(left_cmd)
            right_motor.set_speed(right_cmd)

            if key in ["w", "x", "a", "d", " ", "s"]:
                print_status(
                    target_linear,
                    target_turn,
                    left_cmd,
                    right_cmd
                )

            time.sleep(0.02)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        left_motor.stop()
        right_motor.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        print("\nMotors stopped. Exiting safely.")


if __name__ == "__main__":
    main()