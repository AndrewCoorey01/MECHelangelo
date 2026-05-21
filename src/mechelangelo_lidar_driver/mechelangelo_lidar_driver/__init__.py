"""
MECHelangelo base driver package.

This package contains the real hardware base driver for the MECHelangelo robot.
The base driver subscribes to /cmd_vel and converts velocity commands into
closed-loop motor control using wheel encoder feedback.
"""

__version__ = '0.0.1'