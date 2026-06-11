#!/usr/bin/env python3
"""
dual_ultrasonic_node.py

Sequentially reads two HY-SRF05 / HC-SR04-style ultrasonic sensors using
Raspberry Pi GPIO and publishes sensor_msgs/Range topics.

Default BCM GPIO wiring:
  left/front-left:   TRIG GPIO17, ECHO GPIO22
  right/front-right: TRIG GPIO27, ECHO GPIO23

The Echo lines MUST be reduced to a Pi-safe voltage before reaching GPIO.
The current hardware uses a 10 kΩ / 10 kΩ divider on each Echo line.
"""

from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import lgpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


@dataclass
class SensorChannel:
    name: str
    trigger_gpio: int
    echo_gpio: int
    topic: str
    frame_id: str
    history: Deque[float]


class DualUltrasonicNode(Node):
    """Read two ultrasonic sensors one at a time to avoid acoustic cross-talk."""

    SPEED_OF_SOUND_MPS = 343.0

    def __init__(self) -> None:
        super().__init__("dual_ultrasonic_node")

        self.declare_parameter("left_trigger_gpio", 17)
        self.declare_parameter("left_echo_gpio", 22)
        self.declare_parameter("right_trigger_gpio", 27)
        self.declare_parameter("right_echo_gpio", 23)
        self.declare_parameter("left_topic", "/ultrasonic/left")
        self.declare_parameter("right_topic", "/ultrasonic/right")
        self.declare_parameter("left_frame_id", "ultrasonic_left")
        self.declare_parameter("right_frame_id", "ultrasonic_right")
        self.declare_parameter("min_range_m", 0.02)
        self.declare_parameter("max_range_m", 4.0)
        self.declare_parameter("field_of_view_rad", 0.45)
        self.declare_parameter("inter_trigger_period_s", 0.07)
        self.declare_parameter("echo_timeout_s", 0.03)
        self.declare_parameter("median_window", 5)

        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.field_of_view_rad = float(
            self.get_parameter("field_of_view_rad").value
        )
        self.echo_timeout_s = float(self.get_parameter("echo_timeout_s").value)
        self.median_window = max(1, int(self.get_parameter("median_window").value))

        self.channels = [
            SensorChannel(
                name="left",
                trigger_gpio=int(
                    self.get_parameter("left_trigger_gpio").value
                ),
                echo_gpio=int(self.get_parameter("left_echo_gpio").value),
                topic=str(self.get_parameter("left_topic").value),
                frame_id=str(self.get_parameter("left_frame_id").value),
                history=deque(maxlen=self.median_window),
            ),
            SensorChannel(
                name="right",
                trigger_gpio=int(
                    self.get_parameter("right_trigger_gpio").value
                ),
                echo_gpio=int(self.get_parameter("right_echo_gpio").value),
                topic=str(self.get_parameter("right_topic").value),
                frame_id=str(self.get_parameter("right_frame_id").value),
                history=deque(maxlen=self.median_window),
            ),
        ]

        self.publishers = {
            channel.name: self.create_publisher(Range, channel.topic, 10)
            for channel in self.channels
        }

        try:
            self.gpio_handle = lgpio.gpiochip_open(0)
            if self.gpio_handle < 0:
                raise RuntimeError(
                    f"lgpio.gpiochip_open(0) returned {self.gpio_handle}"
                )

            for channel in self.channels:
                lgpio.gpio_claim_output(
                    self.gpio_handle, channel.trigger_gpio, 0
                )
                lgpio.gpio_claim_input(
                    self.gpio_handle, channel.echo_gpio
                )
                lgpio.gpio_write(
                    self.gpio_handle, channel.trigger_gpio, 0
                )
        except Exception:
            self.get_logger().exception(
                "Unable to claim Raspberry Pi GPIO. "
                "Run with access to /dev/gpiochip0, usually using the "
                "project's privileged Docker container."
            )
            raise

        self.next_channel_index = 0
        inter_trigger_period = max(
            0.055,
            float(self.get_parameter("inter_trigger_period_s").value),
        )
        self.timer = self.create_timer(inter_trigger_period, self._timer_callback)

        self.get_logger().info(
            "Dual ultrasonic node started: "
            "left TRIG/ECHO=%d/%d, right TRIG/ECHO=%d/%d, "
            "one sensor triggered every %.3f s"
            % (
                self.channels[0].trigger_gpio,
                self.channels[0].echo_gpio,
                self.channels[1].trigger_gpio,
                self.channels[1].echo_gpio,
                inter_trigger_period,
            )
        )

    def _timer_callback(self) -> None:
        channel = self.channels[self.next_channel_index]
        self.next_channel_index = (self.next_channel_index + 1) % len(
            self.channels
        )

        raw_distance = self._measure_distance(channel)
        filtered_distance: Optional[float] = None

        if (
            raw_distance is not None
            and self.min_range_m <= raw_distance <= self.max_range_m
        ):
            channel.history.append(raw_distance)
            filtered_distance = float(statistics.median(channel.history))

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = channel.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.field_of_view_rad
        msg.min_range = self.min_range_m
        msg.max_range = self.max_range_m

        # sensor_msgs/Range convention: +inf means no return inside max range.
        msg.range = (
            filtered_distance
            if filtered_distance is not None
            else math.inf
        )
        self.publishers[channel.name].publish(msg)

        if filtered_distance is None:
            self.get_logger().debug("%s: no valid echo" % channel.name)
        else:
            self.get_logger().info(
                "%s: %.3f m" % (channel.name, filtered_distance),
                throttle_duration_sec=1.0,
            )

    def _measure_distance(self, channel: SensorChannel) -> Optional[float]:
        """Return distance in metres, or None on timeout/out-of-range."""
        try:
            # Ensure a clean low level before the trigger pulse.
            lgpio.gpio_write(self.gpio_handle, channel.trigger_gpio, 0)
            time.sleep(0.000002)
            lgpio.gpio_write(self.gpio_handle, channel.trigger_gpio, 1)
            time.sleep(0.000010)
            lgpio.gpio_write(self.gpio_handle, channel.trigger_gpio, 0)

            wait_start = time.monotonic()
            while lgpio.gpio_read(
                self.gpio_handle, channel.echo_gpio
            ) == 0:
                if time.monotonic() - wait_start >= self.echo_timeout_s:
                    return None

            pulse_start = time.monotonic()
            while lgpio.gpio_read(
                self.gpio_handle, channel.echo_gpio
            ) == 1:
                if time.monotonic() - pulse_start >= self.echo_timeout_s:
                    return None

            pulse_end = time.monotonic()
            pulse_duration = pulse_end - pulse_start
            distance_m = (
                pulse_duration * self.SPEED_OF_SOUND_MPS / 2.0
            )

            if not math.isfinite(distance_m):
                return None
            return distance_m

        except Exception as exc:
            self.get_logger().error(
                "%s ultrasonic read failed: %s" % (channel.name, exc)
            )
            return None

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "gpio_handle"):
                for channel in self.channels:
                    try:
                        lgpio.gpio_write(
                            self.gpio_handle, channel.trigger_gpio, 0
                        )
                    except Exception:
                        pass
                lgpio.gpiochip_close(self.gpio_handle)
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[DualUltrasonicNode] = None

    try:
        node = DualUltrasonicNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()