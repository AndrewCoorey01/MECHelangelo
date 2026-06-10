#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import paho.mqtt.client as mqtt
import json

class MqttBridge(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        # ── Parameters — change broker IP here ───────────────────
        self.declare_parameter('broker_ip',   'localhost')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('mqtt_topic',  'arm/angles')

        broker_ip   = self.get_parameter('broker_ip').value
        broker_port = self.get_parameter('broker_port').value
        mqtt_topic  = self.get_parameter('mqtt_topic').value

        # ── ROS publisher ─────────────────────────────────────────
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/arm_pose',
            10
        )

        # ── MQTT client ───────────────────────────────────────────
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        self.get_logger().info(f'Connecting to MQTT broker at {broker_ip}:{broker_port}')

        try:
            self.mqtt_client.connect(broker_ip, broker_port, keepalive=60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to broker: {e}')

        self.mqtt_topic = mqtt_topic
        self.get_logger().info('MQTT bridge node started')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f'Subscribed to: {self.mqtt_topic}')
        else:
            self.get_logger().error(f'MQTT connection failed rc={rc}')

    def on_mqtt_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())

            # Pack into Float32MultiArray
            # Layout: [l_shoulder, l_elbow, l_z_fwd, l_roll,
            #          r_shoulder, r_elbow, r_z_fwd, r_roll]
            ros_msg = Float32MultiArray()
            ros_msg.data = [
                float(data.get('l_shoulder') or -1.0),
                float(data.get('l_elbow')    or -1.0),
                float(data.get('l_z_fwd')    or -1.0),
                0.0,
                float(data.get('r_shoulder') or -1.0),
                float(data.get('r_elbow')    or -1.0),
                float(data.get('r_z_fwd')    or -1.0),
                0.0,
            ]

            self.publisher_.publish(ros_msg)

            # Pretty print to terminal
            def fmt(val):
                return f"{val:>6}" if val is not None else "  ---"

            self.get_logger().info(
                f"\n"
                f"  LEFT  — shoulder: {fmt(data.get('l_shoulder'))}°  "
                f"elbow: {fmt(data.get('l_elbow'))}°  "
                f"z_fwd: {fmt(data.get('l_z_fwd'))}\n"
                f"  RIGHT — shoulder: {fmt(data.get('r_shoulder'))}°  "
                f"elbow: {fmt(data.get('r_elbow'))}°  "
                f"z_fwd: {fmt(data.get('r_z_fwd'))}"
            )

        except Exception as e:
            self.get_logger().error(f'Message parse error: {e}')

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
