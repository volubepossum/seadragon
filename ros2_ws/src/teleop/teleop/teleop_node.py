#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from motor_controller.msg import Motors, Motor
import readchar
import threading


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        self.get_logger().info(
            "Use keys q, w, e, r, t to move forward and a, s, d, f, g to move backward"
        )

        # Publisher for motor thrust commands
        self.publisher_ = self.create_publisher(Motors, "motor_thrust", 10)

        # Initialize thrust values
        self.thrust_values = [0.0] * 5  # For 5 motors
        self.forward_thrust = 10.0  # Ensure float
        self.backward_thrust = -10.0  # Ensure float

        # Define control keys
        self.forward_keys = ["q", "w", "e", "r", "t"]
        self.backward_keys = ["a", "s", "d", "f", "g"]

        # Control flag
        self.running = True

        # Timer for resetting thrust values
        self.reset_timer = self.create_timer(0.1, self.reset_thrust_values)

        # Start keyboard reading thread
        self.keyboard_thread = threading.Thread(target=self.read_keyboard)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def read_keyboard(self):
        while self.running:
            try:
                key = readchar.readchar()
                self.get_logger().info(f"Key pressed: {key}")
                self.handle_key(key)
            except Exception as e:
                self.get_logger().error(f"Error reading keyboard: {e}")

    def handle_key(self, key):
        if key == "x":
            self.running = False
        elif key in self.forward_keys:
            motor_index = self.forward_keys.index(key)
            self.thrust_values[motor_index] = self.forward_thrust
        elif key in self.backward_keys:
            motor_index = self.backward_keys.index(key)
            self.thrust_values[motor_index] = self.backward_thrust
        else:
            return

        # Publish thrust values
        self.publish_thrust_values()

        # Reset the timer
        self.reset_timer.cancel()
        self.reset_timer = self.create_timer(0.1, self.reset_thrust_values)

    def publish_thrust_values(self):
        msg = Motors()
        msg.motors = [
            Motor(id=i + 1, thrust=val)
            for i, val in enumerate(self.thrust_values)
        ]
        self.get_logger().info(f"Publishing thrust values: {self.thrust_values}")
        self.publisher_.publish(msg)

    def reset_thrust_values(self):
        self.thrust_values = [0.0] * 5
        self.publish_thrust_values()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
