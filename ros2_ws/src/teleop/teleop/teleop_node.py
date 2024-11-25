#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from motor_controller.msg import Motors, Motor
import threading
from inputs import get_key, devices

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
        self.forward_thrust = 10.0  
        self.backward_thrust = -10.0

        # Define control keys
        self.forward_keys = ["KEY_Q", "KEY_W", "KEY_E", "KEY_R", "KEY_T"]
        self.backward_keys = ["KEY_A", "KEY_S", "KEY_D", "KEY_F", "KEY_G"]

        # Control flag
        self.running = True

        # Start keyboard reading thread
        self.keyboard_thread = threading.Thread(target=self.read_keyboard)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def read_keyboard(self):
        while self.running:
            try:
                events = devices.keyboards[0].read()
                for event in events:
                    if event.ev_type == 'Key':
                        self.get_logger().info(f"Key event: {event.ev_type} {event.code} {event.state}")
                        self.handle_key(event.code, event.state)
            except Exception as e:
                self.get_logger().error(f"Error reading keyboard: {e}")

    def handle_key(self, key, state):
        if key == "KEY_X" and state == 1:
            self.running = False
        elif key in self.forward_keys and state == 1:
            motor_index = self.forward_keys.index(key)
            self.thrust_values[motor_index] = self.forward_thrust
        elif key in self.backward_keys and state == 1:
            motor_index = self.backward_keys.index(key)
            self.thrust_values[motor_index] = self.backward_thrust

        # Publish thrust values
        self.publish_thrust_values()

        # Reset the timer
        self.reset_timer.cancel()
        self.reset_timer = self.create_timer(0.5, self.reset_thrust_values)

    def publish_thrust_values(self):
        msg = Motors()
        msg.motors = [
            Motor(motor_id=i, thrust=val) for i, val in enumerate(self.thrust_values)
        ]
        self.publisher_.publish(msg)    

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.get_logger().info("Keyboard interrupt, shutting down node")
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
