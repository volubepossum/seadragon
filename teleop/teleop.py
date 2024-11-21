import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import keyboard

class ThrusterControlNode(Node):
    def __init__(self):
        super().__init__('thruster_control_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor_thrust', 10)
        
        # Define max thrust values
        self.forward_thrust = 44.37
        self.backward_thrust = -34.50
        
        # Initial thrust values for each motor
        self.thrust_values = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Key mappings
        self.forward_keys = ["q", "w", "e", "r", "t"]
        self.backward_keys = ["a", "s", "d", "f", "g"]

    def run(self):
        try:
            self.get_logger().info("Thruster control started. Use Q/W/E/R/T for forward and A/S/D/F/G for backward.")
            while rclpy.ok():
                # Check keypresses for forward and backward thrust
                for i in range(len(self.thrust_values)):
                    if keyboard.is_pressed(self.forward_keys[i]):
                        self.thrust_values[i] = self.forward_thrust
                    elif keyboard.is_pressed(self.backward_keys[i]):
                        self.thrust_values[i] = self.backward_thrust
                    else:
                        self.thrust_values[i] = 0.0
                
                # Publish thrust values
                msg = Float32MultiArray()
                msg.data = self.thrust_values
                self.publisher_.publish(msg)
                
        except KeyboardInterrupt:
            self.get_logger().info("Stopping thruster control.")
        finally:
            keyboard.unhook_all()  # Clean up keyboard hooks

def main():
    rclpy.init()
    node = ThrusterControlNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
