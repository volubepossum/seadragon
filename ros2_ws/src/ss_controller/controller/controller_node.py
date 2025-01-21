import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from msg import Model
import numpy as np
from motor_controller.msg import Motors, Motor

class ControllerNode(Node):
    def __init__(self):
        self.state_subscription = self.create_subscription(Float32MultiArray, "/state", self.state_callback, 10)
        self.reference_subscription = self.create_subscription(Float32MultiArray, "/reference", self.reference_callback, 10)
        self.reference = None

        self.K = None
        self.N = None
        
        self.motors_publisher = self.create_publisher(Float32MultiArray, "/motors", 10)

    def reference_callback(self, msg):
        self.reference = np.array(msg.data)

    def state_callback(self, msg):
        if self.reference is None:
            return
        if self.K is None or self.N is None:
            return
        
        state = np.array(msg.data)
        u = -self.K @ state + self.N @ (self.reference)

        signal = Motors()
        for i in range(5):
            signal.motors[i].thrust = u[i]
        # Publish motor commands
        self.motors_publisher.publish(signal)

def main(args=None):
    rclpy.init(args=args)
    controler_node = ControllerNode()
    rclpy.spin(controler_node)
    controler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()