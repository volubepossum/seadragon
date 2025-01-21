import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from msg import Model
import numpy as np
from motor_controller.msg import Motors, Motor

class ControllerNode(Node):
    def __init__(self):
        self.reference_subscription = self.create_subscription(Float32MultiArray, "/reference", self.reference_callback, 10)
        self.reference = None

        self.K = None
        self.N = None
        
        self.motors_publisher = self.create_publisher(Float32MultiArray, "/motors", 10)

        u = np.array([0, 0, 0, 0, 0])
        signal = Motors()
        for i in range(5):
            signal.motors[i].thrust = u[i]
        # Publish motor commands
        self.motors_publisher.publish(signal)
        self.get_logger().info('Waiting for 10 seconds...')
        def timer_callback(self):
            self.get_logger().info('10 seconds have passed. Observer should be close to things now.')
            self.timer.cancel()
            self.state_subscription = self.create_subscription(Float32MultiArray, "/state", self.state_callback, 10)
        
        self.timer = self.create_timer(10.0, self.timer_callback)
        

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