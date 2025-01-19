import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R

class ObserverNode(Node):
    def __init__(self):
        super().__init__('observer_node')
        # Subscribers
        self.imu_subscription = self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.mag_subscription = self.create_subscription(MagneticField, 'imu_data_mag', self.mag_callback, 10)
        self.model_subscription = self.create_subscription(Float32MultiArray, 'model', self.model_callback, 10)
        self.motor_thrust_subscription = self.create_subscription(Float32MultiArray, 'motor_thrust', self.motor_thrust_callback, 10)

        # Publisher
        self.state_publisher = self.create_publisher(Float32MultiArray, 'estimated_state', 10)

        # State variables
        self.A = None
        self.B = None
        self.C = None
        self.current_imu = None
        self.current_mag = None
        self.estimated_state = np.zeros(6)  # [x_rate, y_rate, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate]

        # Timer for state estimation
        self.create_timer(0.01, self.estimate_state)  # 100Hz update rate

    def imu_callback(self, msg):
        self.current_imu = msg
        
    def mag_callback(self, msg):
        self.current_mag = msg

    def model_callback(self, msg):
        self.abc_matrices = np.array(msg.data).reshape((3, 3))

    def motor_thrust_callback(self, msg):
        self.control_input = np.array(msg.data)

    def estimate_state(self):
        if None in (self.current_imu, self.abc_matrices, self.control_input):
            return

        # Extract orientation and angular velocity
        quat = [
            self.current_imu.orientation.x,
            self.current_imu.orientation.y,
            self.current_imu.orientation.z,
            self.current_imu.orientation.w
        ]
        rpy = R.from_quat(quat).as_euler('xyz', degrees=False)
        angular_vel = [
            self.current_imu.angular_velocity.x,
            self.current_imu.angular_velocity.y,
            self.current_imu.angular_velocity.z
        ]

        # Update estimated state
        self.estimated_state[:3] = rpy
        self.estimated_state[3:] = angular_vel

        # Publish estimated state
        msg = Float32MultiArray()
        msg.data = self.estimated_state.tolist()
        self.state_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    observer_node = ObserverNode()
    rclpy.spin(observer_node)
    observer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()