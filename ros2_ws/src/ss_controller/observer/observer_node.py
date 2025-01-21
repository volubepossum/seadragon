import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import numpy as np
from msg import Observer
from motor_controller.msg import Motors


class ObserverNode(Node):
    def __init__(self):
        super().__init__("observer_node")
        # Subscribers
        self.imu_subscription = self.create_subscription(
            Imu, "imu_data", self.imu_callback, 10
        )
        self.imu_lowpass_alpha = 0.1
        self.imu_highpass_alpha = 0.1
        self.observer_subscription = self.create_subscription(
            Observer, "observer", self.observer_callback, 10
        )
        self.motor_thrust_subscription = self.create_subscription(
            Motors, "motor_thrust", self.motor_thrust_callback, 10
        )

        # Publisher
        self.state_publisher = self.create_publisher(
            Float32MultiArray, "state", 10
        )

        # Variables 
        self.A = None
        self.B = None
        self.C = None
        self.L = None
        self.dt = None

        self.state = np.zeros(13)
        self.plant_output = np.zeros(10)
        self.previous_measurement = None

        self.motor_thrust = np.zeros(5)

        self.control_period = 0.01
        # Timer for state estimation
        self.estimation_timer = self.create_timer(self.control_period, self.estimate_state)  # 100Hz update rate

    def imu_callback(self, msg):
        # Low pass filter the angular velocity
        measurement = np.array(
            [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            ]
        )
        self.plant_output = (
            1 - self.imu_lowpass_alpha
        ) * self.plant_output + self.imu_lowpass_alpha * measurement

        # High pass filter
        if self.previous_measurement is not None:
            self.plant_output = (
                self.plant_output
                - self.previous_measurement
                + self.imu_highpass_alpha * self.plant_output
            )
        
        self.previous_measurement = measurement

    def observer_callback(self, msg):
        self.A = np.array(msg.Ao.data).reshape(msg.Ao.rows, msg.Ao.cols)
        self.B = np.array(msg.Bo.data).reshape(msg.Bo.rows, msg.Bo.cols)
        self.C = np.array(msg.Co.data).reshape(msg.Co.rows, msg.Co.cols)
        self.L = np.array(msg.L.data).reshape(msg.L.rows, msg.L.cols)
        self.control_period = msg.dt
        self.estimation_timer.timer_period = self.control_period

    def motor_thrust_callback(self, msg):
        self.motor_thrust = np.array([motor.trhust for motor in msg.motors])

    def estimate_state(self):
        if self.A is not None and self.B is not None and self.C is not None and self.L is not None:
            # Compute the observer state
            self.state = self.A @ self.state + self.B @ self.motor_thrust
            self.state += self.L @ (self.plant_output - self.C @ self.state)

            # Publish the estimated state
            state_msg = Float32MultiArray()
            state_msg.data = self.plant_output.tolist()
            self.state_publisher.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    observer_node = ObserverNode()
    rclpy.spin(observer_node)
    observer_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()