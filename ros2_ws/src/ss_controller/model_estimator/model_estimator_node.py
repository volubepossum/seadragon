import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R
import scipy.linalg
from msg import Model


class ModelEstimatorNode(Node):
    def __init__(self):
        super().__init__("model_estimator_node")
        self.subscription = self.create_subscription(
            Float32MultiArray, "/state", self.state_callback, 10
        )
        self.publisher = self.create_publisher(Model, "/model", 10)
        self.control_interval = 0.01

        # System parameters
        self.g = 9.81
        self.m = 8.77
        self.Ix = 0.0146
        self.Iyz = 0.0585
        self.Dx = 8
        self.Dyz = 16
        self.Drx = 0.037 / 3
        self.Dryz = 0.037
        self.I = np.diag([self.Ix, self.Iyz, self.Iyz])
        self.D = np.diag([self.Dx, self.Dyz, self.Dyz, self.Drx, self.Dryz, self.Dryz])
        self.M = np.block(
            [[self.m * np.eye(3), np.zeros((3, 3))], [np.zeros((3, 3)), self.I]]
        )
        self.M_inv = np.linalg.inv(self.M)
        self.R = np.array(
            [[0.31, 0.31, 0, 0, -0.47], [-0.11, 0.11, -0.11, 0.11, 0], [0, 0, 0, 0, 0]]
        )
        self.F = np.array([[0, 0, 1, 1, 0], [0, 0, 0, 0, 0], [-1, -1, 0, 0, -1]])
        self.Tau = np.vstack((self.F, np.linalg.cross(self.R, self.F, axis=0)))

    def get_rotation_matrix(self, quaternion):
        """Convert quaternion [w,x,y,z] to rotation matrix"""
        return R.from_quat(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
        ).as_matrix()

    def compute_centripetal_matrix(self, angular_velocity):
        """Compute centripetal matrix from angular velocity"""

        def skew_symmetric(v):
            return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

        w_skew = skew_symmetric(angular_velocity)
        return np.block(
            [
                [self.m * w_skew, np.zeros((3, 3))],
                [np.zeros((3, 3)), -skew_symmetric(self.I @ angular_velocity)],
            ]
        )

    def compute_transformation_matrix(self, quaternion):
        """Compute transformation matrix T from quaternion"""
        return 0.5 * np.array(
            [
                [-quaternion[1], -quaternion[2], -quaternion[3]],
                [quaternion[0], -quaternion[3], quaternion[2]],
                [quaternion[3], quaternion[0], -quaternion[1]],
                [-quaternion[2], quaternion[1], quaternion[0]],
            ]
        )

    def assemble_system_matrices(self, x):
        """Assemble A, B, C matrices of the state space model"""
        # Extract quaternion and angular velocity
        quaternion = x[9:13]  # [w,x,y,z]
        angular_velocity = x[3:6]

        # Get component matrices
        R_mat = self.get_rotation_matrix(quaternion)
        Centri = self.compute_centripetal_matrix(angular_velocity)
        T = self.compute_transformation_matrix(quaternion)

        # Assemble A matrix
        A = np.block(
            [
                [self.M_inv @ (-Centri - self.D), np.zeros((6, 7))],
                [R_mat, np.zeros((3, 10))],
                [np.zeros((4, 3)), T, np.zeros((4, 7))],
            ]
        )

        # Assemble B matrix
        B = np.block([[self.M_inv @ self.Tau], [np.zeros((7, 5))]])

        # Assemble C matrix
        C = np.block([[np.zeros((5, 8)), np.eye(5)]])

        return A, B, C
    
    def cont2discrete(self, A, B, C, dt):
        """Convert continuous state space model to discrete"""
        Ad = scipy.linalg.expm(A * dt)
        Bd = (Ad - np.eye(A.shape[0])) @ np.linalg.inv(A) @ B
        Cd = C
        return Ad, Bd, Cd

    def state_callback(self, msg):
        x = np.array(msg.data)
        A, B, C = self.assemble_system_matrices(x)
        Ad, Bd, Cd = self.cont2discrete(A, B, C, self.control_interval)
        # Publish model matrices
        model_msg = Model()
        model_msg.A = A.flatten().tolist()
        model_msg.B = B.flatten().tolist()
        model_msg.C = C.flatten().tolist()
        model_msg.dt = self.control_interval
        model_msg.Ad = Ad.flatten().tolist()
        model_msg.Bd = Bd.flatten().tolist()
        model_msg.Cd = Cd.flatten().tolist()

        self.publisher.publish(model_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModelEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
