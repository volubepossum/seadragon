import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.linalg import null_space, expm
from msg import Model

class ModelEstimatorNode(Node):
    def __init__(self):
        super().__init__("model_estimator_node")
        self.subscription = self.create_subscription(
            Float32MultiArray, "/state", self.state_callback, 10
        )
        self.publisher = self.create_publisher(Model, "/model", 10)
        self.modeling_interval = 0.1

        self.modeling_timer = self.create_timer(self.modeling_interval, self.modeling_callback)

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
    
    def state_callback(self, msg) -> None:
        self.state = np.array(msg.data)

    def get_rotation_matrix(self, quaternion) -> np.ndarray:
        """Convert quaternion [w,x,y,z] to rotation matrix"""
        return R.from_quat(
            [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
        ).as_matrix()

    def compute_centripetal_matrix(self, angular_velocity) -> np.ndarray:
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

    def compute_transformation_matrix(self, quaternion) -> np.ndarray:
        """Compute transformation matrix T from quaternion"""
        return 0.5 * np.array(
            [
                [-quaternion[1], -quaternion[2], -quaternion[3]],
                [quaternion[0], -quaternion[3], quaternion[2]],
                [quaternion[3], quaternion[0], -quaternion[1]],
                [-quaternion[2], quaternion[1], quaternion[0]],
            ]
        )

    def assemble_system_matrices(self, x) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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
    
    def cont2discrete(self, A, B, C, dt) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Convert continuous state space model to discrete"""
        Ad = expm(A * dt)
        Bd = (Ad - np.eye(A.shape[0])) @ np.linalg.inv(A) @ B
        Cd = C
        return Ad, Bd, Cd
    
    def kalman_decomposition(self, A, B, C) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Step 1: Compute the controllability matrix
        n = A.shape[0]  # Number of states
        controllability_matrix = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
        r_c = np.linalg.matrix_rank(controllability_matrix)

        # Basis for the controllable subspace (V_c)
        U_c, _, _ = np.linalg.svd(controllability_matrix)
        V_c = U_c[:, :r_c]

        # Basis for the uncontrollable subspace (V_uc)
        V_uc = null_space(controllability_matrix)

        # Step 2: Compute the observability matrix
        observability_matrix = np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(n)])
        r_o = np.linalg.matrix_rank(observability_matrix)

        # Basis for the observable subspace (V_o)
        U_o, _, _ = np.linalg.svd(observability_matrix)
        V_o = U_o[:, :r_o]

        # Basis for the unobservable subspace (V_uo)
        V_uo = null_space(observability_matrix.T)

        # Step 3: Construct the transformation matrix T
        T = np.hstack([
            np.hstack([V_c @ V_o, V_c @ V_uo]),
            np.hstack([V_uc @ V_o, V_uc @ V_uo])
        ])

        # Verify that T is invertible
        if np.linalg.matrix_rank(T) < n:
            raise ValueError("The transformation matrix T is not invertible!")

        T_inv = np.linalg.inv(T)

        # Return the transformation matrix and its inverse
        return T, T_inv

    def modeling_callback(self):
        A, B, C = self.assemble_system_matrices(self.state)

        # Kalman decomposition
        T, T_inv = self.kalman_decomposition(A, B, C)
        A = T_inv @ A @ T
        B = T_inv @ B
        C = C @ T

        Ad, Bd, Cd = self.cont2discrete(A, B, C, self.control_interval)

        # Publish model matrices
        model_msg = Model()
        model_msg.dt = self.control_interval
        model_msg.Ad.data = Ad.flatten().tolist()
        model_msg.Ad.rows = Ad.shape[0]
        model_msg.Ad.cols = Ad.shape[1]
        model_msg.Bd.data = Bd.flatten().tolist()
        model_msg.Bd.rows = Bd.shape[0]
        model_msg.Bd.cols = Bd.shape[1]
        model_msg.Cd.data = Cd.flatten().tolist()
        model_msg.Cd.rows = Cd.shape[0]
        model_msg.Cd.cols = Cd.shape[1]
        model_msg.T.data = T.flatten().tolist()
        model_msg.T.rows = T.shape[0]
        model_msg.T.cols = T.shape[1]
        model_msg.T_inv.data = T_inv.flatten().tolist()
        model_msg.T_inv.rows = T_inv.shape[0]
        model_msg.T_inv.cols = T_inv.shape[1]

        self.publisher.publish(model_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModelEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
