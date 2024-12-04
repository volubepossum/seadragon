#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from motor_controller.msg import Motors, Motor
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.linalg import solve_continuous_are
import sympy as sp
from scipy.linalg import schur, rsf2csf


class SSControllerNode(Node):
    def __init__(self):
        super().__init__("ss_controller_node")

        self.get_logger().info("SS Controller Node is starting")
        self.motors_publisher = self.create_publisher(Motors, "motor_thrust", 10)

        self.get_logger().info("Initializing variables")
        self.K = np.zeros((5, 14))
        self.N = np.zeros((5, 14))
        self.state = np.zeros(
            14
        )  # u, v, w, p, q, r, x, y, z qw, qx, qy, qz, extra_forces
        self.state[9] = 1
        self.state[10] = 0
        self.state[11] = 0
        self.state[12] = 0
        # Normalize quaternion
        quat_norm = np.linalg.norm(self.state[9:13])
        self.state[9:13] /= quat_norm
        self.state[13] = 1  # extra_forces coefficient
        self.reference = np.zeros(14)
        self.U = Motors()
        self.U.motors = [Motor(id=i) for i in range(5)]

        self.get_logger().info("Subscribing to topics")
        self.subscriptions_list = [
            self.create_subscription(Imu, "/imu_data", self.listener_callback, 10),
            # self.create_subscription(MagneticField, "/imu_data_mag", self.listener_callback, 10),
            # self.create_subscription(MagneticField, "/imu_data_rpy", self.listener_callback, 10),
            self.create_subscription(
                Float64MultiArray, "/reference", self.reference_callback, 10
            ),
        ]

        self.get_logger().info("Initializing controller")
        # Define physical parameters
        self.g = 9.81
        self.m = 4
        self.Ix = 1.1
        self.Iyz = 1.9
        self.Dx = 0.1
        self.Dyz = 0.2
        self.Drx = 0.1
        self.Dryz = 0.2
        self.I = np.diag([self.Ix, self.Iyz, self.Iyz])
        self.D = np.diag([self.Dx, self.Dyz, self.Dyz, self.Drx, self.Dryz, self.Dryz])
        self.M = np.block(
            [[self.m * np.eye(3), np.zeros((3, 3))], [np.zeros((3, 3)), self.I]]
        )
        self.M_inv = np.linalg.inv(self.M)
        self.R = np.array(
            [[0.3, 0.3, 0, 0, -0.3], [-0.1, 0.1, -0.1, 0.1, 0], [0, 0, 0, 0, 0]]
        )
        self.F = np.array([[0, 0, 1, 1, 0], [0, 0, 0, 0, 0], [-1, -1, 0, 0, -1]])
        self.Tau = np.vstack((self.F, np.linalg.cross(self.R, self.F, axis=0)))
        self.Weig = np.array([0, 0, self.m * self.g])
        self.Bouy = np.array([0, 0, -self.m * self.g * 1.01])
        self.r_b = np.array([0.05, 0, 0])
        # Define symbolic variables
        x, y, z, q0, q1, q2, q3 = sp.symbols("x y z q0 q1 q2 q3")
        nu_sym = sp.Matrix([x, y, z, q0, q1, q2, q3])

        # Define the rotation matrix using symbolic variables
        Rot_sym = sp.Matrix(
            [
                [
                    1 - 2 * (q2**2 + q3**2),
                    2 * (q1 * q2 - q0 * q3),
                    2 * (q1 * q3 + q0 * q2),
                ],
                [
                    2 * (q1 * q2 + q0 * q3),
                    1 - 2 * (q1**2 + q3**2),
                    2 * (q2 * q3 - q0 * q1),
                ],
                [
                    2 * (q1 * q3 - q0 * q2),
                    2 * (q2 * q3 + q0 * q1),
                    1 - 2 * (q1**2 + q2**2),
                ],
            ]
        ).T

        # Define the symbolic expressions for g and G
        Weig_sym = sp.Matrix(self.Weig)
        Bouy_sym = sp.Matrix(self.Bouy)
        r_b_sym = sp.Matrix(self.r_b)
        g_sym = sp.Matrix.vstack(
            Rot_sym @ (Weig_sym + Bouy_sym), r_b_sym.cross(Rot_sym @ Bouy_sym)
        )
        G_sym = g_sym.jacobian(nu_sym)
        # Convert sympy expressions to lambda functions for numerical evaluation
        self.g = sp.lambdify(nu_sym, g_sym, "numpy")
        self.G = sp.lambdify(nu_sym, G_sym, "numpy")

        # LQR gains
        self.Q = np.eye(14)
        self.Q[6:, 6:] = 0
        self.Q[1, 1] = 0
        if not np.all(np.linalg.eigvals(self.Q) >= 0):
            raise ValueError("Matrix Q is not positive semi-definite")
        self.R = np.eye(5)
        if not np.all(np.linalg.eigvals(self.R) > 0):
            raise ValueError("Matrix R is not positive definite")
        self.Trans = np.eye(14)
        self.K = np.zeros((5, 14))
        self.N = np.zeros((5, 14))
        # start controller
        self.get_logger().info("Starting controller")
        self.controller_update_period = 0.1  # seconds
        self.update_timer = (
            self.create_timer(self.controller_update_period, self.update_controller),
        )
        self.update_controller()  # Initialize controller
        self.control_period = 0.01  # seconds
        self.control_timer = self.create_timer(self.control_period, self.control)
        self.get_logger().info("SS Controller Node has been started")

    def control(self):
        u = -self.K @ self.state + self.N @ self.reference
        for i in range(5):
            self.U.motors[i].thrust = u[i]
        # Publish motor commands
        self.motors_publisher.publish(self.U)

    def update_controller(self):
        success = False
        for _ in range(10):
            try:
                A, B, T, self.k = self.compute_state_space_matrices()
                self.K[:, : self.k], self.N[:, : self.k] = self.lqr(
                    A[: self.k, : self.k],
                    B[: self.k, :],
                    self.Q[: self.k, : self.k],
                    self.R,
                )
                success = True
                # self.get_logger().info("LQR computation successful")
                break
            except:
                # self.get_logger().warn("LQR computation failed, retrying...")
                pass
        if not success:
            self.get_logger().error("LQR computation failed, exiting...")
            return
        self.K[self.k:, self.k :] = 0
        self.N[:, self.k :] = 0

    def compute_state_space_matrices(self):
        def smtrx(v):
            return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

        Centri = np.block(
            [
                [self.m * smtrx(self.state[3:6]), np.zeros((3, 3))],
                [np.zeros((3, 3)), -smtrx(self.I @ self.state[3:6])],
            ]
        )
        G = self.G(*self.state[6:13])
        g = self.g(*self.state[6:13]) - (G @ self.state[6:13]).reshape(-1, 1)
        R = np.array(
            [
                [
                    1 - 2 * (self.state[11] ** 2 + self.state[12] ** 2),
                    2
                    * (
                        self.state[10] * self.state[11] - self.state[9] * self.state[12]
                    ),
                    2
                    * (
                        self.state[10] * self.state[12] + self.state[9] * self.state[11]
                    ),
                ],
                [
                    2
                    * (
                        self.state[10] * self.state[11] + self.state[9] * self.state[12]
                    ),
                    1 - 2 * (self.state[10] ** 2 + self.state[12] ** 2),
                    2
                    * (
                        self.state[11] * self.state[12] - self.state[9] * self.state[10]
                    ),
                ],
                [
                    2
                    * (
                        self.state[10] * self.state[12] - self.state[9] * self.state[11]
                    ),
                    2
                    * (
                        self.state[11] * self.state[12] + self.state[9] * self.state[10]
                    ),
                    1 - 2 * (self.state[10] ** 2 + self.state[11] ** 2),
                ],
            ]
        )
        T = 0.5 * np.array(
            [
                [-self.state[10], -self.state[11], -self.state[12]],
                [self.state[9], -self.state[12], self.state[11]],
                [self.state[12], self.state[9], -self.state[10]],
                [-self.state[11], self.state[10], self.state[9]],
            ]
        )
        A = np.block(
            [
                [
                    self.M_inv @ (-Centri - self.D),
                    self.M_inv @ G,
                    self.M_inv @ g,
                ],
                [
                    R,
                    np.zeros((3, 11)),
                ],
                [
                    np.zeros((4, 3)),
                    T,
                    np.zeros((4, 8)),
                ],
                [np.zeros((1, 14))],
            ]
        )
        # B bottom should be zeros, but we are faking controlability
        B = np.block([[self.M_inv @ self.Tau], [np.zeros((8, 5))]])
        # C = np.block([
        #     [np.zeros((5, 6)), np.eye(5)]
        # ]) not needed, we are not using a state observer

        # Decompose the system into controllable and uncontrollable partss
        def ctrbf(A, B, tol=1e-9):
            # https://www.cim.mcgill.ca/~boulet/304-501A/L22.pdf
            n = A.shape[0]
            Q = self.compute_controllability_matrix(A, B)
            k = np.linalg.matrix_rank(Q)
            Q, _ = np.linalg.qr(Q)
            T = Q[:, :k]
            T /= np.linalg.norm(T, axis=0)
            while T.shape[1] < n:
                orthogonal_vector = np.random.rand(n, 1)
                orthogonal_vector -= T @ (T.T @ orthogonal_vector)
                orthogonal_vector /= np.linalg.norm(orthogonal_vector)
                T = np.hstack((T, orthogonal_vector))
            T[np.abs(T) < tol] = 0
            T_inv = np.linalg.inv(T)
            Abar = T_inv @ A @ T
            Bbar = T_inv @ B
            return Abar, Bbar, T, k

        Abar, Bbar, T, k = ctrbf(A, B)
        A = Abar
        B = Bbar
        return A, B, T, k

    def lqr(self, A, B, Q, R):
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        N = np.linalg.inv(R) @ B.T
        return K, N

    def compute_controllability_matrix(self, A, B):
        n = A.shape[0]
        controllability_matrix = B
        for i in range(1, n):
            controllability_matrix = np.hstack(
                (controllability_matrix, np.linalg.matrix_power(A, i) @ B)
            )
        return controllability_matrix

    def is_controllable(self, A, B):
        controllability_matrix = self.compute_controllability_matrix(A, B)
        rank = np.linalg.matrix_rank(controllability_matrix, tol=None)
        return rank == A.shape[0], rank, A.shape[0]

    def listener_callback(self, msg):
        # Update state based on IMU data
        # Integrate linear acceleration to get velocity
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if hasattr(self, "previous_time"):
            dt = current_time - self.previous_time
            self.state[0] += (
                0.5
                * (msg.linear_acceleration.x + self.previous_linear_acceleration_x)
                * dt
            )
            self.state[1] += (
                0.5
                * (msg.linear_acceleration.y + self.previous_linear_acceleration_y)
                * dt
            )
        else:
            dt = 0
        self.previous_time = current_time
        self.previous_linear_acceleration_x = msg.linear_acceleration.x
        self.previous_linear_acceleration_y = msg.linear_acceleration.y
        self.state[3] = msg.angular_velocity.x
        self.state[4] = msg.angular_velocity.y
        self.state[5] = msg.angular_velocity.z
        self.state[9] = msg.orientation.w
        self.state[10] = msg.orientation.x
        self.state[11] = msg.orientation.y
        self.state[12] = msg.orientation.z

    def reference_callback(self, msg):
        self.reference = np.array(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SSControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
