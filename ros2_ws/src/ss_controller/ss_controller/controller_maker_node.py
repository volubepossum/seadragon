import rclpy
from rclpy.node import Node
from msg import Model, Controller, Matrix
import numpy as np
from scipy.linalg import solve_discrete_are, eigvals
from numpy.linalg import matrix_rank

class ControllerMakerNode(Node):
    def __init__(self):
        super().__init__('controller_maker_node')
        self.subscription = self.create_subscription(
            Model,
            '/model',
            self.model_callback,
            10)
        self.publisher = self.create_publisher(Controller, '/controller', 10)

        # init parameters
        # LQR gains
        self.Q = np.eye(13) * 1
        self.Q[8, 8] = 3
        if not np.all(np.linalg.eigvals(self.Q) >= 0):
            raise ValueError("Matrix Q is not positive semi-definite")
        self.R = np.eye(5) * 250
        if not np.all(np.linalg.eigvals(self.R) > 0):
            raise ValueError("Matrix R is not positive definite")

        self.get_logger().info('Controller Maker Node has been started.')

    def model_callback(self, msg) -> None:
        # Get the matrices from the message
        def to_matrix(matrix_msg) -> np.ndarray:
            return np.array(matrix_msg.data).reshape((matrix_msg.rows, matrix_msg.cols))

        A = to_matrix(msg.Ad)
        B = to_matrix(msg.Bd)
        C = to_matrix(msg.Cd)

        n_co = msg.n_co
        n__co = msg.n__co
        n_c_o = msg.n_c_o

        # get the observable states from kalman canonical form
        Ac = A[:n_c_o+n_co, :n_c_o+n_co]
        Bc = B[:n_c_o+n_co, :]
        Cc = np.hstack([ np.zeros((C.shape[0], n_c_o)), C[:, -n__co:]])

        # Create the controller
        controller = self.create_controller(Ac, Bc, Cc)

        # Publish the controller
        self.publisher.publish(controller)

    

    def create_controller(self, Ad, Bd, Cd) -> Controller:
        # https://www.egr.msu.edu/classes/me851/mukherji/11-DiscreteSystemControl.pdf
        # Solve the discrete-time algebraic Riccati equation (DARE)
        P = solve_discrete_are(Ad, Bd, self.Q, self.R)
        
        # Calculate the discrete LQR gain
        K = np.linalg.inv(self.R + Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
        
        # Calculate feedforward gain for reference tracking
        N = np.linalg.inv(Cd @ np.linalg.inv(np.eye(Ad.shape[0]) - Ad + Bd @ K) @ Bd)
 
        # Create the Controller message
        controller_msg = Controller()
        
        controller_msg.K.data = K.flatten().tolist()
        controller_msg.K.rows = K.shape[0]
        controller_msg.K.cols = K.shape[1]
        
        controller_msg.N.data = N.flatten().tolist()
        controller_msg.N.rows = N.shape[0]
        controller_msg.N.cols = N.shape[1]
        
        return controller_msg

def main(args=None):
    rclpy.init(args=args)
    node = ControllerMakerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()