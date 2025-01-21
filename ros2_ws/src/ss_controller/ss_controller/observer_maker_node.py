import rclpy
from rclpy.node import Node
from msg import Observer, Model
import numpy as np

class ObserverMakerNode(Node):
    def __init__(self):
        super().__init__('observer_maker_node')
        self.subscription = self.create_subscription(
            Model,
            '/model',
            self.model_callback,
            10)
        self.publisher = self.create_publisher(Observer, '/observer', 10)
        self.subscription  # prevent unused variable warning


    def model_callback(self, msg):
        # Assuming msg.data contains the Kalman canonical form matrices
        # Process the received model data and compute the observer matrices
        observer  = self.compute_observer_matrices(msg.A, msg.B, msg.C, msg.dt, msg.n_co, msg.n__co, msg.n_c_o, msg.n__c_o)
        self.publisher.publish(observer)

    def compute_observer_matrices(self, A, B, C, dt, n_co, n__co, n_c_o, n__c_o):
        # get the observable states from kalman canonical form
        Ao = np.block([
            [A[n_c_o:n__co, n_c_o:n__co], A[n_c_o:n__co, -n__co:]],
            [np.zeros((n__c_o - n_c_o, n__co - n_c_o)), A[-n__c_o:, -n__c_o:]]
            ])
        Bo = np.vstack(B[n_c_o:n_c_o + n_co, :], np.zeros((n__c_o - n_c_o, B.shape[1])))
        Co = np.hstack([C[:, n_c_o:n_c_o + n_co], C[:, -n__co:]])

        poles = np.ones(Ao.shape[0]) * - 1
        # compute the observer gain
        L = np.linalg.place(Ao.T, Co.T, poles).T
        
        observer = Observer()
        observer.L.data = L.flatten().tolist()
        observer.dt = dt

        observer.Ao.data = Ao.flatten().tolist()
        observer.Bo.data = Bo.flatten().tolist()
        observer.Co.data = Co.flatten().tolist()
        observer.Ao.rows = Ao.shape[0]
        observer.Ao.cols = Ao.shape[1]
        observer.Bo.rows = Bo.shape[0]
        observer.Bo.cols = Bo.shape[1]
        observer.Co.rows = Co.shape[0]
        observer.Co.cols = Co.shape[1]
        observer.L.rows = L.shape[0]
        observer.L.cols = L.shape[1]

        return observer
    
def main(args=None):
    rclpy.init(args=args)
    observer_maker_node = ObserverMakerNode()
    rclpy.spin(observer_maker_node)
    observer_maker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()