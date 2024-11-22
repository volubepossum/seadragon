import rclpy
from rclpy.node import Node
import csv
from std_msgs.msg import Float64


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Load the PWM-thrust mapping from the CSV file
        self.declare_parameter('thrust_mapping', 'thrust_mapping.csv')
        self.csv_file = self.get_parameter('thrust_mapping').get_parameter_value().string_value
        self.pwm_thrust_map = self.load_pwm_thrust_map(self.csv_file)

        # Create the subscription and publisher
        self.subscription = self.create_subscription(
            Float64,
            'motor_thrust',
            self.thrust_callback,
            10)
        self.publisher = self.create_publisher(Float64, 'servos_absolute_1', 10)

    def load_pwm_thrust_map(self, csv_file):
        pwm_thrust_map = {}
        with open(csv_file, mode='r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # Skip the header row
            for row in csv_reader:
                pwm, thrust = float(row[0]), float(row[1])
                pwm_thrust_map[thrust] = pwm
        return pwm_thrust_map

    def thrust_callback(self, msg):
        required_thrusts = msg.data
        pwm_values = [self.map_thrust_to_pwm(required_thrust) for required_thrust in required_thrusts]
        self.publisher.msg.data = ServoArray()
        servos=[Servo(servo=i, value=pwm_values[i]) for i in range(len(pwm_values))]
        self.publisher.publish()

    def map_thrust_to_pwm(self, thrust):
        closest_thrust = min(self.pwm_thrust_map.keys(), key=lambda k: abs(k - thrust))
        return int(round(self.pwm_thrust_map[closest_thrust]))

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()