#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
from i2c_pwm_board_msgs.msg import ServoArray, Servo
from motor_controller.msg import Motors
import os
import ament_index_python
from time import sleep


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        # Load the PWM-thrust mapping from the CSV file
        self.declare_parameter("thrust_mapping", "config/thrust_mapping.csv")
        self.csv_file = (
            self.get_parameter("thrust_mapping").get_parameter_value().string_value
        )

        # Get the absolute path of the CSV file relative to the install location
        self.csv_file = os.path.join(
            ament_index_python.get_package_share_directory("motor_controller"),
            self.csv_file,
        )

        self.pwm_thrust_map = self.load_pwm_thrust_map(self.csv_file)
        self.motor_directions = [-1, 1, -1, 1, 1]

        # Create the subscription and publisher
        self.publisher = self.create_publisher(ServoArray, "servos_absolute_1", 10)
        # Initialize ESCs by publishing 0 PWM value to all servos
        self.send_0_pwm()

        self.get_logger().info("Initializing ESCs...")
        sleep(3)

        # create timer object to send 0 PWM value to all servos after inactivity
        self.timer = self.create_timer(0.2, self.send_0_pwm)

        self.subscription = self.create_subscription(
            Motors, "motor_thrust", self.thrust_callback, 10
        )

    def load_pwm_thrust_map(self, csv_file):
        pwm_thrust_map = {}
        with open(csv_file, mode="r") as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # Skip the header row
            for row in csv_reader:
                pwm, thrust = float(row[0]), float(row[1])
                pwm_thrust_map[thrust] = pwm
        return pwm_thrust_map

    def thrust_callback(self, msg):
        motors = msg.motors
        for motor in motors:
            # Find the closest thrust value in the map
            closest_thrust = min(
                self.pwm_thrust_map.keys(), key=lambda k: abs(k - motor.thrust*self.motor_directions[motor.id])
            )
            pwm = self.pwm_thrust_map[closest_thrust]
            self.get_logger().info(
                f"Motor {motor.id} gets PWM value {pwm} for thrust {motor.thrust}"
            )
            self.publisher.publish(
                ServoArray(servos=[Servo(servo=motor.id + 1, value=pwm)])
            )
        self.timer.reset()

    def send_0_pwm(self):
        initial_servo_array = ServoArray()
        initial_servo_array.servos = [Servo(servo=i + 1, value=self.pwm_thrust_map[0.0]) for i in range(5)]
        self.publisher.publish(initial_servo_array)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
