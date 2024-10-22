import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

# Define the green color range in HSV (adjust as needed)
lower_green = np.array([40, 50, 50])
upper_green = np.array([90, 255, 255])

# Initialize contrast and brightness values
alpha = 1  # Contrast control (1.0-3.0 or as needed)
beta = 0     # Brightness control (optional, range -100 to 100)

# Define the distance between camera and laser in meters
rho = 0.08

# Define the angle between the laser and the camera (in degrees)
angle1 = 75  # Example angle, adjust based on your setup
theta1 = math.radians(angle1)  # Convert angle to radians for trigonometry

# Define the horizontal field of view angle (in degrees)
FoV = 120  # Example value, adjust based on your camera's specifications
angle2 = FoV/2 
theta2 = math.radians(angle2)  # Convert to radians for trigonometry

# Initialize the video capture object
class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        self.bridge = CvBridge()
        self.cv_image = None
        self.processed_frame = None
        self.processed_frame_topic = '/processed_image'
        self.distance = None
        self.publisher_ = None
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Replace with your image topic
            self.image_callback,
            10)  # QoS (Quality of Service) profile depth

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')
        # Process the image
        self.distance, self.processed_frame = self.process_image(self.cv_image)
        
        # Publish the processed image
        # self.publisher_ = self.create_publisher(Image, 'processed_image', 10)
        # processed_image_msg = self.bridge.cv2_to_imgmsg(self.processed_frame, "bgr8")
        #self.publisher_.publish(processed_image_msg)
        # Publish the distance
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        distance_msg = Float32()
        distance_msg.data = self.distance
        self.publisher_.publish(distance_msg)

    def process_image(self, frame):
        # Capture frame-by-frame
        frame = image_converter.cv_image
        
        # Adjust contrast and brightness
        adjusted_frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
        
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(adjusted_frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask to filter only green colors
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # If contours are found, find the largest one (assuming it's the laser dot)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            # Only consider if the contour is large enough (to avoid noise)
            if radius > 2:
                # Draw the circle on the original frame
                center = (int(x), int(y))
                cv2.circle(adjusted_frame, center, int(radius), (0, 255, 0), 2)
                cv2.circle(adjusted_frame, center, 5, (0, 0, 255), -1)  # Mark the center of the laser dot
                
                # Calculate the displacement from the image center
                frame_height, frame_width = frame.shape[:2]
                deltamax = frame_width // 2  # X-center of the image (camera center)
                delta = x - deltamax  # Horizontal displacement in pixels
                
                # Calculate the distance from the camera using trigonometry and FoV
                # Using cotangent for theta1
                cot_theta1 = 1 / math.tan(theta1)
                distance = rho / (math.tan(theta2 * delta / deltamax) + cot_theta1)
                
                # Display the position and calculated distance
                cv2.putText(adjusted_frame, f"Position: {int(x)}, {int(y)}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(adjusted_frame, f"Distance: {distance:.2f} meters", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
            # Display the frame with contrast adjustment and laser tracking
            #    cv2.imshow('Laser Tracker (with Distance Calculation)', adjusted_frame)
                return distance, adjusted_frame
        return float('inf'), adjusted_frame


if __name__ == '__main__':
    rclpy.init()
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    rclpy.shutdown()