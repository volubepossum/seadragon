import cv2
import numpy as np
import math

# Initialize the video capture object
cap = cv2.VideoCapture(0)  # 0 for webcam, or provide a video file path

# Define the green color range in HSV (adjust as needed)
lower_green = np.array([40, 50, 50])
upper_green = np.array([90, 255, 255])

# Initialize contrast and brightness values
alpha = 1  # Contrast control (1.0-3.0 or as needed)
beta = 0     # Brightness control (optional, range -100 to 100)

# Define the distance between camera and laser in meters
rho = 0.08

# Define the angle between the laser and the camera (in degrees)
angle1 = 70  # Example angle, adjust based on your setup
theta1 = math.radians(angle1)  # Convert angle to radians for trigonometry

# Define the horizontal field of view angle (in degrees)
FoV = 90  # Example value, adjust based on your camera's specifications
angle2 = FoV/2 
theta2 = math.radians(angle2)  # Convert to radians

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break
    
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
    cv2.imshow('Laser Tracker (with Distance Calculation)', adjusted_frame)
    
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()