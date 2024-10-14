import os
import atexit
import subprocess
from time import sleep
import rospy
from std_msgs.msg import String

def start_camera():
    subprocess.Popen(["rpicam-vid", "-t", "0", "--camera", "0", "--nopreview", "--codec", "yuv420", "--width", "1280", "--height", "720", "--inline", "--listen", "-o", "./camera_pipe"])

def start_camera_server():
    subprocess.Popen(["./mediamtx"])

def start_topic():
    def start_topic():
        rospy.init_node('camera_stream_publisher', anonymous=True)
        pub = rospy.Publisher('camera_stream', String, queue_size=10)
        rate = rospy.Rate(10)  # 10hz

        def read_camera_pipe():
            with open('./camera_pipe', 'rb') as pipe:
                while not rospy.is_shutdown():
                    data = pipe.read(1024)
                    if data:
                        pub.publish(data)
                    rate.sleep()

        read_camera_pipe()

def stop_camera():
    os.system("killall rpicam-vid")

def stop_server():
    os.system("killall mediamtx")
    
def cleanup():
    stop_camera()
    stop_server()

def main():
    stop_camera()
    stop_server()
    start_camera()
    sleep(1)
    start_camera_server()
    while True:
        pass

if __name__ == "__main__":
    main()
    atexit.register(cleanup)