#!/usr/bin/env python3
import rospy
import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseWithCovariance, TwistWithCovariance
import subprocess
import serial.tools.list_ports
import threading

# ✅ Your Arduino IMU's serial number
IMU_SERIAL_NUMBER = "343313236353516030E1"

class IMUOdometry:
    def __init__(self):
        # Initialize publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize odometry message
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.pose = PoseWithCovariance()
        self.odom.twist = TwistWithCovariance()
        
        # TF broadcaster
        # self.tf_broadcaster = tf.TransformBroadcaster()

    def publish_odometry(self, imu_msg):
        # Populate odometry message from IMU data
        self.odom.header.stamp = rospy.Time.now()
        
        # Orientation from IMU
        self.odom.pose.pose.orientation = imu_msg.orientation
        
        # Angular velocity (for twist)
        self.odom.twist.twist.angular = imu_msg.angular_velocity
        
        # Publish odometry
        self.odom_pub.publish(self.odom)
        
        # Publish TF transform (odom → base_link)
        # self.tf_broadcaster.sendTransform(
        #     (0, 0, 0),  # No position data from IMU
        #     (
        #         imu_msg.orientation.x,
        #         imu_msg.orientation.y,
        #         imu_msg.orientation.z,
        #         imu_msg.orientation.w
        #     ),
        #     rospy.Time.now(),
        #     "base_link",
        #     "odom"
        # )

def find_port_by_serial(serial_number):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.serial_number == serial_number:
            print(f"✅ Found Arduino with serial {serial_number} on port: {port.device}")
            return port.device
    print(f"❌ Arduino with serial {serial_number} not found.")
    return None

def run_arduino_imu_node(arduino_port):
    try:
        process = subprocess.Popen([
            "rosrun", 
            "rosserial_python", 
            "serial_node.py", 
            f"_port:={arduino_port}", 
            "_baud:=115200"
        ])
        print("🚀 IMU rosserial node started.")
        return process
    except Exception as e:
        print(f"❌ Failed to start rosserial node: {e}")
        return None

def imu_callback(msg, odom_handler):
    # Publish odometry and TF
    odom_handler.publish_odometry(msg)
    
    # Publish twist if needed (optional)
    twist = Twist()
    twist.angular.z = msg.angular_velocity.z
    odom_handler.twist_pub.publish(twist)

if __name__ == "__main__":
    arduino_port = find_port_by_serial(IMU_SERIAL_NUMBER)
    if not arduino_port:
        exit(1)

    # Start rosserial as background process
    rosserial_process = run_arduino_imu_node(arduino_port)
    if not rosserial_process:
        exit(1)

    try:
        rospy.init_node('imu_odometry_node')
        odom_handler = IMUOdometry()
        
        # Set covariance values (adjust based on your IMU's specs)
        odom_handler.odom.pose.covariance = [0.01]*36  # Position covariance
        odom_handler.odom.twist.covariance = [0.01]*36  # Velocity covariance
        
        rospy.Subscriber('/imu/data', Imu, imu_callback, odom_handler)
        rospy.loginfo("IMU Odometry node started!")
        rospy.spin()
        
    except KeyboardInterrupt:
        print("🛑 Shutting down...")
    finally:
        if rosserial_process:
            rosserial_process.terminate()
            rosserial_process.wait()