#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

class IMUOdometry:
    def __init__(self):
        rospy.init_node('realsense_imu_to_odom')
        
        # Initialize publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Initialize odometry message
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_link"
        self.odom.pose = PoseWithCovariance()
        self.odom.twist = TwistWithCovariance()
        
        # TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # Initialize with valid orientation (identity quaternion)
        self.last_valid_orientation = [0.0, 0.0, 0.0, 1.0]
        
        # Covariance setup (same as before)
        self.odom.pose.covariance = [0.1]*36
        self.odom.twist.covariance = [0.2]*36

    def is_valid_quaternion(self, q):
        """Check if quaternion is valid (non-zero and normalized)"""
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        return norm > 0.9 and norm < 1.1  # Allow some tolerance

    def imu_callback(self, imu_msg):
        # Update timestamp
        self.odom.header.stamp = rospy.Time.now()
        
        # Check and handle orientation data
        if self.is_valid_quaternion(imu_msg.orientation):
            self.last_valid_orientation = [
                imu_msg.orientation.x,
                imu_msg.orientation.y,
                imu_msg.orientation.z,
                imu_msg.orientation.w
            ]
        else:
            rospy.logwarn_once("Invalid IMU orientation received, using last valid orientation")
        
        # Use the (validated) orientation
        self.odom.pose.pose.orientation.x = self.last_valid_orientation[0]
        self.odom.pose.pose.orientation.y = self.last_valid_orientation[1]
        self.odom.pose.pose.orientation.z = self.last_valid_orientation[2]
        self.odom.pose.pose.orientation.w = self.last_valid_orientation[3]
        
        # Use angular velocity for twist
        self.odom.twist.twist.angular = imu_msg.angular_velocity
        
        # Publish odometry message
        self.odom_pub.publish(self.odom)
        
        # Publish TF transform only if we have valid data
        if any(v != 0 for v in self.last_valid_orientation):  # Check not all zeros
            self.tf_broadcaster.sendTransform(
                (0.0, 0.0, 0.0),  # Position remains zero
                self.last_valid_orientation,
                rospy.Time.now(),
                "base_link",
                "odom"
            )

if __name__ == "__main__":
    try:
        odom_converter = IMUOdometry()
        rospy.Subscriber('/imu', Imu, odom_converter.imu_callback)
        rospy.loginfo("RealSense IMU to Odometry converter started!")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down IMU to Odometry converter")