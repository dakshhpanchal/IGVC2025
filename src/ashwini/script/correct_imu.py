#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    # Convert left-handed to right-handed by flipping Y and Z axes
    # msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = msg.linear_acceleration.z, msg.linear_acceleration.x, msg.linear_acceleration.y
    msg.linear_acceleration.x *= -1
    msg.linear_acceleration.z *= -1
    
    # Publish transformed message
    transformed_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('imu_frame_converter')
    sub = rospy.Subscriber('/camera/imu', Imu, imu_callback)
    transformed_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.spin()