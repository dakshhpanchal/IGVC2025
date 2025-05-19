#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("move_straight")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Use correct topic
rate = rospy.Rate(10)

twist = Twist()
twist.linear.x = 0.5  # Move forward

while not rospy.is_shutdown():
    pub.publish(twist)
    rate.sleep()

