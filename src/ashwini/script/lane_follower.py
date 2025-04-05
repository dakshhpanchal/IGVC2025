#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower', anonymous=True)
        # Subscriber to the lane offset error published by lane_detection
        self.offset_sub = rospy.Subscriber("/lane_offset", Float32, self.offset_callback)
        # Publisher to send velocity commands to the robot
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel_lane", Twist, queue_size=10)
        # Set parameters for controlling the robot
        self.linear_speed = rospy.get_param("~linear_speed", 0)
        self.angular_gain = rospy.get_param("~angular_gain", 0.005)
        self.current_error = 0.0
        rospy.loginfo("Lane Follower node started.")

    def offset_callback(self, msg):
        self.current_error = msg.data

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = self.linear_speed
            # The angular velocity is proportional to the lane offset error.
            twist.angular.z = self.angular_gain * self.current_error
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        lane_follower = LaneFollower()
        lane_follower.run()
    except rospy.ROSInterruptException:
        pass
