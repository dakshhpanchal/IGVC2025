#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MoveStraight:
    def __init__(self):
        rospy.init_node('move_straight', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.twist = Twist()
        self.obstacle_threshold = 0.3  # Stop if an obstacle is closer than 0.5m
        self.safe_to_move = True

        self.move_forward()

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)  # Get closest object
        rospy.loginfo(f"Min Distance: {min_distance:.2f}m")

        if min_distance < self.obstacle_threshold:
            self.safe_to_move = False  # Stop moving
        else:
            self.safe_to_move = True  # Continue moving

    def move_forward(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.safe_to_move:
                self.twist.linear.x = 1
                print("chala jaa bsdk")  # Move forward
            else:
                self.twist.linear.x = 0.0
                print("stop baby")  # Stop
            
            self.cmd_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        MoveStraight()
    except rospy.ROSInterruptException:
        pass
