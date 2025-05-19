#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MoveAroundObstacle:
    def __init__(self):
        rospy.init_node('move_around_obstacle', anonymous=True)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.twist = Twist()
        self.obstacle_threshold = 1  # Stop if an obstacle is closer than 0.5m
        self.safe_to_move = True

        self.navigate()

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)  # Get closest object
        rospy.loginfo(f"Min Distance: {min_distance:.2f}m")

        if min_distance < self.obstacle_threshold:
            self.safe_to_move = False  # Start turning
        else:
            self.safe_to_move = True  # Move forward

    def navigate(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.safe_to_move:
                self.twist.linear.x = 0.2  # Move forward
                self.twist.angular.z = 0.0  # No rotation
            else:
                self.twist.linear.x = 0.0  # Stop moving forward
                self.twist.angular.z = 0.5  # Turn left
            
            self.cmd_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        MoveAroundObstacle()
    except rospy.ROSInterruptException:
        pass
