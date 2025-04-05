#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class CmdMux:
    def __init__(self):
        rospy.init_node('cmd_mux', anonymous=True)
        self.lane_cmd = Twist()
        self.planner_cmd = Twist()

        self.w_lane = rospy.get_param("~lane_weight", 1)
        self.w_planner = rospy.get_param("~planner_weight", 1)

        rospy.Subscriber("/cmd_vel_lane", Twist, self.lane_callback)
        rospy.Subscriber("/cmd_vel_planner", Twist, self.planner_callback)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def lane_callback(self, msg):
        self.lane_cmd = msg

    def planner_callback(self, msg):
        self.planner_cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            combined = Twist()
            combined.linear.x =  1 * self.planner_cmd.linear.x
            combined.angular.z = 0.5 * self.lane_cmd.angular.z + 1 * self.planner_cmd.angular.z
            self.cmd_pub.publish(combined)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        mux = CmdMux()
        mux.run()
    except rospy.ROSInterruptException:
        pass
