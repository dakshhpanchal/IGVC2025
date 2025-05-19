#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from tf.transformations import quaternion_from_euler
import math

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('alligator')
        
        # List of waypoints (x, y, yaw_deg)
        self.waypoints = [
            (5.0, 5.0, 0),
            (-5.0, 5.0, 0),
            (-5.0, -5.0, 0),
            (5.0, -5.0, 0)
        ]
        
        self.current_waypoint_index = 0
        self.goal_active = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait for the action server to come up
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Could not connect to move_base server")
            return
            
        rospy.loginfo("Connected to move_base server")
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.goal_active:
            rospy.logwarn("Previous goal still active, not sending new waypoint")
            return
            
        if self.current_waypoint_index < len(self.waypoints):
            x, y, yaw_deg = self.waypoints[self.current_waypoint_index]
            self.send_waypoint(x, y, yaw_deg)
        else:
            rospy.loginfo("All waypoints completed!")

    def send_waypoint(self, x, y, yaw_deg=0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0

        yaw = math.radians(yaw_deg)
        q = quaternion_from_euler(0, 0, yaw)

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"Sending waypoint {self.current_waypoint_index + 1}: x={x}, y={y}, yaw={yaw_deg}")
        self.goal_active = True
        self.client.send_goal(goal, done_cb=self.goal_reached_callback)
        self.current_waypoint_index += 1

    def goal_reached_callback(self, status, result):
        self.goal_active = False
        
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index} reached!")
        else:
            rospy.logwarn(f"Failed to reach waypoint {self.current_waypoint_index}")
        
        # Use a timer to add a small delay before sending next waypoint
        rospy.Timer(rospy.Duration(0.5), self.delayed_next_waypoint, oneshot=True)

    def delayed_next_waypoint(self, event):
        self.send_next_waypoint()

if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
