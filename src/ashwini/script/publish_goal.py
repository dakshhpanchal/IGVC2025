#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def gps_callback(msg):
    geo_point = GeoPointStamped()
    geo_point.header = msg.header
    geo_point.position.latitude = msg.latitude
    geo_point.position.longitude = msg.longitude
    geo_point.position.altitude = msg.altitude

    try:
        target_pose = PoseStamped()
        target_pose.header.frame_id = "utm"
        target_pose.header.stamp = rospy.Time.now()

        target_pose.pose.position.x = 0
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0
        target_pose.pose.orientation.w = 1.0

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(1.0)  

        transform = tf_buffer.lookup_transform("odom", "utm", rospy.Time(0), rospy.Duration(3.0))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose, transform)

        goal_pub.publish(transformed_pose)
        rospy.loginfo("Published goal to /move_base_simple/goal")

    except Exception as e:
        rospy.logwarn(f"Could not transform GPS -> odom: {e}")

if __name__ == "__main__":
    rospy.init_node("gps_to_goal_node")
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    rospy.Subscriber("/waypoint_gps", NavSatFix, gps_callback)
    rospy.spin()

