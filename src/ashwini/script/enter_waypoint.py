#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import time

waypoints = [
    (37.4275, -122.1697, 0.0),
    (37.4280, -122.1700, 0.0),
    (37.4285, -122.1703, 0.0)
]

def send_waypoints():
    rospy.init_node("waypoint_sender", anonymous=True)
    pub = rospy.Publisher("/waypoint_gps", NavSatFix, queue_size=10)
    rate = rospy.Rate(0.2)  # 0.2 Hz → one message every 5 seconds

    rospy.loginfo("Starting to send waypoints...")

    for idx, (lat, lon, alt) in enumerate(waypoints):
        if rospy.is_shutdown():
            break

        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "gps"

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        msg.status.status = 0
        msg.status.service = 1
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        pub.publish(msg)
        rospy.loginfo(f"Published Waypoint {idx+1}: Lat {lat}, Lon {lon}, Alt {alt}")
        rate.sleep()

    rospy.loginfo("All waypoints sent.")

if __name__ == "__main__":
    try:
        send_waypoints()
    except rospy.ROSInterruptException:
        pass

