#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

class Lidar:
    def __init__(self):
        rospy.init_node('lidar1')

        self.pub = rospy.Publisher('/lidar', LaserScan, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.loginfo("SmartScanFilter is running. Filtering to front 180 degrees (-90° to +90°)...")

    def scan_callback(self, msg):
        front_min = math.pi / 2       # +90°
        front_max = 3 * math.pi / 2   # +270°

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        total_readings = len(msg.ranges)

        start_idx = int((front_min - angle_min) / angle_inc)
        end_idx = int((front_max - angle_min) / angle_inc)

        if end_idx > total_readings:
            end_idx = total_readings

        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = angle_min + start_idx * angle_inc
        filtered_scan.angle_max = angle_min + (end_idx - 1) * angle_inc
        filtered_scan.angle_increment = angle_inc
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        filtered_scan.ranges = msg.ranges[start_idx:end_idx]

        if msg.intensities:
            filtered_scan.intensities = msg.intensities[start_idx:end_idx]

        self.pub.publish(filtered_scan)

if __name__ == '__main__':
    try:
        Lidar()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
