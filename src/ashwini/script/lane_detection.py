#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ashwini/camera/rgb/image_raw", Image, self.image_callback)
        self.offset_pub = rospy.Publisher("/lane_offset", Float32, queue_size=10)
        rospy.loginfo("Lane Detector initialized using ROI + Contour + Centroid method")

    def image_callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return

        height, width, _ = frame.shape

        # 1. Crop ROI (bottom quarter of image)
        roi = frame[int(height * 0.6):, :]

        # 2. Convert to HSV and threshold for white lanes
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 55, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # 3. Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:  # ignore small noise
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy))
                    # Draw for debug
                    cv2.circle(roi, (cx, cy), 5, (0, 255, 0), -1)

        # 4. Find left and right lane centroids
        if len(centroids) >= 2:
            centroids = sorted(centroids, key=lambda x: x[0])  # sort by x
            left_lane = centroids[0]
            right_lane = centroids[-1]
            lane_center = (left_lane[0] + right_lane[0]) // 2
            image_center = width // 2
            error = image_center - lane_center
            self.offset_pub.publish(Float32(error))

            # Draw info
            cv2.line(roi, (lane_center, 0), (lane_center, roi.shape[0]), (255, 0, 0), 2)
            cv2.line(roi, (image_center, 0), (image_center, roi.shape[0]), (0, 0, 255), 2)
            rospy.loginfo(f"Lane center: {lane_center}, Error: {error}")
        else:
            rospy.logwarn("Did not find two lane lines")

        # Show for debug
        cv2.imshow("Lane ROI", roi)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        detector = LaneDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
