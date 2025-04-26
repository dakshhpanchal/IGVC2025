#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

def get_dummy_camera_info(width=640, height=480):
    info = CameraInfo()
    info.width = width
    info.height = height
    info.K = [525, 0, width / 2,
              0, 525, height / 2,
              0, 0, 1]
    info.P = [525, 0, width / 2, 0,
              0, 525, height / 2, 0,
              0, 0, 1, 0]
    info.R = [1, 0, 0,
              0, 1, 0,
              0, 0, 1]
    info.D = [0, 0, 0, 0, 0]
    info.distortion_model = "plumb_bob"
    return info

def main():
    rospy.init_node('camera_publisher', anonymous=True)

    image_pub = rospy.Publisher('/ashwini/camera/rgb/image_raw', Image, queue_size=10)
    info_pub = rospy.Publisher('/ashwini/camera/rgb/camera_info', CameraInfo, queue_size=10)

    bridge = CvBridge()
    cap = cv2.VideoCapture(2)  # Use 1 if external camera

    if not cap.isOpened():
        rospy.logerr("Error: Cannot open camera.")
        return

    # Get width and height of the frame
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    camera_info = get_dummy_camera_info(width, height)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                timestamp = rospy.Time.now()

                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = "camera"

                camera_info.header.stamp = timestamp
                camera_info.header.frame_id = "camera"

                image_pub.publish(img_msg)
                info_pub.publish(camera_info)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: %s", e)
        else:
            rospy.logwarn("Failed to capture image from camera.")
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass