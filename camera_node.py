#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('usb_camera_publisher', anonymous=True)
    pub = rospy.Publisher('/camera_view', Image, queue_size=10)
    bridge = CvBridge()

    # Open USB camera (adjust index if it's not /dev/video0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Cannot open USB camera")
        return

    rospy.loginfo("USB camera stream started...")

    rate = rospy.Rate(30)  # 30 FPS
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image")
            continue

        try:
            # Convert OpenCV image to ROS Image message
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(img_msg)
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()
