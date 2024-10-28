#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np


def camera_sub_callback(msg: Image):
    frame = np.array(list(msg.data)).astype(np.uint8)
    frame = np.reshape(frame, (msg.height, msg.width, 3))
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    cv2.imshow("drone_camera", frame)
    cv2.waitKey(1)


def camera_sub():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, camera_sub_callback)
    rospy.spin()

if __name__ == '__main__':
    camera_sub()
    cv2.destroyAllWindows()
    