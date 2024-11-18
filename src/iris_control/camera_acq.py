import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
import cv2
import numpy as np

def camera_sub_callback(msg: CameraInfo):
    # frame = np.array(list(msg.data)).astype(np.uint8)
    # frame = np.reshape(frame, (msg.height, msg.width, 3))
    #frame = cv2.cvtColor(frame, cv2.COLORRGB2BGR)
    print(msg.distortion_model)
    print(msg.D)
    print(msg.K)
    
    # cv2.imshow("drone_camera", frame)
    # cv2.waitKey(1)


def camera_sub():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/iris/usb_cam/camera_info", Pose, camera_sub_callback)
    rospy.spin()

if __name__ == '__main__':
    camera_sub()
    