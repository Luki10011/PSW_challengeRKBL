import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np



class Nodo(object):
    def __init__(self):
        self.frame = None
        self.loop_rate = rospy.Rate(100)
        rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.camera_sub_callback)

    def camera_sub_callback(self, msg: Image):
        img = np.array(list(msg.data)).astype(np.uint8)
        self.frame = np.reshape(img, (msg.height, msg.width, 3))
        frame_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("drone_camera", self.frame)
        img_aruco = self.decodeImage(self.frame)
        cv2.imshow("aruco detection", img_aruco)
        cv2.imwrite("frame", img_aruco)
        cv2.waitKey(1)

    def decodeImage(self, frame):
        marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        param_markers  = cv2.aruco.DetectorParameters_create()
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        param_markers.minMarkerPerimeterRate = 0.03  # Minimalny stosunek obwodu markera
        param_markers.maxMarkerPerimeterRate = 6.0  # Maksymalny stosunek obwodu markera
        param_markers.polygonalApproxAccuracyRate = 0.05  # Precyzja aproksymacji
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters = param_markers
        )
        # print(marker_corners)
        print(marker_IDs, reject)
        # corner1, corner2, corner3, corner4 = [marker[i][0] for i, marker in enumerate(marker_corners)]
        size = gray_frame.shape
        if marker_IDs is not None and len(marker_corners) > 0:
            # Assuming we are interested in the first marker
            corners = marker_corners[0][0]  # Get the corners of the first marker
            i_cor = list(set(corners[:, 1]))
            j_cor = list(set(corners[:, 0]))

            for i in range(size[0]):
                for j in range(size[1]):
                    if i == i_cor[0] or i == i_cor[1] or j == j_cor[0] or j == j_cor[1]:
                        gray_frame[i, j] = 100
            
        return gray_frame



    def camera_sub(self):
        #rospy.init_node("listener", anonymous=True)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("camera_test", anonymous=True)
    my_node = Nodo()
    my_node.camera_sub()
    cv2.destroyAllWindows()