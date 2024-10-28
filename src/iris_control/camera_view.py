#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np


def camera_sub_callback(msg: Image):
    frame = np.array(list(msg.data)).astype(np.uint8)
    frame = np.reshape(frame, (msg.height, msg.width, 3))
    # frame = np.where()

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    param_markers  = cv2.aruco.DetectorParameters_create()
    

    corners, ids, reject = cv2.aruco.detectMarkers(
        gray_frame, marker_dict, parameters = param_markers
    )
    # print(marker_corners)
    # corner1, corner2, corner3, corner4 = [marker[i][0] for i, marker in enumerate(marker_corners)]
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # draw the bounding box of the ArUCo detection
            cv2.line(gray_frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(gray_frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(gray_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(gray_frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(gray_frame, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            cv2.putText(gray_frame, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
            
    cv2.imshow("test", gray_frame)
    cv2.waitKey(1)


def camera_sub():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, camera_sub_callback)
    rospy.spin()

if __name__ == '__main__':
    camera_sub()
    cv2.destroyAllWindows()
    