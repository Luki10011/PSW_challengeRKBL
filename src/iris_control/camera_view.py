#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from aruco_pose_estimation import pose_estimation
import sys, time, math
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


pos_pub_topic = "/mavros/setpoint_position/local"
pos_sub_topic = "/mavros/local_position/pose"
pub = rospy.Publisher(pos_pub_topic, PoseStamped, queue_size=10)

challenge_started = False


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def camera_sub_callback(msg: Image):
    frame = np.array(list(msg.data)).astype(np.uint8)
    frame = np.reshape(frame, (msg.height, msg.width, 3))
    # frame = np.where()

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    param_markers  = cv2.aruco.DetectorParameters_create()
    matrix_coeff = np.array([[640.5098521801531, 0.0, 640.5],
                              [0.0, 640.5098521801531, 360.5],
                              [0.0, 0.0, 1.0]])
    dist_matrix = np.array([0, 0, 0, 0, 0]).astype(float)

    corners, ids, reject = cv2.aruco.detectMarkers(
        gray_frame, marker_dict, parameters = param_markers, cameraMatrix=matrix_coeff,
        distCoeff=dist_matrix
    )
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0
    # print(marker_corners)
    # corner1, corner2, corner3, corner4 = [marker[i][0] for i, marker in enumerate(marker_corners)]
    font = cv2.FONT_HERSHEY_PLAIN
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        
        ret= cv2.aruco.estimatePoseSingleMarkers(corners, .25, np.array(matrix_coeff),
                                                                    np.array(dist_matrix))  
        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] 
        cv2.aruco.drawDetectedMarkers(gray_frame, corners)
        cv2.aruco.drawAxis(gray_frame, matrix_coeff, dist_matrix, rvec, tvec, 0.25)
        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
        cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc    = R_ct.T
        #Get the attitude in terms of euler 321 (Needs to be flipped first)
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to camera frame
        str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                            math.degrees(yaw_marker))
        cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        pos_camera = -R_tc*np.matrix(tvec).T

        str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
        cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the frame
        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
        cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        print(str_position)
        #print(str_attitude)


    cv2.imshow("test", gray_frame)
    cv2.waitKey(1)






def camera_sub():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, camera_sub_callback)
    rospy.spin()

if __name__ == '__main__':
    camera_sub()
    # drone_control()
    cv2.destroyAllWindows()
    
