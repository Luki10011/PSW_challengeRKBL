#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys, time, math
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import UInt8


GATE_HEIGHT = 1  # TODO
GATE_WIDTH = 1  # TODO
MARKER_SIZE = 0.25
CAMERA_MATRIX = np.array([[640.5098521801531, 0.0, 640.5],
                              [0.0, 640.5098521801531, 360.5],
                              [0.0, 0.0, 1.0]])
CAMERA_DISTORTION = np.array([0, 0, 0, 0, 0]).astype(float)


target_topic = "/iris/target_pose"

pose_sub_topic = "/mavros/local_position/pose"
gate_sub_topic = "/iris/next_gate"
camera_sub_topic = '/iris/usb_cam/image_raw'

target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10)

next_gate = 1
current_drone_pose = Pose()


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

    corners, ids, reject = cv2.aruco.detectMarkers(
        gray_frame, marker_dict, parameters = param_markers, cameraMatrix=CAMERA_MATRIX,
        distCoeff=CAMERA_DISTORTION
    )
    R_flip  = np.zeros((3,3), dtype=np.float32)
    R_flip[0,0] = 1.0
    R_flip[1,1] =-1.0
    R_flip[2,2] =-1.0
    # print(marker_corners)
    # corner1, corner2, corner3, corner4 = [marker[i][0] for i, marker in enumerate(marker_corners)]
    font = cv2.FONT_HERSHEY_PLAIN
    if len(corners) > 0:
        for corner, id in zip(corners, ids):
            if int(id) == next_gate:
                ret= cv2.aruco.estimatePoseSingleMarkers(corner, .25, np.array(CAMERA_MATRIX),
                                                                            np.array(CAMERA_DISTORTION))  
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] 
                cv2.aruco.drawDetectedMarkers(gray_frame, corner)
                cv2.aruco.drawAxis(gray_frame, CAMERA_MATRIX, CAMERA_DISTORTION, rvec, tvec, 0.25)
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
                
                drone_rotation = current_drone_pose.orientation  # To jest w kwaterionie
                drone_position = current_drone_pose.position

                # TODO: Tutaj trzeba przeliczyć pos_camera na pozycję bezwględną w świecie za pomocą
                # wektora obrotu drona (drone_rotation) względem świata - zrzutować pos_camera na osie świate
                # i dodać pozycję drona, potem dodać jeszcze wymiary bramki na podstawie roll_camera, pitch_camera, yaw_camera

                target_x = ???
                target_y = ???
                target_z = ???

                # Obliczone wartości publikujemy jako PoseStamped w topicu target_topic
                # TODO: chcemy zadawać też rotację? Jaką i po co?

                target_msg = PoseStamped()
                target_msg.pose.position.x = target_x
                target_msg.pose.position.y = target_y
                target_msg.pose.position.z = target_z

                target_pub.publish()

    cv2.imshow("test", gray_frame)
    cv2.waitKey(1)


def pose_sub_callback(msg: PoseStamped):
    global current_drone_pose
    current_drone_pose = msg.pose


def gate_sub_callback(msg: UInt8):
    global next_gate
    next_gate = msg.data


def camera_sub():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber(camera_sub_topic, Image, camera_sub_callback)
    rospy.Subscriber(pose_sub_topic, PoseStamped, pose_sub_callback)
    rospy.Subscriber(gate_sub_topic, UInt8, gate_sub_callback)
    rospy.spin()


if __name__ == '__main__':
    camera_sub()
    # drone_control()
    cv2.destroyAllWindows()
    
