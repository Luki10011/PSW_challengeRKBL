#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Bool
from utils.utils import regulator_PD, calculate_area
from collections import deque

#"""DEFINITIONS START"""
"""ArUco parameters"""
MARKER_DICT = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_50)
PARAM_MARKERS  = cv.aruco.DetectorParameters_create()

"""ROSPY IMAGE -> cv.Image"""
BRIDGE = CvBridge()

"""World and camera parmeters"""
GATE_HEIGHT = 1  # TODO
GATE_WIDTH = 1  # TODO
MARKER_SIZE = 0.25
CAMERA_MATRIX = np.array([[640.5098521801531, 0.0, 640.5],
                              [0.0, 640.5098521801531, 360.5],
                              [0.0, 0.0, 1.0]])
CAMERA_DISTORTION = np.array([0, 0, 0, 0, 0]).astype(float)

"""Paths to topics"""
target_topic = "/iris/target_pose"
start_topic = "/iris_control/challenge_start"
pose_sub_topic = "/mavros/local_position/pose"
gate_sub_topic = "/iris/next_gate"
camera_sub_topic = '/iris/usb_cam/image_raw'
cmd_vel_topic = "/iris_control/cmd_vel"
velocity_sub_topic = "/mavros/local_position/velocity_local"
detect_gate_topic = "/iris/detected_gate"
pos_sub_topic = "/mavros/local_position/pose"
pose_topic = "/iris_control/pose"

"""ROSPY Publishers"""
target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10)
start_pub = rospy.Publisher(start_topic, Bool, queue_size=10)
vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
detect_gate_pub = rospy.Publisher(detect_gate_topic, UInt8, queue_size=10)

"""Global variables"""
current_altitude = 0
frame_cnt = 0
next_gate = 1

"""Flags"""
aruco_area_flag = False
frame_cnt_flag = False
challenge_started = False

"""Constants"""
ARUCO_AREA_THRESH = 2500
FRAME_CNT_THRESH = 7
QUEUE_LEN = 2

"""Global lists and dicts"""
queue_col = deque(np.zeros(shape=(0),dtype=np.int16),maxlen=QUEUE_LEN)
queue_row = deque(np.zeros(shape=(0),dtype=np.int16),maxlen=QUEUE_LEN)
reg_err = np.zeros(shape=(10,))
gates = {
    1: None,
    2: None,
    3: None,
    4: None,
    5: None
}
#"""DEFINITIONS END"""


def start_challenge_callback(msg: Bool):
    global challenge_started
    if msg.data:
        rospy.loginfo("Challenge Started!")
        challenge_started = True


def camera_sub_callback(msg: Image):
    global frame_cnt, next_gate 
    global frame_cnt_flag, aruco_area_flag, was_next_gate_in
    global reg_err, drone_position 
    frame = BRIDGE.imgmsg_to_cv2(msg,"passthrough")
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    corners, ids, reject = cv.aruco.detectMarkers(
        gray_frame, MARKER_DICT, 
        parameters = PARAM_MARKERS, 
        cameraMatrix=CAMERA_MATRIX,
        distCoeff=CAMERA_DISTORTION
    )

    #rospy.loginfo(f"Current gate = {next_gate} | Found gates = {ids}")
    
    was_next_gate_in = False
    if len(corners) > 0: 
        for corner, id in zip(corners, ids):
            id = int(id)
            corner = [corner]
            ret= cv.aruco.estimatePoseSingleMarkers(corner, .25, CAMERA_MATRIX, CAMERA_DISTORTION)  
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            aruco_center = np.zeros(shape=(3,1))
            aruco_center[0] -= GATE_WIDTH 
            aruco_center[1] += GATE_HEIGHT
            
            goal_center, _____ = cv.projectPoints(aruco_center, rvec, tvec, CAMERA_MATRIX,CAMERA_DISTORTION)
            goal_center = (int(goal_center[0][0][0]), int(goal_center[0][0][1]))

            if id == next_gate:
                was_next_gate_in = True
                frame_cnt = 0

                queue_col.append(goal_center[0])
                queue_row.append(goal_center[1])
                col_mean = np.mean(queue_col,dtype="int16")
                row_mean = np.mean(queue_row,dtype="int16")

                gates[id] = (col_mean, row_mean)
                aruco_area_flag = calculate_area(corner[0][0]) > ARUCO_AREA_THRESH
                cv.circle(gray_frame, (col_mean, row_mean), 5, (0,255,0),-1)
                cv.aruco.drawDetectedMarkers(gray_frame, corner)
                cv.aruco.drawAxis(gray_frame, CAMERA_MATRIX, CAMERA_DISTORTION, rvec, tvec, 0.25)
                cv.putText(gray_frame, f"{id}", (10, 120), cv.FONT_HERSHEY_PLAIN, 4, (0, 255, 0), 2, cv.LINE_AA)

            else:
                cv.circle(gray_frame, goal_center, 5, (0,255,0),-1)
                gates[id] = goal_center

    # #TODO: next_gate != 5 is a temporary solution
    # #aruco_area_flag is sort of useless (valid area, changes depending on angle)
    if frame_cnt_flag and aruco_area_flag and next_gate != 5:
        next_gate += 1
        frame_cnt_flag = False 
        aruco_area_flag = False
        queue_col.clear()
        queue_row.clear()
    elif not was_next_gate_in:
        frame_cnt += 1
        if frame_cnt > FRAME_CNT_THRESH:
            frame_cnt_flag = True
            frame_cnt = 0

    cv.circle(gray_frame, (640,360) , 5, (0,255,0),1)
    cv.imshow("test", gray_frame)
    cv.waitKey(1)


def pose_callback(msg: Pose):
    global current_altitude
    #rospy.loginfo(f"DRONE POSE:\nPosition:\n{msg.position}\nOrientation:\n{msg.orientation}")
    current_altitude = msg.position.z


def velocity_control(event):
    global challenge_started
    global vel_pub, current_altitude
    global gates, next_gate, frame_cnt
    if challenge_started:
        z_val, y_val = gates[next_gate]
        next = next_gate + 1 if next_gate < 5 else 5
        new_vel = Twist()
        new_vel.linear.x = 2
        new_vel.linear.y = 0
        new_vel.linear.z, reg_err[2] = regulator_PD(2, 5,
                                        val = current_altitude, e_prev = reg_err[2],
                                        setpoint = 2.75, saturation = 10)
        new_vel.angular.x = 0
        new_vel.angular.y = 0
        new_vel.angular.z, reg_err[5] = regulator_PD(0.0075, 1.5,
                                        val = z_val, e_prev = reg_err[5],
                                        setpoint = 640, saturation = 10)
        rospy.loginfo(f"\ncurrent_altitude = {current_altitude}\n"
                      f"linear.z = {new_vel.linear.z}, error = {reg_err[2]}\n"
                      f"angular.z = {new_vel.angular.z}, error = {reg_err[5]}\n"
                      f"Gate = {gates[next_gate]}\n"
                      f"Next = {gates[next]}"
                      f"Frame cnt = {frame_cnt}")

        vel_pub.publish(new_vel)


def camera_sub():
    rospy.init_node("target_position", anonymous=True)
    rospy.Subscriber(start_topic, Bool, start_challenge_callback)
    rospy.Subscriber(camera_sub_topic, Image, camera_sub_callback)
    rospy.Subscriber(pose_topic, Pose, pose_callback)
    rospy.Timer(rospy.Duration(secs=0,nsecs=100),velocity_control)
    rospy.spin()


if __name__ == '__main__':
    camera_sub()
    cv.destroyAllWindows()
    
