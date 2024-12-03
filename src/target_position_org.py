#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import sys, time, math
from geometry_msgs.msg import PoseStamped, Pose, Twist, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import UInt8, Bool
from utils.utils import regulator_PD, calculate_area
from collections import deque

FONT_OF_TEXT = cv2.FONT_HERSHEY_PLAIN
MARKER_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
PARAM_MARKERS  = cv2.aruco.DetectorParameters_create()

BRIDGE = CvBridge()

GATE_HEIGHT = 1  # TODO
GATE_WIDTH = 1  # TODO
MARKER_SIZE = 0.25
CAMERA_MATRIX = np.array([[640.5098521801531, 0.0, 640.5],
                              [0.0, 640.5098521801531, 360.5],
                              [0.0, 0.0, 1.0]])
CAMERA_DISTORTION = np.array([0, 0, 0, 0, 0]).astype(float)

gates = {
    1: None,
    2: None,
    3: None,
    4: None,
    5: None
}

target_topic = "/iris/target_pose"
start_topic = "/iris_control/challenge_start"
pose_sub_topic = "/mavros/local_position/pose"
gate_sub_topic = "/iris/next_gate"
camera_sub_topic = '/iris/usb_cam/image_raw'
# cmd_vel_topic = "/iris_control/cmd_vel"
cmd_vel_topic = "/mavros/setpoint_raw/local"
velocity_sub_topic = "/mavros/local_position/velocity_local"
detect_gate_topic = "/iris/detected_gate"
pos_sub_topic = "/mavros/local_position/pose"


target_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10)

next_gate = 1
# current_drone_pose = Pose()
current_drone_vel = Twist()
drone_position = PoseStamped()

start_pub = rospy.Publisher(start_topic, Bool, queue_size=10)
vel_pub = rospy.Publisher(cmd_vel_topic, PositionTarget, queue_size=10)
detect_gate_pub = rospy.Publisher(detect_gate_topic, UInt8, queue_size=10)

frame_cnt = 0
frame_cnt_flag = False
FRAME_CNT_THRESH = 100
aruco_area_flag = False
ARUCO_AREA_THRESH = 2500

QUEUE_LEN = 5
queue_col = deque(np.zeros(shape=(0),dtype=np.int16),maxlen=QUEUE_LEN)
queue_row = deque(np.zeros(shape=(0),dtype=np.int16),maxlen=QUEUE_LEN)

reg_err = np.zeros(shape=(10,))
challenge_started = False

def start_challenge_cb(msg: Bool):
    print("Sprawdzam")
    global challenge_started
    if msg.data:
        challenge_started = True

def camera_sub_callback(msg: Image):
    global frame_cnt, next_gate 
    global frame_cnt_flag, aruco_area_flag, was_next_gate_in
    global reg_err, drone_position 
    frame = BRIDGE.imgmsg_to_cv2(msg,"passthrough")
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, reject = cv2.aruco.detectMarkers(
        gray_frame, MARKER_DICT, 
        parameters = PARAM_MARKERS, 
        cameraMatrix=CAMERA_MATRIX,
        distCoeff=CAMERA_DISTORTION
    )

    print(f"Current gate = {next_gate} | Found gates = {ids}")
    
    if len(corners) > 0: 
        was_next_gate_in = False
        for corner, id in zip(corners, ids):
            id = int(id)
            corner = [corner]
            ret= cv2.aruco.estimatePoseSingleMarkers(corner, .25, CAMERA_MATRIX, CAMERA_DISTORTION)  
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
            aruco_center = np.zeros(shape=(3,1))
            aruco_center[0] -= 1 
            aruco_center[1] += 1
            
            goal_center, _____ = cv2.projectPoints(aruco_center, rvec, tvec, CAMERA_MATRIX,CAMERA_DISTORTION)
            goal_center = (int(goal_center[0][0][0]), int(goal_center[0][0][1]))

            if id == next_gate:
                was_next_gate_in = True

                queue_col.append(goal_center[0])
                queue_row.append(goal_center[1])
                col_mean = np.mean(queue_col,dtype="int16")
                row_mean = np.mean(queue_row,dtype="int16")

                gates[id] = (col_mean, row_mean)
                aruco_area_flag = calculate_area(corner[0][0]) > ARUCO_AREA_THRESH

                cv2.circle(gray_frame, (col_mean, row_mean), 5, (0,255,0),-1)
                cv2.aruco.drawDetectedMarkers(gray_frame, corner)
                cv2.aruco.drawAxis(gray_frame, CAMERA_MATRIX, CAMERA_DISTORTION, rvec, tvec, 0.25)
                cv2.putText(gray_frame, f"{id}", (10, 120), FONT_OF_TEXT, 4, (0, 255, 0), 2, cv2.LINE_AA)

            else:
                cv2.circle(gray_frame, goal_center, 5, (0,255,0),-1)
                gates[id] = goal_center

    if frame_cnt_flag:
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
        
    # if challenge_started:
    #     # def regulator_PD(Kp,Kd,val,val_prev=0,setpoint=620,saturation=0.5):
    #     vel_msg = PositionTarget()
    #     vel_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

    #     z_val, y_val = gates[next_gate]
    #     Kp_z = 0.001
    #     Kd_z = 0
    #     Kp_y = 0.01
    #     Kd_y = 0
    #     upper_bits_set_on = 0b1111000000000000
    #     vel_msg.type_mask = not (PositionTarget.IGNORE_VX \
    #     | PositionTarget.IGNORE_VY \
    #     | PositionTarget.IGNORE_VZ \
    #     | PositionTarget.IGNORE_YAW_RATE | upper_bits_set_on)
    #     vel_msg.velocity.x = 2
    #     # vel_msg.velocity.y = vel_msg.velocity.y 
    #     # vel_msg.velocity.z = vel_msg.velocity.z
    #     vel_msg.position.z = 2.5
    #     # vel_msg.linear.y = 0
    #     # vel_msg.linear.z = 0

    #     # vel_msg.linear.y, reg_err[0] = regulator_PD(Kp_z, Kd_z,
    #     #                                   val = z_val, e_prev = reg_err[0],
    #     #                                   setpoint = 640, saturation = 10 )
    #     # vel_msg.linear.z, reg_err[1] = regulator_PD(Kp_y, Kd_y,
    #     #                                   val = y_val, e_prev = reg_err[1],
    #     #                                   setpoint = 360, saturation = 10 )
        
    #     # linear_vel_vector = 

    #     vel_msg.yaw_rate, reg_err[2] = regulator_PD(Kp_z, Kd_z,
    #                                     val = z_val, e_prev = reg_err[2],
    #                                     setpoint = 640, saturation = 10)
        
        
    #     # vel_msg.linear.y = (reg_err[2])/100 if  (reg_err[2])/100 <= 0.6 else 0.6
    #     # vel_msg.linear.z, reg_err[3] = regulator_PD(Kp_y, 0, y_val, e_prev = reg_err[3],
    #     #                                 setpoint = 360, saturation = 6)
    #     # vel_msg.angular.x = -10*vel_msg.angular.z
    #     # vel_msg.angular.x = -10*vel_msg.angular.z
    #     vel_pub.publish(vel_msg)
    #     #print(f"{gates}")
    #     print(f"vel.x == {vel_msg.velocity.x} \n")
    #     # print(f"vel.lin z == {vel_msg.linear.z} \nvel.lin.y == {vel_msg.linear.y}")
    #     # print(f"error y == {reg_err[2]} \nerror z == {reg_err[3]}")


    cv2.circle(gray_frame, (640,360) , 5, (0,255,0),1)
    cv2.imshow("test", gray_frame)
    cv2.waitKey(1)


def gate_sub_callback(msg: UInt8):
    global next_gate
    next_gate = msg.data


def velocity_callback(msg: Twist):
    global current_drone_vel
    global gates, challenge_started
    if challenge_started:
        current_drone_vel = msg

# def pose_callback(msg: PoseStamped):
#     global drone_position
#     drone_position = msg


def camera_sub():
    rospy.init_node("target_position", anonymous=True)
    rospy.Subscriber(start_topic, Bool, start_challenge_cb)
    rospy.Subscriber(camera_sub_topic, Image, camera_sub_callback)
    rospy.Subscriber(gate_sub_topic, UInt8, gate_sub_callback)
    rospy.Subscriber(cmd_vel_topic, Twist, velocity_callback)
    #rospy.Subscriber(pos_sub_topic, PoseStamped, pose_callback)

    rospy.spin()


if __name__ == '__main__':
    camera_sub()
    # drone_control()
    cv2.destroyAllWindows()
    
