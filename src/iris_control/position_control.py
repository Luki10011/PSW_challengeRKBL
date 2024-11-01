#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from aruco_pose_estimation import estimate_pose
from enum import Enum
import numpy as np

GATE_HEIGHT = 1  # TODO
GATE_WIDTH = 1  # TODO
MARKER_SIZE = 0.1  # TODO
CAMERA_MATRIX = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]  # TODO
CAMERA_DISTORTION = [0, 0, 0, 0, 0]  # TODO
NO_GATES = 5  # TODO: Sprawdzić

pos_pub_topic = "/mavros/setpoint_position/local"
pos_sub_topic = "/mavros/local_position/pose"
pub = rospy.Publisher(pos_pub_topic, PoseStamped, queue_size=10)

class Stage(Enum):
    STARTING = 1
    FLIGHT = 2
    IN_TARGET = 3
    FINISH = 4

challenge_started = False
challenge_stage  = Stage.STARTING
current_marker_pose = PoseStamped()
current_target = PoseStamped()
next_gate = 1


def pose_cb(msg: PoseStamped):
    global challenge_stage, next_gate, challenge_started

    drone_pose = msg.pose.position

    match challenge_stage:

        case Stage.STARTING:
            if challenge_started:
                challenge_stage = Stage.FLIGHT

        case Stage.FLIGHT:
            # Wykrywanie znacznika

            # Jeśli wykryto oczekiwany znacznik, to liczymy jego pozycję i target

            # Dron przeleciał przez bramkę
            if np.round(drone_pose.x, 1) == np.round(current_target.x, 1) and\
               np.round(drone_pose.y, 1) == np.round(current_target.y, 1) and\
               np.round(drone_pose.z, 1) == np.round(current_target.z, 1):
                challenge_stage = Stage.IN_TARGET

        case Stage.IN_TARGET:
            
            if next_gate <= NO_GATES:
                next_gate += 1
                challenge_stage = Stage.FLIGHT
            else:
                challenge_stage = Stage.FINISH




        case Stage.FINISH:
            # Koniec trasy - lądowanie
            current_target.pose.position.z = 0
            challenge_started = False
        

    pub.publish(current_target)


def start_challenge_cb(msg: Bool):
    global challenge_started
    if msg.data:
        challenge_started = True


def drone_control():
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("/iris_control/challenge_start", Bool, start_challenge_cb)

    if challenge_started:
        rospy.Subscriber(pos_sub_topic, PoseStamped, pose_cb)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        drone_control()
    except rospy.ROSInterruptException:
        pass