#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8
from enum import Enum
import numpy as np

NO_GATES = 5  # TODO: Sprawdzić

start_topic = "/iris_control/challenge_start"

pos_pub_topic = "/mavros/setpoint_position/local"
gate_pub_topic = "/iris/next_gate"

pos_sub_topic = "/mavros/local_position/pose"
target_sub_topic = "/iris/target_pose"

start_pub = rospy.Publisher(start_topic, Bool, queue_size=10)
pos_pub = rospy.Publisher(pos_pub_topic, PoseStamped, queue_size=10)
gate_pub = rospy.Publisher(gate_pub_topic, UInt8, queue_size=10)

class Stage(Enum):
    STARTING = 1
    FLIGHT = 2
    IN_TARGET = 3
    FINISH = 4

challenge_started = False
challenge_stage  = Stage.STARTING
# current_marker_pose = PoseStamped()
current_target = PoseStamped()
next_gate = 1


def pose_cb(msg: PoseStamped):
    global challenge_stage, next_gate, challenge_started

    drone_pose = msg.pose.position

    match challenge_stage:

        case Stage.STARTING:
            if challenge_started:
                gate_msg = UInt8()
                gate_msg.data = np.uint8(next_gate)
                gate_pub.publish(gate_msg)
                challenge_stage = Stage.FLIGHT

        case Stage.FLIGHT:
            pos_pub.publish(current_target)

            # Dron przeleciał przez bramkę
            if np.round(drone_pose.x, 1) == np.round(current_target.x, 1) and\
               np.round(drone_pose.y, 1) == np.round(current_target.y, 1) and\
               np.round(drone_pose.z, 1) == np.round(current_target.z, 1):
                challenge_stage = Stage.IN_TARGET

        case Stage.IN_TARGET:
            
            if next_gate <= NO_GATES:
                next_gate += 1
                gate_msg = UInt8()
                gate_msg.data = np.uint8(next_gate)
                gate_pub.publish(gate_msg)
                challenge_stage = Stage.FLIGHT
            else:
                next_gate = 1
                gate_msg = UInt8()
                gate_msg.data = np.uint8(next_gate)
                gate_pub.publish(gate_msg)
                challenge_stage = Stage.FINISH

        case Stage.FINISH:
            # Koniec trasy - lądowanie
            current_target.pose.position.z = 0
            pos_pub.publish(current_target)

            # Koniec challengu
            challenge_started = False
            start_msg = Bool
            start_msg.data = challenge_started
            start_pub.publish(start_msg)

def start_challenge_cb(msg: Bool):
    global challenge_started
    if msg.data:
        challenge_started = True
        

def target_cb(msg: PoseStamped):
    global current_target
    current_target = msg


def drone_control():
    rospy.init_node('control', anonymous=True)
    rospy.Subscriber(start_topic, Bool, start_challenge_cb)

    if challenge_started:
        rospy.Subscriber(pos_sub_topic, PoseStamped, pose_cb)
        rospy.Subscriber(target_sub_topic, PoseStamped, target_cb)
    
    rospy.spin()


if __name__ == '__main__':
    try:
        drone_control()
    except rospy.ROSInterruptException:
        pass