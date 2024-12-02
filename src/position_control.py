#! /usr/bin/env python3
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
detect_gate_topic = "/iris/detected_gate"


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
current_target= PoseStamped()
next_gate = 1
detected_gate = 0

drone_pose = PoseStamped()


def pose_cb(msg: PoseStamped):
    global drone_pose
    drone_pose = msg.pose.position
    if challenge_stage == Stage.FLIGHT:
        pos_pub.publish(current_target)


def start_challenge_cb(msg: Bool):
    print("Sprawdzam")
    global challenge_started
    if msg.data:
        challenge_started = True
        

def target_cb(msg: PoseStamped):
    global challenge_stage, next_gate, challenge_started, detected_gate, current_target

    current_target = msg
    

    if challenge_stage == Stage.STARTING:
        if challenge_started:
            gate_msg = UInt8()
            gate_msg.data = np.uint8(next_gate)
            gate_pub.publish(gate_msg)
            challenge_stage = Stage.FLIGHT
            print("STARTED")

    elif challenge_stage == Stage.FLIGHT:
        # pos_pub.publish(current_target)

        # Dron przeleciał przez bramkę
        # if np.round(drone_pose.x, 1) == np.round(current_target.pose.position.x, 1) and\
        #     np.round(drone_pose.y, 1) == np.round(current_target.pose.position.y, 1) and\
        #     np.round(drone_pose.z, 1) == np.round(current_target.pose.position.z, 1):
        print(f"diff = {np.linalg.norm(np.array([drone_pose.x, drone_pose.y, drone_pose.z]) - np.array([current_target.pose.position.x, current_target.pose.position.y, current_target.pose.position.z]))}")
        if np.linalg.norm(np.array([drone_pose.x, drone_pose.y]) - np.array([current_target.pose.position.x, current_target.pose.position.y])) < 0.7:
            challenge_stage = Stage.IN_TARGET
            next_gate += 1
        
        print(f"FLIGHT: {current_target.pose.position.x}, {current_target.pose.position.y}, {current_target.pose.position.z}")

    elif challenge_stage == Stage.IN_TARGET:
        if next_gate <= NO_GATES:
            gate_msg = UInt8()
            gate_msg.data = np.uint8(next_gate)
            gate_pub.publish(gate_msg)
            print(f"det: {detected_gate}, next: {next_gate}")
            if detected_gate == next_gate:
                challenge_stage = Stage.FLIGHT
                print("NEXT")
        else:
            next_gate = 1
            gate_msg = UInt8()
            gate_msg.data = np.uint8(next_gate)
            gate_pub.publish(gate_msg)
            challenge_stage = Stage.FINISH
            print("FINISH")
            

    elif challenge_stage == Stage.FINISH:
        # Koniec trasy - lądowanie
        current_target.pose.position.z = 0
        pos_pub.publish(current_target)

        # Koniec challengu
        challenge_started = False
        start_msg = Bool
        start_msg.data = challenge_started
        start_pub.publish(start_msg)
        print("END")


def detect_gate_cb(msg: UInt8):
    global detected_gate
    detected_gate = msg.data


def drone_control():
    rospy.init_node('control', anonymous=True)
    rospy.Subscriber(start_topic, Bool, start_challenge_cb)
    rospy.Subscriber(pos_sub_topic, PoseStamped, pose_cb)
    rospy.Subscriber(target_sub_topic, PoseStamped, target_cb)
    rospy.Subscriber(detect_gate_topic, UInt8, detect_gate_cb)
    
    rospy.spin()


if __name__ == '__main__':
    print("CHUJ")
    drone_control()
    # try:
    #     drone_control()
    # except rospy.ROSInterruptException:
    #     pass