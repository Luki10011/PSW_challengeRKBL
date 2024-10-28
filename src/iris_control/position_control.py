#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

pos_pub_topic = "/mavros/setpoint_position/local"
pos_sub_topic = "/mavros/local_position/pose"
pub = rospy.Publisher(pos_pub_topic, PoseStamped, queue_size=10)

challenge_started = False


def pose_cb(msg: PoseStamped):
    target_pose = (10, 10, 2)
    if msg.pose.position.z >= 2:
        pub_msg = PoseStamped()

        pub_msg.pose.position.x = target_pose[0]
        pub_msg.pose.position.y = target_pose[1]
        pub_msg.pose.position.z = target_pose[2]

        pub.publish(pub_msg)


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