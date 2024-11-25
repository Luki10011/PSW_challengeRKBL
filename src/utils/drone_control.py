#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import math

# ROS topics
pos_pub_topic = "/mavros/setpoint_position/local"
pos_sub_topic = "/mavros/local_position/pose"
pub = rospy.Publisher(pos_pub_topic, PoseStamped, queue_size=10)

challenge_started = False
coordinates_acquired = False
target_pose = (0, 0, 0)  # Initialize target_pose

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
    global challenge_started, target_pose, coordinates_acquired
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    param_markers = cv2.aruco.DetectorParameters_create()
    matrix_coeff = np.array([[640.5098521801531, 0.0, 640.5],
                              [0.0, 640.5098521801531, 360.5],
                              [0.0, 0.0, 1.0]])
    dist_matrix = np.zeros((5,), dtype=float)

    corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

    if ids is not None and 2 in ids and not coordinates_acquired:
        ids = ids.flatten()
        marker_index = np.where(ids == 2)[0][0]
        marker_corner = corners[marker_index]
        
        ret = cv2.aruco.estimatePoseSingleMarkers(marker_corner, 0.25, matrix_coeff, dist_matrix)
        rvec, tvec = ret[0][0, 0, :], ret[1][0][0, :]

        # Update target_pose with tvec
        target_pose = (tvec[2], tvec[1], 2)#(tvec[2], tvec[1], tvec[0])  # Assuming z, y, x order
        
        # Draw detected markers and axes
        cv2.aruco.drawDetectedMarkers(gray_frame, corners)
        cv2.aruco.drawAxis(gray_frame, matrix_coeff, dist_matrix, rvec, tvec, 0.25)
        coordinates_acquired = True
        # Publish the pose
        publish_pose(tvec)

    cv2.imshow("test", gray_frame)
    cv2.waitKey(1)

def publish_pose(tvec):
    pub_msg = PoseStamped()
    pub_msg.pose.position.x = tvec[2] + 1
    pub_msg.pose.position.y = tvec[1]
    pub_msg.pose.position.z = 3#tvec[0]
    pub.publish(pub_msg)
    print(f"Published Position: x={tvec[2]}, y={tvec[1]}, z={tvec[0]}")

def start_challenge_cb(msg: Bool):
    global challenge_started
    if msg.data:
        challenge_started = True

def pose_cb(msg: PoseStamped):
    global target_pose
    pub_msg = PoseStamped()
    
    pub_msg.pose.position.x = target_pose[0]
    pub_msg.pose.position.y = target_pose[1]
    pub_msg.pose.position.z = 2

    pub.publish(pub_msg)
    print(f"Target Position set to: x={target_pose[0]}, y={target_pose[1]}, z={target_pose[2]}")

def drone_control():
    rospy.init_node('drone_control_node', anonymous=True)
    rospy.Subscriber("/iris_control/challenge_start", Bool, start_challenge_cb)
    if not coordinates_acquired:
        rospy.Subscriber('/iris/usb_cam/image_raw', Image, camera_sub_callback)
    rospy.Subscriber(pos_sub_topic, PoseStamped, pose_cb)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        drone_control()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
