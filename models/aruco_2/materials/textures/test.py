import numpy as  np
import cv2
import matplotlib.pyplot as plt



if __name__ == "__main__":
    image_file = "/root/sim_ws/src/psw_challenge/models/aruco_5/materials/textures/aruco_5.jpeg"
    frame = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
    print(type(frame))
    frame = np.pad(frame, pad_width=100, constant_values = 255)
    print(type(frame))
    cv2.imshow("Okno", frame)
    cv2.waitKey(0)
    cv2.imwrite("/root/sim_ws/src/psw_challenge/models/aruco_5/materials/textures/aruco_5_test.jpeg", frame)