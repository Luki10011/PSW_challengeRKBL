import numpy as np
import cv2
import itertools




def decodeImage(IMG):
    marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    param_markers  = cv2.aruco.DetectorParameters_create()
    gray_frame = cv2.cvtColor(IMG, cv2.COLOR_BGR2GRAY)
    # param_markers.minMarkerPerimeterRate = 0.03  # Minimalny stosunek obwodu markera
    # param_markers.maxMarkerPerimeterRate = 6.0  # Maksymalny stosunek obwodu markera
    # param_markers.polygonalApproxAccuracyRate = 0.05  # Precyzja aproksymacji
    marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
        gray_frame, marker_dict, parameters = param_markers
    )
    # print(marker_corners)
    print(marker_IDs, reject)
    # corner1, corner2, corner3, corner4 = [marker[i][0] for i, marker in enumerate(marker_corners)]
    size = gray_frame.shape
    if marker_IDs is not None and len(marker_corners) > 0:
        # Assuming we are interested in the first marker
        corners = marker_corners[0][0]  # Get the corners of the first marker
        i_cor = list(set(corners[:, 1]))
        j_cor = list(set(corners[:, 0]))

        for i in range(size[0]):
            for j in range(size[1]):
                if i == i_cor[0] or i == i_cor[1] or j == j_cor[0] or j == j_cor[1]:
                    gray_frame[i, j] = 100
        
    cv2.imshow("test", gray_frame)
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image = cv2.imread("/root/sim_ws/src/psw_challenge/src/iris_control/test4x4_gazebo.jpeg")

    decodeImage(image)

    # closing all open windows
    
    
