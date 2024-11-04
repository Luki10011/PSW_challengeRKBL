import cv2
import numpy as np


# Dla opencv 4.10

# def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
#     '''
#     This will estimate the rvec and tvec for each of the marker corners detected by:
#        corners, ids, rejectedImgPoints = detector.detectMarkers(image)
#     corners - is an array of detected corners for each detected marker in the image
#     marker_size - is the size of the detected markers
#     mtx - is the camera matrix
#     distortion - is the camera distortion matrix
#     RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
#     '''
#     marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
#                               [marker_size / 2, marker_size / 2, 0],
#                               [marker_size / 2, -marker_size / 2, 0],
#                               [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
#     trash = []
#     rvecs = []
#     tvecs = []
#     for c in corners:
#         nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
#         rvecs.append(R)
#         tvecs.append(t)
#         trash.append(nada)
#     return rvecs, tvecs, trash


# def estimate_spec_marker_pose(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, marker_size, expected_marker_id):

#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
#     parameters = cv2.aruco.DetectorParameters()

#     detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
#     corners, ids, rejected_img_points = detector.detectMarkers(gray)

#     ret_rvec = None
#     ret_tvec = None

#     if len(corners) > 0:
#         for i in range(0, len(ids)):

#             rvec, tvec, markerPoints = estimatePoseSingleMarkers(corners[i], marker_size, matrix_coefficients,
#                                                                        distortion_coefficients)
            
#             # TODO: Tu jeszcze można dodać jakąś wizualizację dla każdego z wykrytych punktów

#             if ids[i] == expected_marker_id:
#                 ret_rvec = rvec
#                 ret_tvec = tvec
    

            
#     return ret_rvec, ret_tvec, frame


# Dla opencv 4.2


def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, marker_size: float, expected_marker_id: int):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)
    
    ret_rvec = None
    ret_tvec = None

    if len(corners) > 0:
        for i in range(len(ids)):
           
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, matrix_coefficients,
                                                                       distortion_coefficients)
            
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

            # TODO: Tu jeszcze można dodać jakąś dodatkową wizualizację dla każdego z wykrytych punktów

            if int(ids[i]) == expected_marker_id:
                ret_rvec = rvec
                ret_tvec = tvec

    return ret_rvec, ret_tvec, frame

