"""
ARDVARC Flight Software

Author: Timothy Behrer
email: tibe0349@colorado.edu

Description:
    File for the camera vision and RGV inertial position estimate

Notes:
    TODO Implement camera vision object recognition software (TB 2021-09-20: Preliminarily complete)
    TODO Implement camera (in/ex)trinsics (TB 2021-09-20: constants.INTRINSICS_PI_CAMERA)
    TODO Implement camera variance (TB 2021-09-20: constants.DISTORTION)
    TODO Implement frame transition and UAS state to backout the RGV inertial position(TB 2021-09-20: MVP Complete)
    TODO Clean out old code after a review and unit testing
    TODO Get rid of possible appends
    """
# Python 3.7
## Imports from the existing ArUco marker detection and annotation function
import sys
import cv2
from cv2 import aruco
import numpy as np
import os
import argparse
from dataclasses import dataclass
from ..config import constants
import rospy
from rosardvarc.msg import UasToRgvDirectionVectorUasFrame
from typing import Optional, List
## TODO Make sure all imports are correct


@dataclass
class DetectionInfo():
    annotated_camera_frame: cv2.typing.MatLike
    ids: cv2.typing.MatLike
    direction_vector: List[UasToRgvDirectionVectorUasFrame]


## Function to detect ArUco markers
def detect_ArUco_Direction_and_Pose(frame: cv2.typing.MatLike) -> Optional[DetectionInfo]: 
    aruco_type_list = [] 
    direction_vector = []

    for aruco_type, dictionary_id in constants.ARUCO_DICT.items():

        parameters =  cv2.aruco.DetectorParameters()
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)  
        
        ## Get the ArUco directory
        (corners, ids, _) = detector.detectMarkers(frame)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            aruco_type_list.append(aruco_type)
            """ 
            Not Sure I wanna keep this yet, depending on how I want to store the ids
            # flatten the ArUco IDs list
            ids = ids.flatten()
            """
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids.flatten()):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners_reshaped = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_reshaped
                # convert each of the (x, y)-coordinate pairs to integers

                ###### Unnecessary Code ######
                """
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                """
                ###### Unnecessary Code ######

                ###### OLD OLD Line Marking Code ######
                # draw the bounding box of the ArUCo detection in RED
                """
                cv2.line(frame, topLeft, topRight, (0, 0, 255), 2)
                cv2.line(frame, topRight, bottomRight, (0, 0, 255), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 0, 255), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 0, 255), 2)
                """
                ###### OLD OLD Line Marking Code ######

                ###### Old Line Marking Code ######
                """
                #cv2.polylines(frame, [markerCorner.astype(int)], True, (0, 0, 255), 2)
                #cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                """
                ###### Old Line Marking Code ######


                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2)
                cY = int((topLeft[1] + bottomRight[1]) / 2)

                ## TODO Might want to put this in the for loop for FrameAxes
                cv2.aruco.drawDetectedMarkers(frame, corners) #Draw the detected markers

                

            #Compute the pose of the aruco: rvec = Rotation Vector, tvec = Translation Vector
            for i in range(0, len(ids)):

                (rvec, tvec, _) = my_estimatePoseSingleMarkers(corners[i], 0.025, constants.INTRINSICS_PI_CAMERA, constants.DISTORTION)                
                rvec = np.array(rvec)
                tvec = np.array(tvec)
                cv2.drawFrameAxes(frame, constants.INTRINSICS_PI_CAMERA, constants.DISTORTION, rvec, tvec, 0.01) 
        
                ## TODO Do math to get direction_vector (TB 2021-09-20: Added a MVP implementation of the possible DCM and Translation Vector)
                ## TODO Configure the direction_vector to be in the UAS Frame (TB 2021-09-20: Roughly Executed)
                tvec_UASFrame = camera_frame_to_UAS_frame(tvec)
                direction_vector.append(tvec_UASFrame)
                
        return DetectionInfo(frame, ids, direction_vector)


def camera_frame_to_UAS_frame(position: np.typing.NDArray) -> np.typing.NDArray:
    #Position: 3x1 vector of the vector given in the camera frame transformed to the UAS frame
    ## TODO Implement this function(TB 2021-09-20: MVP Complete)
    ## TODO Make sure the matrix mult is right

    #Convert from rotation Vector to Rotation Matrix
    position_UASFrame = constants.EXTRINSICS_PI_CAMERA_DCM * (position) + constants.EXTRINSICS_PI_CAMERA_TVEC

    return position_UASFrame

## TODO (TB) Test this function as It was done by someone else:

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    """https://stackoverflow.com/questions/75750177/solve-pnp-or-estimate-pose-single-markers-which-is-better"""

    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''

    ## TODO (TB) Clean up this and preallocate function

    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []

    # rvecs = np.zeros(len(corners), 3)
    
    for ii in corners:
        _, R, t = cv2.solvePnP(marker_points, ii, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
       # trash.append(nada)
    return rvecs, tvecs, trash


#Unit Test Code
## TODO Unit test this code
if __name__ == "__main__":

    ## TODO Test and Implement various videos and images
    image_path = "FSW/fake_data_generators/DJI_0011_AR_2_30_S_-_Trim.mp4"

    cap = cv2.VideoCapture(image_path)

    while cap.isOpened():
        ret, image = cap.read()
        if not ret:
            break
        
        Detection_Info = detect_ArUco_Direction_and_Pose(image)
             
        image = Detection_Info.annotated_camera_frame
        ##Show the image with the estimated pose, USE IF ONLY FEW IMAGE
        cv2.imshow('Estimated Pose', image)

                
        if cv2.waitKey(50) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()



# Notes and relevant information:
    """
    OpenCV routines that deal with cameras and camera calibration (including AruCo) use a pinhole camera model. The world origin is defined as the centre of projection of the camera model (where all light rays entering the camera converge), the Z axis is defined as the optical axis of the camera model, and the X and Y axes form an orthogonal system with Z. +Z is in front of the camera, +X is to the right, and +Y is down. All AruCo coordinates are defined in this coordinate system. That explains why your "camera" tvecs and rvecs change: they do not define your camera's position in some world coordinate system, but rather the markers' positions relative to your camera.

You don't really need to know how the camera calibration algorithm works, other than that it will give you a camera matrix and some lens distortion parameters, which you use as input to other AruCo and OpenCV routines.

Once you have calibration data, you can use AruCo to identify markers and return their positions and orientations in the 3D coordinate system defined by your camera, with correct compensation for the distortion of your camera lens. This is adequate to do, for example, augmented reality using OpenGL on top of the video feed from your camera.

The tvec of a marker is the translation (x,y,z) of the marker from the origin; the distance unit is whatever unit you used to define your printed calibration chart (ie, if you described your calibration chart to OpenCV using mm, then the distance unit in your tvecs is mm).

The rvec of a marker is a 3D rotation vector which defines both an axis of rotation and the rotation angle about that axis, and gives the marker's orientation. It can be converted to a 3x3 rotation matrix using the Rodrigues function (cv::Rodrigues()). It is either the rotation which transforms the marker's local axes onto the world (camera) axes, or the inverse -- I can't remember, but you can easily check.
"""