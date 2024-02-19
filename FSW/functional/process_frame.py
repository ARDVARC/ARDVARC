"""
ARDVARC Flight Software

Author: Timothy Behrer
email: tibe0349@colorado.edu

Description:
    File for the camera vision and RGV inertial position estimate

Notes:
    TODO Implement camera vision object recognition software
    TODO Implement camera (in/ex)trinsics
    TODO Implement camera variance
    TODO Implement frame transition and UAS state to backout the RGV inertial position

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
    direction_vectors: List[UasToRgvDirectionVectorUasFrame]


## Function to detect ArUco markers
def detect_ArUco_Direction_and_Pose(frame: cv2.typing.MatLike) -> Optional[DetectionInfo]: 
    aruco_type_list = [] 
    for aruco_type, dictionary_id in constants.ARUCO_DICT.items():

        parameters =  cv2.aruco.DetectorParameters()
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)  
        
        ## Get the ArUco directory
        corners, ids, _ = detector.detectMarkers(frame)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            direction_vectors = []
            aruco_type_list.append(aruco_type)
            """ Not Sure I wanna keep this yet, depending on how I want to store the ids
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
                """topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))"""
                ###### Unnecessary Code ######

                ###### OLD OLD Line Marking Code ######
                # draw the bounding box of the ArUCo detection in RED
                """cv2.line(frame, topLeft, topRight, (0, 0, 255), 2)
                cv2.line(frame, topRight, bottomRight, (0, 0, 255), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 0, 255), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 0, 255), 2)"""
                ###### OLD OLD Line Marking Code ######


                # compute and draw the center (x, y)-coordinates of the ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2)
                cY = int((topLeft[1] + bottomRight[1]) / 2)

                ###### Old Line Marking Code ######
                #cv2.polylines(frame, [markerCorner.astype(int)], True, (0, 0, 255), 2)
                #cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                ###### Old Line Marking Code ######

                ## TODO Do math to get direction_vectors (TBD)
                direction_vectors.append(
                    UasToRgvDirectionVectorUasFrame(
                        # TODO: Make this something reasonable
                    )
                )

                ## TODO Might want to put this in the for loop for FrameAxes
                cv2.aruco.drawDetectedMarkers(frame, corners) #Draw the detected markers

                

            #Compute the pose of the aruco
            for i in range(0, len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.025, constants.INTRINSICS_PI_CAMERA, constants.DISTORTION)

                cv2.drawFrameAxes(frame, constants.INTRINSICS_PI_CAMERA, constants.DISTORTION, rvec, tvec, 0.01) 
            
            

                
        return DetectionInfo(frame, ids, direction_vectors)


## TODO Get rid of this
class Camera:
    """Class for camera vison object recognition software"""

    def __init__(self, camera_id):
        """Initialize the camera"""
        self.camera = cv2.VideoCapture(camera_id)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, constants.FRAME_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,constants.FRAME_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS, constants.VIDEO_FPS)
        self.camera.set(cv2.CAP_PROP_OPENNI_FOCAL_LENGTH, constants.FOCAL_LENGTH)
        if not self.camera.isOpened():
            print("Error opening video file")
            sys.exit()

"""Unit Test Code
if __name__ == "__main__":

    image_path = r"arucoMarkers/singlemarkersoriginal.jpg"

    intrinsic_camera = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
    distortion = np.array((-0.43948, 0.18514, 0, 0))

    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, image = cap.read()

        for aruco_type in detect_markers(image):
             image = pose_estimation(image, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)
             
        cv2.imshow('Estimated Pose', image)
                
        if cv2.waitKey(50) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()"""