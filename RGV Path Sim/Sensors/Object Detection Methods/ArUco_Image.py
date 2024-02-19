# import the necessary packages
import argparse
import cv2
import sys
from cv2 import aruco
import numpy as np
import time
import os 
start_time = time.time()

## Initialize arUco marker detection
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
  
# Create a VideoCapture object and read from input file 
"""cap = cv2.imread("aprilTag16H5_30.png")"""
cap = cv2.VideoCapture("DJI_0011_AR_2_30_S_-_Trim.mp4")
frame_width = int(cap.get(3)) 
frame_height = int(cap.get(4)) 
print(frame_width, frame_height)
size = (frame_width, frame_height) 
# result = cv2.VideoWriter('DJI_0011_AR_2_30_S_-_Trim_Annotated.avi', cv2.VideoWriter_fourcc(*'MJPG'),10, size) #XVID/MJPG
# Check if camera opened successfully 
if (cap.isOpened()== False): 
    print("Error opening video file") 

while(cap.isOpened()): 

# Capture frame-by-frame 
    ret, frame = cap.read() 
    if ret == True:  
        ## Get the ArUco directory
        (corners, ids, rejected) = detector.detectMarkers(frame)
        
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box of the ArUCo detection in RED
                cv2.line(frame, topLeft, topRight, (0, 0, 255), 2)
                cv2.line(frame, topRight, bottomRight, (0, 0, 255), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 0, 255), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 0, 255), 2)
                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image in Green
                cv2.putText(frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                
                # Label the ArUco marker
                print("[INFO] ArUco marker ID: {}".format(markerID))
            
        # result.write(frame) 
        end_time = time.time()
        elapsed_time = end_time - start_time
        print("Time: ", elapsed_time)
        # # Display the resulting frame 
        # cv2.imshow('Frame', cap)
        # # Press Q on keyboard to exit 
        # if cv2.waitKey(0) & 0xFF == ord('q'): 
        #     break

    # Break the loop 
    else:  
        break
# When everything done, release 
# the video capture object 
cap.release() 
# result.release() 

end_time = time.time()
elapsed_time = end_time - start_time
print("Time: ", elapsed_time)