import numpy as np
import cv2

img = cv2.imread("huge.jpg",cv2.IMREAD_GRAYSCALE) #Read in the image in greyscale
img = cv2.GaussianBlur(img,[3,3],.1)
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()


# Change thresholds
params.minThreshold = 50
params.maxThreshold = 70 

# Filter by distance between blobs
#params.filterByDistance = False
params.minDistBetweenBlobs = 1

# Filter by Area.
params.filterByArea = True
params.minArea = 150
params.maxArea = 1000

# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.1

# Filter by Convexity
params.filterByConvexity = False
params.minConvexity = 0.87

# Filter by Inertia
params.filterByInertia = False
params.minInertiaRatio = 0.01

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params) #Define the detector
keypoints = detector.detect(img) #Find the keypoints for the simple blob detector


#Create the image with the keypoints
imgKey = cv2.drawKeypoints(img,keypoints,np.array([]),(0,0,225),cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
cv2.imshow("Keypoints",imgKey)
cv2.waitKey(0)
cv2.destroyWindow




