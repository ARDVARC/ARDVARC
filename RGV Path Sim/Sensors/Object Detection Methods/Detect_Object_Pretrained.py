import cv2
import math
from ultralytics import YOLO #Load in YOLO


image = "huge.jpg"
#Load in a pre-trained model
model = YOLO("yolov8n.pt")

#Run object detection on pre-trained database
results = model.predict(image)

result = results[0]
print(len(result.boxes))
img = cv2.imread(image)

for box in result.boxes:
    label = box.cls[0].item()
    cords = [round(x) for x in box.xyxy[0].tolist()]
    prob = box.conf[0].item()
    print("Object type: ",label)
    print("Coordinates: ",cords)
    print("Probability: ",prob)
    img = cv2.rectangle(img,(cords[0],cords[1]),(cords[2],cords[3]),(255, 0, 0)) #Plot the rectangle
    center_point = (int((math.floor(cords[0]+cords[2])/2)), int(math.floor((cords[1]+cords[3])/2))) #Find the centroid
    print("Centroid Pixel: ", center_point)
    img = cv2.circle(img,center_point,0,(0, 0, 255),thickness= -1) #Plot the centroid

    

cv2.imshow("YOLO Model",img)
cv2.waitKey(0)

