# Commands to run:
# pip3 install robotpy
# pip3 install robotpy-cscore (for import cscore)
# pip3 install opencv-python (for import cv2)
# pip3 install apriltag
# pip3 install pynetworktables (for import networktables)
# pip3 install robotpy_apriltag (for import robotpy_apriltag)
# pip3 install robotpy-photonvision
# pip3 uninstall [package] to uninstall it

import cv2

from networktables import NetworkTables
import robotpy_apriltag

import numpy

cap = cv2.VideoCapture(0)
detector = robotpy_apriltag.AprilTagDetector()
detector.addFamily("tag16h5")

i = 0

NetworkTables.initialize(server="10.9.91.2") # IP Address of RoboRio, I think

table = NetworkTables.getTable("SmartDashboard")

while (True):
    i += 1
    success, frame = cap.read()
    
    if (success):

        # print("Frame rate: ", int(cap.get(cv2.CAP_PROP_FPS)), "FPS")
        
        cv2.imshow('frame', frame) # puts it in a window
        
        if (i % 10) == 0: # roughly once a second
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert to grayscale
            
            gray = cv2.inRange(gray, numpy.mat([200]), numpy.mat([255]))

            results = detector.detect(gray)
            
            data = []
            
            for result in results:
                id = result.getId()
                margin = int(result.getDecisionMargin() * 10 + 0.5) / 10.0
                x = int(result.getCenter().x * 10 + 0.5) / 10.0
                y = int(result.getCenter().y * 10 + 0.5) / 10.0
                print("Id:\t" + str(id))
                print("Decision Margin:\t" + str(margin))
                print("Center:\t(" + str(x) + ", " + str(y) + ")") # center point
                
                if (margin > 160):
                    data.append(id) # cannot be a list of lists :(
                                # however can still do modular lists (ex. each list of 4 elements is actually one element)
                
            table.putNumberArray(
                "April", 
                data
            )
            
            print(table.getNumberArray("April", "None"))
    else:
        print("error")
    
    if cv2.waitKey(100) == 13:
        break

# loop over; clean up and dump the last updated frame for convenience of debugging

cv2.destroyAllWindows()