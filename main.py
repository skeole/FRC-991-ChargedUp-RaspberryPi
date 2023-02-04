# Commands to run:
# pip3 install robotpy
# pip3 install robotpy-cscore (for import cscore)
# pip3 install opencv-python (for import cv2)
# pip3 install apriltag
# pip3 install pynetworktables (for import networktables)
# pip3 install robotpy_apriltag (for import robotpy_apriltag)
# pip3 install robotpy-photonvision
# pip3 uninstall [package] to uninstall it
# you need to install logging in order to read messages from the SmartDashbaord

import cv2

from networktables import NetworkTables
from robotpy_apriltag import AprilTagDetector, AprilTagPoseEstimator

# import logging
# logging.basicConfig(level=logging.DEBUG)

import numpy
import math

cap = cv2.VideoCapture(0)
detector = AprilTagDetector()
detector.addFamily("tag16h5")
estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(
        0.05, # tag size, in meters
        1008, # focal length in pixels: length in pixels * distance / actual length
         # around 7mm for 5mm object to be 720 pixels --> 
        1008, # has to be experimentally determined, but fx and fy are generally the same
        640, # literally just the resolution divided by 2 :D
        360 # I think
    ))

i = 0

limit = 200
minconfidence = 150

draw = False

NetworkTables.initialize(server="10.9.91.2") # IP Address of RoboRio, I think

table = NetworkTables.getTable("SmartDashboard")

while (True):
    i += 1
    success, frame = cap.read()
    
    if (success):

        # print("Frame rate: ", int(cap.get(cv2.CAP_PROP_FPS)), "FPS")
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert to grayscale
        
        gray = cv2.inRange(gray, numpy.mat([limit]), numpy.mat([255]))

        results = detector.detect(gray)
        
        data = []
        
        for result in results:
            if (result.getCorner(0).x - result.getCorner(2).x) * (result.getCorner(0).x - result.getCorner(2).x) + (result.getCorner(0).y - result.getCorner(2).y) * (result.getCorner(0).y - result.getCorner(2).y) < 10:
                continue # if the diagonal distance is too small
            
            if min(result.getCorner(0).x, result.getCorner(1).x, result.getCorner(2).x, result.getCorner(3).x, result.getCorner(0).y, result.getCorner(1).y, result.getCorner(2).y, result.getCorner(3).y) < 50:
                continue
            
            if min(1280 - result.getCorner(0).x, 1280 - result.getCorner(1).x, 1280 - result.getCorner(2).x, 1280 - result.getCorner(3).x, 720 - result.getCorner(0).y, 720 - result.getCorner(1).y, 720 - result.getCorner(2).y, 720 - result.getCorner(3).y) < 15:
                continue # if we're too close to the edge of the screen
            
            id = result.getId()
            margin = int(result.getDecisionMargin() * 10 + 0.5) / 10.0
            
            if (margin < minconfidence): continue # don't do anything else in the loop
            
            
            x = int(result.getCenter().x * 10 + 0.5) / 10.0
            y = int(result.getCenter().y * 10 + 0.5) / 10.0
            
            # center of april tag
            
            if draw:
                
                points = []
                for i in range(4):
                    points.append([
                        result.getCorner(i).x, result.getCorner(i).y
                    ])
                
                for j in range(4):
                    cv2.line(
                        frame, 
                        (int(points[j][0] + 0.5), int(points[j][1] + 0.5)), 
                        (int(points[(j + 1) % 4][0] + 0.5), int(points[(j + 1) % 4][1] + 0.5)), 
                        (0, 255, 0), # BGR Color
                        10
                    )
                
                cv2.line(
                    frame, 
                    (int(points[0][0] + 0.5), int(points[0][1] + 0.5)), 
                    (int(points[2][0] + 0.5), int(points[2][1] + 0.5)), 
                    (0, 0, 255), # BGR Color
                    10
                )
                
                
                cv2.line(
                    frame, 
                    (int(points[1][0] + 0.5), int(points[1][1] + 0.5)), 
                    (int(points[3][0] + 0.5), int(points[3][1] + 0.5)), 
                    (0, 0, 255), # BGR Color
                    10
                )
            
            poses = estimator.estimateOrthogonalIteration(result, 10)
            
            # figure out how the x/y/z translate to a length and an angle
            # length: 1 / z
            # angle: 
            # obviously, 0/0 means no angle
            # y is -90 --> due right
            # x is +90 --> due up
            # is it literally just tangent????
            
            angle1 = 0 - math.atan(poses.pose1.rotation().x_degrees / poses.pose1.rotation().y_degrees)
            if (poses.pose1.rotation().y_degrees > 0): 
                angle1 += math.pi
            
            multiplier = math.sqrt(poses.pose1.rotation().x_degrees * poses.pose1.rotation().x_degrees + poses.pose1.rotation().y_degrees * poses.pose1.rotation().y_degrees) / 127.2792206136
            length = 20 / poses.pose1.translation().z
            
            if draw:
                cv2.line(
                    frame, 
                    (int(x + 0.5), int(y + 0.5)), 
                    (int(x + length * math.cos(angle1) * multiplier + 0.5), int(y + length * math.sin(angle1) * multiplier + 0.5)), 
                    (255, 0, 0), # BGR Color
                    10
                )
            
            data.append(id) # cannot be a list of lists :(
                            # however can still do modular lists (ex. each list of 4 elements is actually one element)
            
        table.putNumberArray(
            "April", 
            data
        )
        
        if (draw):
            print(data)
        
        # flippedframe = cv2.flip(frame, 1)
        
        # flippedgray = cv2.cvtColor(flippedframe, cv2.COLOR_BGR2GRAY) # convert to grayscale
        
        # flippedgray = cv2.inRange(flippedgray, numpy.mat([limit]), numpy.mat([255]))

        
        cv2.imshow('frame', frame) # puts it in a window  
        
        # cv2.imshow('gray', flippedgray) # puts it in a window  
        
        # print(cv2.getWindowImageRect('frame')[2], cv2.getWindowImageRect('frame')[3])
        
    else:
        table.putNumberArray(
            "April", 
            [0, 0, 0, 0] # error message ig
            # i mean even if we get this it means its not terrible lol
        )
    
    if cv2.waitKey(100) == 13:
        break

# loop over; clean up and dump the last updated frame for convenience of debugging

cv2.destroyAllWindows()