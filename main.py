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

from robotpy_apriltag import AprilTagDetector, AprilTagPoseEstimator
from networktables import NetworkTables

# import logging
import numpy
import math
# logging.basicConfig(level=logging.DEBUG)

cap = cv2.VideoCapture(0)
detector = AprilTagDetector()
detector.addFamily("tag16h5")
estimator = AprilTagPoseEstimator(AprilTagPoseEstimator.Config(
        0.05, # tag size, in meters
        1008, # focal length in pixels: length in pixels * distance / actual length
         # around 7mm for 5mm object to be 720 pixels --> 
        1008, # has to be experimentally determined, but fx and fy are generally the same
        640, # literally just the resolution divided by 2 :D
        360
    ))

limit = 200
minconfidence = 150

draw = False

NetworkTables.initialize(server="10.9.91.2") # IP Address of RoboRio, I think
                    # might have to be roborio-991-frc.local, I'm not sure

table = NetworkTables.getTable("SmartDashboard")

while (True):
    success, frame = cap.read()
    
    if (success):
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # convert to grayscale
        
        gray = cv2.inRange(gray, numpy.mat([limit]), numpy.mat([255]))

        results = detector.detect(gray)
        
        data = []
        
        for result in results:
            if (result.getCorner(0).x - result.getCorner(2).x) * (result.getCorner(0).x - result.getCorner(2).x) + (result.getCorner(0).y - result.getCorner(2).y) * (result.getCorner(0).y - result.getCorner(2).y) < 10: continue # if the diagonal distance is too small
            
            if min(result.getCorner(0).x, result.getCorner(1).x, result.getCorner(2).x, result.getCorner(3).x, result.getCorner(0).y, result.getCorner(1).y, result.getCorner(2).y, result.getCorner(3).y) < 50: continue
            
            if min(1280 - result.getCorner(0).x, 1280 - result.getCorner(1).x, 1280 - result.getCorner(2).x, 1280 - result.getCorner(3).x, 720 - result.getCorner(0).y, 720 - result.getCorner(1).y, 720 - result.getCorner(2).y, 720 - result.getCorner(3).y) < 15: continue # if we're too close to the edge of the screen
            
            if (result.getDecisionMargin() < minconfidence): continue # if not very confident
            
            poses = estimator.estimateOrthogonalIteration(result, 10)
            
            if draw:
                
                x = int(result.getCenter().x * 10 + 0.5) / 10.0
                y = int(result.getCenter().y * 10 + 0.5) / 10.0
                angle1 = 0 - math.atan(poses.pose1.rotation().x_degrees / poses.pose1.rotation().y_degrees)
                
                if (poses.pose1.rotation().y_degrees > 0): 
                    angle1 += math.pi
                
                multiplier = math.sqrt(poses.pose1.rotation().x_degrees * poses.pose1.rotation().x_degrees + poses.pose1.rotation().y_degrees * poses.pose1.rotation().y_degrees) / 127.2792206136
                length = 20 / poses.pose1.translation().z
                
                points = []
                for i in range(4):
                    points.append([
                        result.getCorner(i).x, result.getCorner(i).y
                    ])
                
                for j in range(4): # boundary lines
                    cv2.line(
                        frame, 
                        (int(points[j][0] + 0.5), int(points[j][1] + 0.5)), 
                        (int(points[(j + 1) % 4][0] + 0.5), int(points[(j + 1) % 4][1] + 0.5)), 
                        (0, 255, 0), # BGR Color
                        10
                    )
                
                for i in range(2): # diagonal lines
                    cv2.line(
                        frame, 
                        (int(points[i][0] + 0.5), int(points[i][1] + 0.5)), 
                        (int(points[i + 2][0] + 0.5), int(points[i + 2][1] + 0.5)), 
                        (0, 0, 255), # BGR Color
                        10
                    )
                
                cv2.line( # 3d line
                    frame, 
                    (int(x + 0.5), int(y + 0.5)), 
                    (int(x + length * math.cos(angle1) * multiplier + 0.5), int(y + length * math.sin(angle1) * multiplier + 0.5)), 
                    (255, 0, 0), # BGR Color
                    10
                )
            
            # modular lists (ex. each list of 4 elements is actually one element)
            data.append(round(result.getId())) # id of the tag
            data.append(round(poses.pose1.rotation().y_degrees, 2)) # how far we need to turn right to see it head on
            data.append(round(poses.pose1.translation().x * 100, 2)) # how far we need to move to right to be dead on with the thing
            data.append(round(poses.pose1.translation().z * 100, 2)) #  z depth, assuming we are looking head on
            # all measurements in degrees or cm
            
        table.putNumberArray(
            "April", 
            data
        )
        
        if (draw):

            # print("Frame rate: ", int(cap.get(cv2.CAP_PROP_FPS)), "FPS")
            # print(cv2.getWindowImageRect('frame')[2], cv2.getWindowImageRect('frame')[3]) # resolution of camera
            
            print(data)
            print(table.getNumberArray("April", "None"))
        
            cv2.imshow('frame', frame) # puts it in a window  
            cv2.imshow('gray', gray) # puts it in a window  
        
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