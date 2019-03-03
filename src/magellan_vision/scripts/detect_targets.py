import cv2 as cv
import numpy as np
from scipy import ndimage

# Temporary for testing - hues from the images in rules document
hueArch = 39
huePole = 0
hueRamp = 180

# These should probably be ROSparams
hueMinArch = hueArch - 20
hueMaxArch = hueArch + 20
hueMinRamp = hueRamp - 100
hueMaxRamp = hueRamp + 100
hueMinPole = huePole - 20
hueMaxPole = huePole + 20
cannyThresh = 100
houghArchThresh = 70
verticalArchThresh = 10
houghPoleThresh = 40
verticalPoleThresh = 10
# I think these should be the same for all targets
# Sat should be wide for various outdoors conditions
satMin = 100
satMax = 255
# Low val = less color - raise min to reduce risk of picking up non-target objects
valMin = 0
valMax = 255

detectArch = True
detectRamp = True
detectPole = True

# Get the current frame from ROS
if detectArch:
    frame = cv.imread('Hoop.png', cv.IMREAD_COLOR)
    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    archThreshold = cv.inRange(frameHSV, (hueMinArch, satMin, valMin), (hueMaxArch, satMax, valMax))
    copy = np.copy(frame)

# Arch detection should look for two vertical lines and find the midpoint
    lines = cv.HoughLinesP(archThreshold, 1, np.pi/180, houghArchThresh)

    poles = []
    archMidPt = None
    for line in lines:
        line = line[0]
        if abs(line[0] - line[2]) < verticalArchThresh:
            x = (line[0] + line[2]) / 2
            sameLine = False
            for pole in poles:
                if abs(x - pole) < 10:
                    sameLine = True
            if not sameLine:
                poles.append(x)
                cv.line(frame, (line[0], line[1]), (line[2], line[3]), (0,0,255), 3)
                cv.imshow("lines", frame)
                cv.waitKey(0)    

    if len(poles) == 2:
        archMidPt = (poles[0] + poles[1]) / 2
        archWidth = abs(poles[0] - poles[1])
        print('Center of arch is {}, arch width is {}'.format(archMidPt, archWidth))

if detectRamp:
    frame = cv.imread('Ramp.png', cv.IMREAD_COLOR)
    frameGray = cv.imread('Ramp.png', cv.IMREAD_GRAYSCALE)
    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    rampThreshold = cv.inRange(frameHSV, (hueMinRamp, satMin, valMin), (hueMaxRamp, satMax, valMax))
    center = ndimage.measurements.center_of_mass(rampThreshold)
    centerImage = cv.circle(frame, (int(center[1]), int(center[0])), 10, (0,0,255), 3)
    cv.imshow("threshold", centerImage)
    cv.waitKey(0)

if detectPole:
    frame = cv.imread('Stanchion2.png', cv.IMREAD_COLOR)
    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    poleThreshold = cv.inRange(frameHSV, (hueMinPole, satMin, valMin), (hueMaxPole, satMax, valMax))
    cv.imshow("threshold", poleThreshold)
    lines = cv.HoughLinesP(poleThreshold, 1, np.pi/180, houghPoleThresh)
    for line in lines:
        line = line[0]
        cv.line(frame, (line[0], line[1]), (line[2], line[3]), (0,0,255), 3)
    cv.imshow("lines", frame)
    cv.waitKey(0)

    poles = [] 
    poleLengths = []
    for line in lines:
        line = line[0]
        if abs(line[0] - line[2]) < verticalPoleThresh:
            x = (line[0] + line[2]) / 2
            sameLine = False
            myLength = sqrt((line[0] - line[2])**2 + (line[1] - line[3])**2)
            for i in range(len(poles))
                if abs(x - poles[i]) < 10:
                    sameLine = True
                    if myLength > poleLengths[i]:
                        poles[i] = x
                        poleLengths[i] = myLength

            if not sameLine:
                poles.append(x)
                cv.line(frame, (line[0], line[1]), (line[2], line[3]), (0,0,255), 3)
                cv.imshow("lines", frame)
                cv.waitKey(0)

    for pole in poles:
        print('There is a pole at x={}'.format(pole))
    
    #detector = cv.SimpleBlobDetector_create()
    #keypoints = detector.detect(rampThreshold)
    #for kp in keypoints:
    #    x, y = kp.pt
    #    imKeypoints = cv.circle(frame, (int(x), int(y)), 10, (0,0,255), 3)
    #cv.imshow("blobs", imKeypoints)
    #cv.waitKey(0)

# # Ramp detection should find the center of the blob
# 
# poleThreshold = cv.inRange(frameHSV, (hueMinPole, satMin, valMin), (hueMaxPole, satMax, valMax))

# Pole detection should find the center of the blob
