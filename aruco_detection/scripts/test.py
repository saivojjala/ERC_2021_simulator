#!/usr/bin/env python

import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
print("a")
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
print("b")
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
print("c")
images = glob.glob('/home/saipranav/catkin_ws/src/ROS_aruco_detection/aruco_detection/src/calib_result.jpg')
print("d")
for fname in images:
    print("e")
    img = cv.imread(fname)
    print("f")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    print("g")
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (7,6), None)
    print("h")
    # If found, add object points, image points (after refining them)
    if ret == True:
        print("i")
        objpoints.append(objp)
        print("j")
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        print("k")
        imgpoints.append(corners)
        print("l")
        # Draw and display the corners
        cv.drawChessboardCorners(img, (7,6), corners2, ret)
        print("m")
        cv.imshow('img', img)
        print("n")
        cv.waitKey(500)
        print("o")

print (objpoints)
print (imgpoints)
print("p")
cv.destroyAllWindows()