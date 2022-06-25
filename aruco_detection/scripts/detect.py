#!/usr/bin/env python3

import rospy
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import glob
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_markers",Image, queue_size=1)
    self.id_pub = rospy.Publisher("/arudo_ID", String, queue_size=1)
    self.rot_pub = rospy.Publisher("/rvec", String, queue_size=1)
    self.trans_pub = rospy.Publisher("/tvec", String, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #CALCULATE CAMERA MATRIX AND DISTORTION COEFFICIENT
    
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob('/home/saipranav/erc_ws/src/ROS_aruco_detection/aruco_detection/src/calib_result.jpg')
    for fname in images:
        img2 = cv.imread(fname)
        gray2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray2, (7,6), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray2,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            # Draw and display the corners
            cv.drawChessboardCorners(img2, (7,6), corners2, ret)

    ret, mtx, dist, rotation, translation = cv.calibrateCamera(objpoints, imgpoints, gray2.shape[::-1], None, None)
    markers_img, ids_list, rot, trans = self.detect_aruco(cv_image, mtx, dist)

    # if ids_list is None:
    #   self.id_pub.publish(ids_list)
    # else:
    #   ids_str = ''.join(str(e) for e in ids_list)
    #   self.id_pub.publish(ids_str)

    try:
      self.id_pub.publish(str(ids_list))
    except CvBridgeError as e:
      print(e)

    

    try:
      self.rot_pub.publish(str(rot))
    except CvBridgeError as e:
      print(e)

    try:
      self.trans_pub.publish(str(trans))
    except CvBridgeError as e:
      print(e)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
    except CvBridgeError as e:
      print(e)
  
  def detect_aruco(self, img, cameraMatrix, distCoeff):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    output = aruco.drawDetectedMarkers(img, corners, ids)  # detect the aruco markers and display its aruco id.
    rvecs, tvecs = aruco.estimatePoseSingleMarkers(corners, 0.02, cameraMatrix, distCoeff)
    return output, ids, rvecs, tvecs

def main():
  rospy.init_node('Detection', anonymous=True)
  ic = image_converter()
  rospy.spin()
  
if __name__ == '__main__':
    main()
