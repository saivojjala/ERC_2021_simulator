#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from simulation.srv import *
from fiducial_msgs.msg import FiducialTransformArray


def client():

    rospy.wait_for_service('aruco_pose')
    data = rospy.ServiceProxy('aruco_pose', ArUcoPose)
    resp = data()

    print (resp.tag_id)


if __name__== "__main__":
    client()