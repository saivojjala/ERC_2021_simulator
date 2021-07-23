#!/usr/bin/env python
from simulation.srv import ArUcoPose, ArUcoPoseResponse
import rospy
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray
   
def retrieve_pose(req):

    # for m in req.transform:
    trans = m.transform.translation
    rot = m.transform.rotation
    tag_id = m.fiducial_id
    t = Transform()

    t.translation.x = trans.x
    t.translation.y = trans.y
    t.translation.z = trans.z
    t.rotation.x = rot.x
    t.rotation.y = rot.y
    t.rotation.z = rot.z
    t.rotation.w = rot.w

    return ArUcoPoseResponse(tag_id, t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z)


def pose_server():
    rospy.init_node('aruco_pose_server')
    service = rospy.Service('aruco_pose', ArUcoPose, retrieve_pose)
    rospy.spin()
   
if __name__ == "__main__":
    pose_server()