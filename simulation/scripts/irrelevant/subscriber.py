#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from fiducial_msgs.msg import FiducialTransformArray

def callback(data):
    
    for m in data.transforms:
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

        print (tag_id)
        print (t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    
def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
