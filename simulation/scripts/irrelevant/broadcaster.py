#!/usr/bin/env python
import rospy
import tf
import roslib
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist, Transform


def pose_cb(msg):
    br = tf.TransformBroadcaster()

    for m in msg.transforms:

        trans = m.transform.translation
        rot = m.transform.rotation
        t = Transform()
        t.translation.x = trans.x
        t.translation.y = trans.y
        t.translation.z = trans.z
        t.rotation.x = rot.x
        t.rotation.y = rot.y
        t.rotation.z = rot.z
        t.rotation.w = rot.w

        br.sendTransform((t.translation.x, t.translation.y, t.translation.z), (t.rotation.x, t.rotation.y,
                         t.rotation.z, t.rotation.w), rospy.Time.now(), "/camera_camera_gazebo", "camera_camera")

        print (t.translation.x, t.translation.y, t.translation.z,
               t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)


if __name__ == '__main__':
    rospy.init_node('fiducial_tf_broadcaster')

    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, pose_cb)
    rospy.spin()
