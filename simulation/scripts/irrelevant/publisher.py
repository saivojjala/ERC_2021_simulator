#!/usr/bin/env python
import rospy
from std_msgs.msg import String 

def gripper():
    rospy.init_node('gripper', anonymous=True)
    pub = rospy.Publisher("/gripper_command", String, queue_size=10)

    while not rospy.is_shutdown():
        position = "semi_open"
        pub.publish(position)

if __name__ == "__main__":
    gripper()
