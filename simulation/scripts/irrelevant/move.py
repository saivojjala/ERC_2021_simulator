#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

manipulator_group = moveit_commander.MoveGroupCommander("manipulator")

pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base_link"
pose_target.pose.orientation.w = 0.68347
pose_target.pose.orientation.x = 0.68566
pose_target.pose.orientation.y = 0.18243
pose_target.pose.orientation.z = 0.17162
pose_target.pose.position.x = 0.429-0.20
pose_target.pose.position.y = 0.147
pose_target.pose.position.z = 0.275

# (0.4296804229511878, 0.1477133113564625, 0.2758523865489008)

manipulator_group.set_pose_target(pose_target)
plan1 = manipulator_group.go()
manipulator_group.stop()
manipulator_group.clear_pose_targets()

rospy.spin()
moveit_commander.roscpp_shutdown()