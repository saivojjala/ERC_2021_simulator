#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
import copy
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#initilize moveit commander(to communicate with the move group)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('home', anonymous=True)
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()
manipulator_group = moveit_commander.MoveGroupCommander("manipulator")

group_variable_values = manipulator_group.get_current_joint_values()
#home
group_variable_values[0] = 0
group_variable_values[1] = -1.570796
group_variable_values[2] = 0
group_variable_values[3] = -1.570796
group_variable_values[4] = 0
group_variable_values[5] = 0

manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)

rospy.sleep(5)
moveit_commander.roscpp_shutdown()