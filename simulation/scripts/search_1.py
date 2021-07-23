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
rospy.init_node('pickplace', anonymous=True)
robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()
manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
grasping_group = moveit_commander.MoveGroupCommander("gripper")

group_variable_values = manipulator_group.get_current_joint_values()
    

group_variable_values[0] = -2.9322
group_variable_values[1] = -1.2043
group_variable_values[2] = 0.3142
group_variable_values[3] = -2.5133
group_variable_values[4] = 2.0071
group_variable_values[5] = -4.8171
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)

group_variable_values = manipulator_group.get_current_joint_values()
group_variable_values[0] = -3.6826
group_variable_values[1] = -0.8552
group_variable_values[2] = -1.2217
group_variable_values[3] = -1.6057
group_variable_values[4] = 1.8675
group_variable_values[5] = 0
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)

group_variable_values = manipulator_group.get_current_joint_values()
group_variable_values[0] = -3.5954
group_variable_values[1] = -0.2269
group_variable_values[2] = -1.4312
group_variable_values[3] = 1.5708
group_variable_values[4] = -1.6232
group_variable_values[5] = 0
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)
'''
group_variable_values[0] = -3.3685
group_variable_values[1] = -1.3614
group_variable_values[2] = -1.9548
group_variable_values[3] = -0.6808
group_variable_values[4] = -4.66
group_variable_values[5] = -3.2463
manipulator_group.set_joint_value_target(group_variable_values)
plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)
'''
group_variable_values = manipulator_group.get_current_joint_values()
group_variable_values[0] = -3.5954
group_variable_values[1] = -1.6581
group_variable_values[2] = -0.9599
group_variable_values[3] = 1.8326
group_variable_values[4] = -1.6232
group_variable_values[5] = 0.0175
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)

group_variable_values = manipulator_group.get_current_joint_values()
group_variable_values[0] = -4.6251
group_variable_values[1] = -1.8326
group_variable_values[2] = -1.309
group_variable_values[3] = 1.8151
group_variable_values[4] = -1.6232
group_variable_values[5] = 0.0175
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)


group_variable_values = manipulator_group.get_current_joint_values()
group_variable_values[0] = -1.658
group_variable_values[1] = -1.222
group_variable_values[2] = -0.803
group_variable_values[3] = 1.134
group_variable_values[4] = -1.623
group_variable_values[5] = 0
manipulator_group.set_joint_value_target(group_variable_values)
'''
group_variable_values = manipulator_group.get_current_joint_values()
group_variable_values[0] = 0
group_variable_values[1] = -1.5708
group_variable_values[2] = 0
group_variable_values[3] = 0
group_variable_values[4] = -1.472
group_variable_values[5] = 0
manipulator_group.set_joint_value_target(group_variable_values)
plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)
'''

group_variable_values = manipulator_group.get_current_joint_values()
group_variable_values[0] = -5.689
group_variable_values[1] = 0.785
group_variable_values[2] = -1.989
group_variable_values[3] = -0.331
group_variable_values[4] = -1.553
group_variable_values[5] = -5.602
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)


rospy.sleep(20)
moveit_commander.roscpp_shutdown()