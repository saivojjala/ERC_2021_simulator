#! /usr/bin/env python

import rospy
import sys
import numpy as np
import pandas as pd
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gazebo_msgs.srv import GetModelState
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform

global sub

class tag_info():
    aruco_id = None 
    position_x = None 
    position_y = None 
    position_z = None 

list = []

for i in range(15):
    list.append(tag_info())


def main():

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    group_variable_values = manipulator_group.get_current_joint_values()
    moveit_commander.roscpp_initialize(sys.argv)

    joint_constraint = moveit_msgs.msg.JointConstraint()
    joint_constraint.joint_name = manipulator_group.get_joints()[4]
    joint_constraint.tolerance_above =  2.30383
    joint_constraint.tolerance_below = -3.1415
    joint_constraint_list = []
    joint_constraint_list.append(joint_constraint)

    constraint_list = moveit_msgs.msg.Constraints()
    constraint_list.name = 'wrist_2_joint'
    constraint_list.joint_constraints = joint_constraint_list
    manipulator_group.set_path_constraints

    group_variable_values[0] = 0
    group_variable_values[1] = -2.0944
    group_variable_values[2] = 1.74533
    group_variable_values[3] = 0.349
    group_variable_values[4] =  2.35
    group_variable_values[5] = -1.5707
    manipulator_group.set_joint_value_target(group_variable_values)
    
    manipulator_group.go(wait=True)

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node('maintenance', anonymous=True)
    main()
