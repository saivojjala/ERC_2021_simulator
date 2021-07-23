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
    
group_variable_values[0] = 0.4887
group_variable_values[1] = -1.0123
group_variable_values[2] = 2.1817
group_variable_values[3] = -2.7576
group_variable_values[4] = -1.5359
group_variable_values[5] = 0.5236
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)


group_variable_values[0] = 0.314159
group_variable_values[1] = -1.25664
group_variable_values[2] = 2.28638
group_variable_values[3] = -4.13643
group_variable_values[4] = -1.51844
group_variable_values[5] = 1.64061
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)

group_variable_values[0] = -0.5061
group_variable_values[1] = -1.4486
group_variable_values[2] = 2.3387
group_variable_values[3] = -3.9793
group_variable_values[4] = -1.0821
group_variable_values[5] = 1.5533
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)


group_variable_values[0] = -1.0996
group_variable_values[1] = -1.7802
group_variable_values[2] = 2.042
group_variable_values[3] = -2.3387
group_variable_values[4] = -1.1519
group_variable_values[5] = 0.9774
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)


group_variable_values[0] = -1.4835
group_variable_values[1] = -0.9948
group_variable_values[2] = 2.0595
group_variable_values[3] = -2.6878
group_variable_values[4] = -1.5533
group_variable_values[5] = 1.6406
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)

group_variable_values[0] = -1.71042
group_variable_values[1] = -0.401426
group_variable_values[2] = 2.33874
group_variable_values[3] = -4.76475
group_variable_values[4] = -0.20944
group_variable_values[5] = 1.16937
manipulator_group.set_joint_value_target(group_variable_values)

plan2 = manipulator_group.plan()
manipulator_group.go(wait=True)


# group_variable_values[0] = 0
# group_variable_values[1] = -1.570796
# group_variable_values[2] = 0
# group_variable_values[3] = -1.570796
# group_variable_values[4] = 0
# group_variable_values[5] = 0
# manipulator_group.set_joint_value_target(group_variable_values)

# plan2 = manipulator_group.plan()
# manipulator_group.go(wait=True)


rospy.sleep(20)
moveit_commander.roscpp_shutdown()