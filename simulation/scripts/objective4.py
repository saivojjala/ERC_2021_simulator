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
# grasping_group = moveit_commander.MoveGroupCommander("gripper")

pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base_link"
# put the arm at the 1st grasping position
pose_target.pose.orientation.w = 0.839976
pose_target.pose.orientation.x = 0.504429
pose_target.pose.orientation.y = 0.165694
pose_target.pose.orientation.z = 0.111966
pose_target.pose.position.x = 0.283182
pose_target.pose.position.y = 0.203364
pose_target.pose.position.z = 0.39029   

manipulator_group.set_pose_target(pose_target)
plan1 = manipulator_group.go()  #plan and execute
manipulator_group.stop()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()