#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gazebo_msgs.srv import GetModelState

def main():
    rospy.init_node('pickplace', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # target()
    gripper_control('semi_open')
    model_state()

    # rospy.sleep(10)
    rospy.spin()
    moveit_commander.roscpp_shutdown()

def target():
    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")

    # put the arm at the 1st grasping position
    pose_target = geometry_msgs.msg.PoseStamped()
    pose_target.header.frame_id = "base_link"
    pose_target.pose.orientation.w = 0.7073883
    pose_target.pose.orientation.x = 0
    pose_target.pose.orientation.y = 0.7068252
    pose_target.pose.orientation.z = 0
    pose_target.pose.position.x = 0.21
    pose_target.pose.position.y = 0.2672
    pose_target.pose.position.z = 0.1

    # pose_target = geometry_msgs.msg.PoseStamped()
    # pose_target.header.frame_id = "world"
    # pose_target.pose.orientation.w = 0.103 #469690266
    # pose_target.pose.orientation.x = 0.000 #268478423153
    # pose_target.pose.orientation.y = -0.994 #632559991
    # pose_target.pose.orientation.z = 0.0001 #47378266295
    # pose_target.pose.position.x = -0.227 #541533497
    # pose_target.pose.position.y = -0.001 #54226288197
    # pose_target.pose.position.z = 0.469 #145334499

    # pose_target.pose.orientation.w = 0.5000482
    # pose_target.pose.orientation.x = -0.5
    # pose_target.pose.orientation.y = 0.4999518
    # pose_target.pose.orientation.z = 0.5
    # pose_target.pose.position.x = 0.458800
    # pose_target.pose.position.y = 0
    # pose_target.pose.position.z = 0.35

    manipulator_group.set_pose_target(pose_target)
    plan1 = manipulator_group.go()
    manipulator_group.stop()

def model_state():
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resp_coordinates = model_coordinates('robot','wrist_3_link')

    position_x = resp_coordinates.pose.position.x
    position_y = resp_coordinates.pose.position.y
    position_z = resp_coordinates.pose.position.z    

    print position_x, position_y, position_z

def gripper_control(mode):
    pub = rospy.Publisher("/gripper_command", String, queue_size=10)

    while not rospy.is_shutdown():
        pub.publish(mode)
        

if __name__ == "__main__":
    main()
