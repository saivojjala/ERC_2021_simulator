#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import csv
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# filename="markers.csv"
# fields=[]
# rows=[]
# with open(filename,'r') as csvfile:
#     csvreader=csv.reader(csvfile)
#     fields=next(csvreader)
#     for row in csvreader:
        # rows.append(row)

def go_to_pose(pose_x, pose_y, pose_z):
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")

    pose_target = geometry_msgs.msg.PoseStamped()
    pose_target.header.frame_id = "base_link"
    pose_target.pose.orientation.w = 0.7073883 
    pose_target.pose.orientation.x = 0 
    pose_target.pose.orientation.y = 0.7068252 
    pose_target.pose.orientation.z = 0 
    pose_target.pose.position.x = float(pose_x) + 0.07
    pose_target.pose.position.y = float(pose_y)
    pose_target.pose.position.z = float(pose_z) + 0.915

    manipulator_group.set_pose_target(pose_target)
    plan1 = manipulator_group.go()
    manipulator_group.stop()
    manipulator_group.clear_pose_targets()


def gripper_control(mode):
    pub = rospy.Publisher("/gripper_command", String, queue_size=10)
    rate = rospy.Rate(10)

    t0 = rospy.Time.now().to_sec()
    t1 = 0
    while t1 != 10:   
        t1 = rospy.Time.now().to_sec()
        t1 = t1 - t0     
        pub.publish(mode)

# if __name__=="__main__":
rospy.init_node("objective3")

filename= rospy.get_param('~file')
fields=[]
rows=[]
with open(filename,'r') as csvfile:
    csvreader=csv.reader(csvfile)
    fields=next(csvreader)
    for row in csvreader:
        rows.append(row)
        
x = rows[10][0]
y = rows[10][1]
z = rows[10][2]

go_to_pose(x, y, z) 
gripper_control('semi_open')
manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
scale = 1
waypoints = []
wpose = manipulator_group.get_current_pose().pose
wpose.position.z -= scale * 0.05
waypoints.append(copy.deepcopy(wpose))
(plan, fraction) = manipulator_group.compute_cartesian_path(
                                waypoints,  
                                0.01,        
                                0.0)
manipulator_group.execute(plan, wait=True)  

rospy.sleep(5)
moveit_commander.roscpp_shutdown()
