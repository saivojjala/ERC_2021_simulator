#! /usr/bin/env python

import rospy
import sys
import csv
import numpy as np
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

list.append(tag_info())
list.append(tag_info())
list.append(tag_info())
list.append(tag_info())
list.append(tag_info())
list.append(tag_info())

def main():

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    group_variable_values = manipulator_group.get_current_joint_values()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()     

    group_variable_values[0] = 0.4887
    group_variable_values[1] = -1.0123
    group_variable_values[2] = 2.1817
    group_variable_values[3] = -2.7576
    group_variable_values[4] = -1.5359
    group_variable_values[5] = 0.5236
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_1 = np.array([[ 0.03398006,  0.99022806, -0.13524068, 0.21311],
                                [ 0.99939653,  -0.03461061, -0.00228688, 0.3294],
                                [-0.00694507, -0.13508159,  -0.99081012, 0.087242 ],
                                [ 0, 0, 0, 1]])

    aruco = np.matmul(waypoint_1, matrix)

    list[0].aruco_id = A_id
    list[0].position_x = aruco[0,3]
    list[0].position_y = aruco[1,3]
    list[0].position_z = aruco[2,3]

    print (list[0].aruco_id)
    print (list[0].position_x, list[0].position_y, list[0].position_z)
            
    group_variable_values[0] = 0.314159
    group_variable_values[1] = -1.25664
    group_variable_values[2] = 2.28638
    group_variable_values[3] = -4.13643
    group_variable_values[4] = -1.51844
    group_variable_values[5] = 1.64061
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_2 = np.array([[ 0.2195349,   0.00850069,  0.9755662, 0.2239 ],
                            [ 0.35524536,  -0.93200981,  -0.07182131, 0.18888],
                            [ 0.90862666,  0.36233314, -0.20763068, 0.37044],
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_2, matrix)

    list[1].aruco_id = A_id
    list[1].position_x = aruco[0,3]
    list[1].position_y = aruco[1,3]
    list[1].position_z = aruco[2,3]
    
    print (list[1].aruco_id)
    print (list[1].position_x, list[1].position_y, list[1].position_z)

    group_variable_values[0] = -0.5061
    group_variable_values[1] = -1.4486
    group_variable_values[2] = 2.3387
    group_variable_values[3] = -3.9793
    group_variable_values[4] = -1.0821
    group_variable_values[5] = 1.5533
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_3 = np.array([[ 0.21890384, -0.01083373,  0.97569494, 0.28562],
                                [-0.01709427, -0.99982748, -0.00726667, 0.014619],
                                [ 0.97560534, -0.01508804, -0.21903742, 0.39686],
                                [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_3, matrix)

    list[2].aruco_id = A_id
    list[2].position_x = aruco[0,3]
    list[2].position_y = aruco[1,3]
    list[2].position_z = aruco[2,3]
    
    print (list[2].aruco_id)
    print (list[2].position_x, list[2].position_y, list[2].position_z)
    
    group_variable_values[0] = -1.0996
    group_variable_values[1] = -1.7802
    group_variable_values[2] = 2.042
    group_variable_values[3] = -2.3387
    group_variable_values[4] = -1.1519
    group_variable_values[5] = 0.9774
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_4 = np.array([[ 0.82095203, -0.34775911,  0.45288926, 0.31375],
                                [-0.37869337, -0.92521149, -0.02398621, -0.19832],
                                [ 0.42735982 , -0.15181438,  -0.89124431, 0.36504],
                                [0 , 0, 0, 1]])

    aruco = np.matmul(waypoint_4, matrix)

    list[3].aruco_id = A_id
    list[3].position_x = aruco[0,3]
    list[3].position_y = aruco[1,3]
    list[3].position_z = aruco[2,3]
    
    print (list[3].aruco_id)
    print (list[3].position_x, list[3].position_y, list[3].position_z)

    group_variable_values[0] = -1.4835
    group_variable_values[1] = -0.9948
    group_variable_values[2] = 2.0595
    group_variable_values[3] = -2.6878
    group_variable_values[4] = -1.5533
    group_variable_values[5] = 1.6406
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_5 = np.array([[ 0.02055522, -0.99214174, -0.12335268, 0.14315],
                                [-0.99960762, -0.01811605, -0.02099216, -0.40162],
                                [ 0.01859367,  0.12373694, -0.99214078, 0.098798],
                                [0, 0, 0,1]])

    aruco = np.matmul(waypoint_5, matrix)

    list[4].aruco_id = A_id
    list[4].position_x = aruco[0,3]
    list[4].position_y = aruco[1,3]
    list[4].position_z = aruco[2,3]
    
    print (list[4].aruco_id)
    print (list[4].position_x, list[4].position_y, list[4].position_z)

    group_variable_values[0] = -1.71042
    group_variable_values[1] = -0.401426
    group_variable_values[2] = 2.33874
    group_variable_values[3] = -4.76475
    group_variable_values[4] = -0.20944
    group_variable_values[5] = 1.16937
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)
    
    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_6 = np.array([[ 0.25355372,  0.00806933,  0.9672928, 0.17022 ],
                            [-0.32569184, -0.94086943,  0.09322028, -0.20891],
                            [ 0.91084867, -0.338674,   -0.23592519, 0.21088],
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_6, matrix)

    list[5].aruco_id = A_id
    list[5].position_x = aruco[0,3]
    list[5].position_y = aruco[1,3]
    list[5].position_z = aruco[2,3]
    
    print (list[5].aruco_id)
    print (list[5].position_x, list[5].position_y, list[5].position_z)

    group_variable_values[0] = 0
    group_variable_values[1] = -1.570796
    group_variable_values[2] = 0
    group_variable_values[3] = -1.570796
    group_variable_values[4] = 0
    group_variable_values[5] = 0
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    moveit_commander.roscpp_shutdown()

def csv():
    with open('aruco.csv', 'w') as f:
        write = csv.writer(f)
        write.writerow(list[0])

def save_id(msg):
    for m in msg.transforms:
        tag_id = m.fiducial_id
    
    return tag_id

def transformation(msg):
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

        q0 = t.rotation.w
        q1 = t.rotation.x 
        q2 = t.rotation.y
        q3 = t.rotation.z

        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 + q0 * q3)
        r02 = 2 * (q1 * q3 - q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 - q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 + q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 + q0 * q2)
        r21 = 2 * (q2 * q3 - q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1

        trans_matrix = np.array([[r00, r01, r02, t.translation.x],
                        [r10, r11, r12, t.translation.y],
                        [r20, r21, r22, t.translation.z],
                        [0, 0, 0, 1]])

        return trans_matrix

if __name__ == "__main__":
    rospy.init_node('maintenance', anonymous=True)
    main()


