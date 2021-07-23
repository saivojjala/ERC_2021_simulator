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

for i in range(7):
    list.append(tag_info())


def main():

    manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
    group_variable_values = manipulator_group.get_current_joint_values()
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()     

    #imu_scan
    group_variable_values[0] = 4.36332
    group_variable_values[1] = -1.48353
    group_variable_values[2] = -2.00713
    group_variable_values[3] = -1.08210
    group_variable_values[4] = -4.71239
    group_variable_values[5] = -5.02655
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_1 = np.array([[ 0.03568882,  0.99830891, -0.04587327, 0.202328],
                            [ 0.99844397, -0.03365225,  0.04445202, 0.30734],
                            [ 0.04283316, -0.04738838, -0.99795774, 0.264114],
                            [0,0,0,1]])

    aruco = np.matmul(waypoint_1, matrix)

    list[0].aruco_id = A_id
    list[0].position_x = aruco[0,3]
    list[0].position_y = aruco[1,3]
    list[0].position_z = aruco[2,3]

    print (list[0].aruco_id)
    print (list[0].position_x, list[0].position_y, list[0].position_z)

    #left_pannel            
    group_variable_values[0] = 0.38397
    group_variable_values[1] = -1.74533
    group_variable_values[2] = 2.3038
    group_variable_values[3] = -3.6652
    group_variable_values[4] = -1.5533
    group_variable_values[5] = 1.6232
    manipulator_group.set_joint_value_target(group_variable_values)
    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_2 = np.array([[ 0.2118823,  0.03302006,   0.97673763, 0.16728],
                            [ 0.38862657, -0.91985785, -0.0532069, 0.18534 ],
                            [ 0.89670291,  0.39085965, -0.20773341, 0.44858 ],
                            [0, 0,0 ,1]])


    aruco = np.matmul(waypoint_2, matrix)

    list[1].aruco_id = A_id
    list[1].position_x = aruco[0,3]
    list[1].position_y = aruco[1,3]
    list[1].position_z = aruco[2,3]
    
    print (list[1].aruco_id)
    print (list[1].position_x, list[1].position_y, list[1].position_z)

    #button_1
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

    #window_cover
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

    # ground_right
    group_variable_values[0] = -0.99484 #-1.4835
    group_variable_values[1] = -2.042035 #-0.9948
    group_variable_values[2] = 2.26893 #2.0595
    group_variable_values[3] = -1.98967 #-2.6878
    group_variable_values[4] = -1.6755 #-1.5533
    group_variable_values[5] = 2.1293 #1.6406
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)


    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_5 = np.array([[ 0.00812328, -0.99904004,  0.0432003, 0.194541 ],
                             [-0.99986798, -0.00872969, -0.01417216, -0.193188],
                             [ 0.01453587, -0.04307928, -0.99896591, 0.278131],
                             [0,0,0,1]])
    # waypoint_5 = np.array([[ 0.02055522, -0.99214174, -0.12335268, 0.14315],
    #                             [-0.99960762, -0.01811605, -0.02099216, -0.40162],
    #                             [ 0.01859367,  0.12373694, -0.99214078, 0.098798],
    #                             [0, 0, 0,1]])

    aruco = np.matmul(waypoint_5, matrix)

    list[4].aruco_id = A_id
    list[4].position_x = aruco[0,3]
    list[4].position_y = aruco[1,3]
    list[4].position_z = aruco[2,3]
    
    print (list[4].aruco_id)
    print (list[4].position_x, list[4].position_y, list[4].position_z)


    #inspection_window
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

    moveit_commander.roscpp_shutdown()

def save_id(msg):
    tag_id = 0
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

def swap(list,size):
    i=0 
    j=0
    for i in range(size):
        for j in range(0,size-i-1):
            if list[j].aruco_id>list[j+1].aruco_id:
                list[j],list[j+1]=list[j+1],list[j]
    return list        

if __name__ == "__main__":
    rospy.init_node('final_3', anonymous=True)
    main()
    list=swap(list,7)
    tagid=[]
    posx=[]
    posy=[]
    posz=[]
    i=0
    for i in range (7):
        tagid.append(list[i].aruco_id)
        posx.append(list[i].position_x)
        posy.append(list[i].position_y)
        posz.append(list[i].position_z)
    dict={'tag_id':tagid,'position_x':posx,'position_y':posy,'position_z':posz}
    df=pd.DataFrame(dict)
    df.to_csv('/markers3.csv',index=False)
    df.to_csv('/catkin_ws/src/ERC_2021_simulator/simulation/scripts/markers3.csv',index=False)
    print (df)