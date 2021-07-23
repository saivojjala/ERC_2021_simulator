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
                            [0,0,0,1]]
    )

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
    group_variable_values[0] = -0.10472
    group_variable_values[1] = -1.6057
    group_variable_values[2] = 1.98967
    group_variable_values[3] = -3.4732
    group_variable_values[4] = -1.44862
    group_variable_values[5] = 1.623156
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_3 = np.array([[ 0.22548577 ,-0.05397673 , 0.97274996, 0.288833],
                            [ 0.01451495 ,-0.9981671 , -0.05875169, 0.08784],
                            [ 0.97413822  ,0.0273671  ,-0.22428917, 0.483742],
                            [0,0,0,1]])

    aruco = np.matmul(waypoint_3, matrix)

    list[2].aruco_id = A_id
    list[2].position_x = aruco[0,3]
    list[2].position_y = aruco[1,3]
    list[2].position_z = aruco[2,3]
    
    print (list[2].aruco_id)
    print (list[2].position_x, list[2].position_y, list[2].position_z)

    #button_2
    group_variable_values[0] = -0.57596
    group_variable_values[1] = -1.69297
    group_variable_values[2] = 2.09439
    group_variable_values[3] = -3.47321
    group_variable_values[4] = -0.97738
    group_variable_values[5] = 1.570796
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_4 = np.array([[ 0.23103422 ,-0.03434747 , 0.97233928 ,0.29084],
                            [ 0.01622101 ,-0.99910179 ,-0.03914707,-0.0016116],
                            [ 0.97281052  ,0.02481663 ,-0.23026936,0.478119],
                            [0,0,0,1]])

    aruco = np.matmul(waypoint_4, matrix)

    list[3].aruco_id = A_id
    list[3].position_x = aruco[0,3]
    list[3].position_y = aruco[1,3]
    list[3].position_z = aruco[2,3]
    
    print (list[3].aruco_id)
    print (list[3].position_x, list[3].position_y, list[3].position_z)

    #button_3
    group_variable_values[0] = -0.92502
    group_variable_values[1] = -1.58825
    group_variable_values[2] = 2.02458
    group_variable_values[3] = -3.47321
    group_variable_values[4] = -0.62832
    group_variable_values[5] = 1.53589
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)
    
    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_5 = np.array([[ 0.23476259, -0.04472845 , 0.97102199, 0.294083 ],
                            [ 0.01675733, -0.9986061 , -0.05005043, -0.0931667],
                            [ 0.97190716 , 0.02802177 ,-0.23368759, 0.472484],
                            [0,0,0,1]])

    aruco = np.matmul(waypoint_5, matrix)

    list[4].aruco_id = A_id
    list[4].position_x = aruco[0,3]
    list[4].position_y = aruco[1,3]
    list[4].position_z = aruco[2,3]
    
    print (list[4].aruco_id)
    print (list[4].position_x, list[4].position_y, list[4].position_z)

    #button_4
    group_variable_values[0] = -0.92502 #-1.029744
    group_variable_values[1] = -0.82030 #-1.48353
    group_variable_values[2] = 2.09439 #2.44346
    group_variable_values[3] = -4.69494 #-4.01426
    group_variable_values[4] = -0.62832 #-0.610865
    group_variable_values[5] = 1.79769 #3.054323
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_6 = np.array([ [0.0118561074 , 0.00135429115 , 0.999214024 , 0.277802],
                            [ 0.0358802489 ,-0.999355234 , 0.000902436050 , -0.0936901],
                            [ 0.998570532 , 0.0358660362 ,-0.0133088179 , 0.307398],
                            [0,0,0,1]]
    )

    aruco = np.matmul(waypoint_6, matrix)

    list[5].aruco_id = A_id
    list[5].position_x = aruco[0,3]
    list[5].position_y = aruco[1,3]
    list[5].position_z = aruco[2,3]
    
    print (list[5].aruco_id)
    print (list[5].position_x, list[5].position_y, list[5].position_z)

    #button_5
    group_variable_values[0] = -0.57596 #-0.66323
    group_variable_values[1] =  -0.85521 #-1.22173
    group_variable_values[2] =  2.26893 #2.40855
    group_variable_values[3] =  -4.74729 #-4.24115
    group_variable_values[4] =  -0.95993 #-0.872664
    group_variable_values[5] =  1.69297 #1.55334
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)


    waypoint_7 = np.array([ [0.01805263 ,-0.01022708 , 0.99978487,0.274853],
                            [ 0.04536057 ,-0.99890971 ,-0.01103718,-0.000264169],
                            [ 0.99880769  ,0.04555006 ,-0.01756877, 0.307775],
                            [0,0,0,1]]
    )

    aruco = np.matmul(waypoint_7, matrix)

    list[6].aruco_id = A_id
    list[6].position_x = aruco[0,3]
    list[6].position_y = aruco[1,3]
    list[6].position_z = aruco[2,3]
    
    print (list[6].aruco_id)
    print (list[6].position_x, list[6].position_y, list[6].position_z)

    #button_6
    group_variable_values[0] = -0.12217 #-0.20944
    group_variable_values[1] = -0.83776 #-1.29154
    group_variable_values[2] = 2.19911 #2.3213
    group_variable_values[3] = -4.66003 #-4.11898
    group_variable_values[4] = -1.41372 #-1.32645
    group_variable_values[5] = 1.60570#0.03491
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_8 = np.array([[ 0.01981105, -0.00947242,  0.99975859, 0.27174],
                            [ 0.03839737, -0.9992102,  -0.01022811, 0.0893925],
                            [ 0.99906587,  0.03859074, -0.01943222, 0.307383],
                            [0,0,0,1]]
    )

    aruco = np.matmul(waypoint_8, matrix)

    list[7].aruco_id = A_id
    list[7].position_x = aruco[0,3]
    list[7].position_y = aruco[1,3]
    list[7].position_z = aruco[2,3]
    
    print (list[7].aruco_id)
    print (list[7].position_x, list[7].position_y, list[7].position_z)

    #button_7
    group_variable_values[0] = -2.33874 #-0.174533
    group_variable_values[1] = -2.18166 #-0.50615
    group_variable_values[2] = -2.26893 #2.26893
    group_variable_values[3] = 4.69494 #-4.85201
    group_variable_values[4] = -0.82030 #-1.36136
    group_variable_values[5] = -1.72788 #0.03491
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_9 = np.array([[-0.00268827, -0.0113514,   0.99993291, 0.2938],
                            [ 0.03931703, -0.9991636,  -0.01123694, 0.0784019],
                            [ 0.99922413,  0.03928414,  0.00313424, 0.162834],
                            [0,0,0,1]]
    )

    aruco = np.matmul(waypoint_9, matrix)

    list[8].aruco_id = A_id
    list[8].position_x = aruco[0,3]
    list[8].position_y = aruco[1,3]
    list[8].position_z = aruco[2,3]
    
    print (list[8].aruco_id)
    print (list[8].position_x, list[8].position_y, list[8].position_z)

    #button_8
    group_variable_values[0] = -2.67035 #-0.55851
    group_variable_values[1] = -2.18166 #-0.610865
    group_variable_values[2] = -2.30383 #2.37365
    group_variable_values[3] = 4.66003 #-4.85201
    group_variable_values[4] = -1.13446 #-1.04719
    group_variable_values[5] = -1.65806 #-1.640609
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)
    
    waypoint_10 = np.array([ [0.01618374 , 0.01286094,  0.9997866,0.296634 ],
                             [ 0.02692346 ,-0.99956031 , 0.01242223 , -0.00514957],
                             [ 0.99950676 , 0.02671668 ,-0.01652234 , 0.158541],
                             [0,0,0,1]]
    )

    aruco = np.matmul(waypoint_10, matrix)

    list[9].aruco_id = A_id
    list[9].position_x = aruco[0,3]
    list[9].position_y = aruco[1,3]
    list[9].position_z = aruco[2,3]
    
    print (list[9].aruco_id)
    print (list[9].position_x, list[9].position_y, list[9].position_z)

    #button_9
    group_variable_values[0] = -3.10669 #-0.785398
    group_variable_values[1] = -2.21657 #-0.593412
    group_variable_values[2] = -2.14675 #2.303835
    group_variable_values[3] = 4.53786 #-4.782202
    group_variable_values[4] = -1.57079 #-0.8203047
    group_variable_values[5] = -1.57079 #-3.211406
    manipulator_group.set_joint_value_target(group_variable_values)

    plan2 = manipulator_group.plan()
    manipulator_group.go(wait=True)

    sub = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
    matrix = transformation(sub)
    A_id = save_id(sub)

    waypoint_11 = np.array([[ 0.000115458026,  -0.000142960147,   0.999999791, 0.301704],
                            [  0.0349018458,  -0.999390734,  -0.000146909443, -0.101897],
                            [  0.999390546,   0.0349018622,  -0.000110781654, 0.162008],
                            [0,0,0,1]]
    )

    aruco = np.matmul(waypoint_11, matrix)

    list[10].aruco_id = A_id
    list[10].position_x = aruco[0,3]
    list[10].position_y = aruco[1,3]
    list[10].position_z = aruco[2,3]
    
    print (list[10].aruco_id)
    print (list[10].position_x, list[10].position_y, list[10].position_z)

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

    waypoint_12 = np.array([[ 0.82095203, -0.34775911,  0.45288926, 0.31375],
                                [-0.37869337, -0.92521149, -0.02398621, -0.19832],
                                [ 0.42735982 , -0.15181438,  -0.89124431, 0.36504],
                                [0 , 0, 0, 1]])

    aruco = np.matmul(waypoint_12, matrix)

    list[11].aruco_id = A_id
    list[11].position_x = aruco[0,3]
    list[11].position_y = aruco[1,3]
    list[11].position_z = aruco[2,3]
    
    print (list[11].aruco_id)
    print (list[11].position_x, list[11].position_y, list[11].position_z)

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

    waypoint_13 = np.array([[ 0.00812328, -0.99904004,  0.0432003, 0.194541 ],
                             [-0.99986798, -0.00872969, -0.01417216, -0.193188],
                             [ 0.01453587, -0.04307928, -0.99896591, 0.278131],
                             [0,0,0,1]]
)
    # waypoint_13 = np.array([[ 0.02055522, -0.99214174, -0.12335268, 0.14315],
    #                             [-0.99960762, -0.01811605, -0.02099216, -0.40162],
    #                             [ 0.01859367,  0.12373694, -0.99214078, 0.098798],
    #                             [0, 0, 0,1]])

    aruco = np.matmul(waypoint_13, matrix)

    list[12].aruco_id = A_id
    list[12].position_x = aruco[0,3]
    list[12].position_y = aruco[1,3]
    list[12].position_z = aruco[2,3]
    
    print (list[12].aruco_id)
    print (list[12].position_x, list[12].position_y, list[12].position_z)


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

    waypoint_14 = np.array([[ 0.25355372,  0.00806933,  0.9672928, 0.17022 ],
                            [-0.32569184, -0.94086943,  0.09322028, -0.20891],
                            [ 0.91084867, -0.338674,   -0.23592519, 0.21088],
                            [0, 0, 0, 1]])

    aruco = np.matmul(waypoint_14, matrix)

    list[13].aruco_id = A_id
    list[13].position_x = aruco[0,3]
    list[13].position_y = aruco[1,3]
    list[13].position_z = aruco[2,3]
    
    print (list[13].aruco_id)
    print (list[13].position_x, list[13].position_y, list[13].position_z)

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
    rospy.init_node('final_2', anonymous=True)
    main()
    list=swap(list,15)
    tagid=[]
    posx=[]
    posy=[]
    posz=[]
    i=0
    for i in range (15):
        tagid.append(list[i].aruco_id)
        posx.append(list[i].position_x)
        posy.append(list[i].position_y)
        posz.append(list[i].position_z)
    dict={'tag_id':tagid,'position_x':posx,'position_y':posy,'position_z':posz}
    df=pd.DataFrame(dict)
    df.to_csv('/markers2.csv',index=False)
    df.to_csv('/catkin_ws/src/ERC_2021_simulator/simulation/scripts/markers2.csv',index=False)
    print (df)