#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from gazebo_msgs.srv import GetModelState
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform
from std_msgs.msg import String
import numpy as np



class tag_info(object):

    aruco_id = None 
    position_x = None 
    position_y = None 
    position_z = None 

list = []

for i in range(6):
    list.append(tag_info())


class arm(tag_info):

    tag_id = None
    trans_matrix = None

    def __init__(self):
        self.manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
        self.group_variable_values = self.manipulator_group.get_current_joint_values()
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()     
        self.search_pattern()
        rospy.spin()
        moveit_commander.roscpp_shutdown()

    def transformation(self):

        for m in self.msg.transforms:
            trans = m.transform.translation
            rot = m.transform.rotation
            self.tag_id = m.fiducial_id
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

            self.trans_matrix = np.array([[r00, r01, r02, t.translation.x],
                           [r10, r11, r12, t.translation.y],
                           [r20, r21, r22, t.translation.z],
                           [0, 0, 0, 1]])


    def search_pattern(self):

        self.group_variable_values[0] = 0.4887
        self.group_variable_values[1] = -1.0123
        self.group_variable_values[2] = 2.1817
        self.group_variable_values[3] = -2.7576
        self.group_variable_values[4] = -1.5359
        self.group_variable_values[5] = 0.5236
        self.manipulator_group.set_joint_value_target(self.group_variable_values)

        plan2 = self.manipulator_group.plan()
        self.manipulator_group.go(wait=True)

        self.msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
        self.transformation()

        waypoint_1 = np.array([[ 0.03398006,  0.99022806, -0.13524068, 0.21311],
                                    [ 0.99939653,  -0.03461061, -0.00228688, 0.3294],
                                    [-0.00694507, -0.13508159,  -0.99081012, 0.087242 ],
                                    [ 0, 0, 0, 1]])

        aruco = np.matmul(waypoint_1, self.trans_matrix)

        list[0].tag_info() = self.tag_id
        list[0].position_x = aruco[0,3]
        list[0].position_y = aruco[1,3]
        list[0].position_z = aruco[2,3]

        print (list[0].tag_info())
        print (list[0].position_x, list[0].position_y, list[0].position_z)
                
        self.group_variable_values[0] = 0.5759
        self.group_variable_values[1] = -1.1519
        self.group_variable_values[2] = 1.9197
        self.group_variable_values[3] = -3.8397
        self.group_variable_values[4] = -2.1642
        self.group_variable_values[5] = 0.0873
        self.manipulator_group.set_joint_value_target(self.group_variable_values)

        plan2 = self.manipulator_group.plan()
        self.manipulator_group.go(wait=True)

        self.msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
        self.transformation()

        waypoint_2 = np.array([[ 0.19455169,  0.98017262,  0.03759805, 0.237287],
                                    [ 0.05673758, -0.04951037,  0.99716169, 0.320772],
                                    [ 0.97925116,  -0.1918651,  -0.06524405, 0.311133],
                                    [ 0, 0,0,1]])

        aruco = np.matmul(waypoint_2, self.trans_matrix)

        list[1].tag_info() = self.tag_id
        list[1].position_x = aruco[0,3]
        list[1].position_y = aruco[1,3]
        list[1].position_z = aruco[2,3]
        
        print (list[1].tag_info())
        print (list[1].position_x, list[1].position_y, list[1].position_z)

        self.group_variable_values[0] = -0.5061
        self.group_variable_values[1] = -1.4486
        self.group_variable_values[2] = 2.3387
        self.group_variable_values[3] = -3.9793
        self.group_variable_values[4] = -1.0821
        self.group_variable_values[5] = 1.5533
        self.manipulator_group.set_joint_value_target(self.group_variable_values)

        plan2 = self.manipulator_group.plan()
        self.manipulator_group.go(wait=True)

        self.msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
        self.transformation()

        waypoint_3 = np.array([[ 0.21890384, -0.01083373,  0.97569494, 0.28562],
                                    [-0.01709427, -0.99982748, -0.00726667, 0.014619],
                                    [ 0.97560534, -0.01508804, -0.21903742, 0.39686],
                                    [0, 0, 0, 1]])

        aruco = np.matmul(waypoint_3, self.trans_matrix)

        list[2].tag_info() = self.tag_id
        list[2].position_x = aruco[0,3]
        list[2].position_y = aruco[1,3]
        list[2].position_z = aruco[2,3]
        
        print (list[2].tag_info())
        print (list[2].position_x, list[2].position_y, list[2].position_z)
        
        self.group_variable_values[0] = -1.0996
        self.group_variable_values[1] = -1.7802
        self.group_variable_values[2] = 2.042
        self.group_variable_values[3] = -2.3387
        self.group_variable_values[4] = -1.1519
        self.group_variable_values[5] = 0.9774
        self.manipulator_group.set_joint_value_target(self.group_variable_values)

        plan2 = self.manipulator_group.plan()
        self.manipulator_group.go(wait=True)

        self.msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
        self.transformation()

        waypoint_4 = np.array([[ 0.82095203, -0.34775911,  0.45288926, 0.31375],
                                    [-0.37869337, -0.92521149, -0.02398621, -0.19832],
                                    [ 0.42735982 , -0.15181438,  -0.89124431, 0.36504],
                                    [0 , 0, 0, 1]])

        aruco = np.matmul(waypoint_4, self.trans_matrix)

        list[3].tag_info() = self.tag_id
        list[3].position_x = aruco[0,3]
        list[3].position_y = aruco[1,3]
        list[3].position_z = aruco[2,3]
        
        print (list[3].tag_info())
        print (list[3].position_x, list[3].position_y, list[3].position_z)

        self.group_variable_values[0] = -1.4835
        self.group_variable_values[1] = -0.9948
        self.group_variable_values[2] = 2.0595
        self.group_variable_values[3] = -2.6878
        self.group_variable_values[4] = -1.5533
        self.group_variable_values[5] = 1.6406
        self.manipulator_group.set_joint_value_target(self.group_variable_values)

        plan2 = self.manipulator_group.plan()
        self.manipulator_group.go(wait=True)

        self.msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
        self.transformation()

        waypoint_5 = np.array([[ 0.02055522, -0.99214174, -0.12335268, 0.14315],
                                    [-0.99960762, -0.01811605, -0.02099216, -0.40162],
                                    [ 0.01859367,  0.12373694, -0.99214078, 0.098798],
                                    [0, 0, 0,1]])

        aruco = np.matmul(waypoint_5, self.trans_matrix)

        list[4].tag_info() = self.tag_id
        list[4].position_x = aruco[0,3]
        list[4].position_y = aruco[1,3]
        list[4].position_z = aruco[2,3]
        
        print (list[4].tag_info())
        print (list[4].position_x, list[4].position_y, list[4].position_z)

        self.group_variable_values[0] = -1.7453
        self.group_variable_values[1] = -1.4835
        self.group_variable_values[2] = 2.5831
        self.group_variable_values[3] = -2.042
        self.group_variable_values[4] = -0.1047
        self.group_variable_values[5] = 0.87266
        self.manipulator_group.set_joint_value_target(self.group_variable_values)

        plan2 = self.manipulator_group.plan()
        self.manipulator_group.go(wait=True)
        
        self.msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
        self.transformation()

        waypoint_6 = np.array([[ 0.06691152, -0.9964496,   0.05098173, 0.14799],
                                    [-0.09140992, -0.05700961, -0.99417484, -0.29945],
                                    [ 0.99355683,  0.0618674, -0.09490603, 0.15353],
                                    [0, 0, 0, 1]])

        aruco = np.matmul(waypoint_6, self.trans_matrix)

        list[5].tag_info() = self.tag_id
        list[5].position_x = aruco[0,3]
        list[5].position_y = aruco[1,3]
        list[5].position_z = aruco[2,3]
        
        print (list[5].tag_info())
        print (list[5].position_x, list[5].position_y, list[5].position_z)

        self.group_variable_values[0] = 0
        self.group_variable_values[1] = -1.570796
        self.group_variable_values[2] = 0
        self.group_variable_values[3] = -1.570796
        self.group_variable_values[4] = 0
        self.group_variable_values[5] = 0
        self.manipulator_group.set_joint_value_target(self.group_variable_values)

        plan2 = self.manipulator_group.plan()
        self.manipulator_group.go(wait=True)
       
        self.msg = rospy.wait_for_message("fiducial_transforms", FiducialTransformArray)
        self.transformation()

if __name__ == "__main__":
    rospy.init_node('pickplace', anonymous=True)
    obj = arm()


