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

class arm:

    def __init__(self):
        self.manipulator_group = moveit_commander.MoveGroupCommander("manipulator")
        self.group_variable_values = self.manipulator_group.get_current_joint_values()
        # rospy.Subscriber("/gripper_mode", String, self.callback_mode)
        self.current_mode = 'open'
        self.rate = rospy.Rate(5)
        self.main()
        

    def main(self):
        
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        

        self.search_pattern()
        #target()
        # tag1=arucoid()
        # tag1.model_state()
        # tag1.show_values()
        #gripper_control('semi_open')
        #model_state()

        # rospy.sleep(10)
        rospy.spin()
        moveit_commander.roscpp_shutdown()

    def save_id(self, msg):
        
        for m in msg.transforms:
            trans = m.transform.translation
            rot = m.transform.rotation
            tag_id = m.fiducial_id
            t = Transform()

            t.translation.x = trans.x
            t.translation.y = trans.y
            t.translation.z = trans.z
            t.rotation.x = rot.x
            t.rotation.y = rot.y
            t.rotation.z = rot.z
            t.rotation.w = rot.w

            print (tag_id)
            print (t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)

    # def callback_mode(self, msg):
    #     self.current_mode = msg.data

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

        # rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.save_id)
        
        # # self.gripper_control('semi_open')
        
        # self.group_variable_values[0] = 0.5759
        # self.group_variable_values[1] = -1.1519
        # self.group_variable_values[2] = 1.9197
        # self.group_variable_values[3] = -3.8397
        # self.group_variable_values[4] = -2.1642
        # self.group_variable_values[5] = 0.0873
        # self.manipulator_group.set_joint_value_target(self.group_variable_values)

        # plan2 = self.manipulator_group.plan()
        # self.manipulator_group.go(wait=True)

        # rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.save_id)
        
        # self.group_variable_values[0] = -0.5061
        # self.group_variable_values[1] = -1.4486
        # self.group_variable_values[2] = 2.3387
        # self.group_variable_values[3] = -3.9793
        # self.group_variable_values[4] = -1.0821
        # self.group_variable_values[5] = 1.5533
        # self.manipulator_group.set_joint_value_target(self.group_variable_values)

        # plan2 = self.manipulator_group.plan()
        # self.manipulator_group.go(wait=True)

        # rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.save_id)
        
        
        # self.group_variable_values[0] = -1.0996
        # self.group_variable_values[1] = -1.7802
        # self.group_variable_values[2] = 2.042
        # self.group_variable_values[3] = -2.3387
        # self.group_variable_values[4] = -1.1519
        # self.group_variable_values[5] = 0.9774
        # self.manipulator_group.set_joint_value_target(self.group_variable_values)

        # plan2 = self.manipulator_group.plan()
        # self.manipulator_group.go(wait=True)


        # self.group_variable_values[0] = -1.4835
        # self.group_variable_values[1] = -0.9948
        # self.group_variable_values[2] = 2.0595
        # self.group_variable_values[3] = -2.6878
        # self.group_variable_values[4] = -1.5533
        # self.group_variable_values[5] = 1.6406
        # self.manipulator_group.set_joint_value_target(self.group_variable_values)

        # plan2 = self.manipulator_group.plan()
        # self.manipulator_group.go(wait=True)

        # self.group_variable_values[0] = -1.7453
        # self.group_variable_values[1] = -1.4835
        # self.group_variable_values[2] = 2.5831
        # self.group_variable_values[3] = -2.042
        # self.group_variable_values[4] = -0.1047
        # self.group_variable_values[5] = 0.87266
        # self.manipulator_group.set_joint_value_target(self.group_variable_values)

        # plan2 = self.manipulator_group.plan()
        # self.manipulator_group.go(wait=True)


        # self.group_variable_values[0] = 0
        # self.group_variable_values[1] = -1.570796
        # self.group_variable_values[2] = 0
        # self.group_variable_values[3] = -1.570796
        # self.group_variable_values[4] = 0
        # self.group_variable_values[5] = 0
        # self.manipulator_group.set_joint_value_target(self.group_variable_values)

        # plan2 = self.manipulator_group.plan()
        # self.manipulator_group.go(wait=True)

    def gripper_control(self, mode):
        pub = rospy.Publisher("/gripper_command", String, queue_size=10)
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.current_mode != mode:
                pub.publish(mode)
                
            else :
                break        
class arucoid:
    positionx=None
    positiony=None
    positionz=None
   

    def show_values(self):
        print "position x=",self.positionx
        print "position y=",self.positiony
        print "position z=",self.positionz
    def model_state(self):
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates('robot','wrist_3_link')

        self.positionx = resp_coordinates.pose.position.x
        self.positiony = resp_coordinates.pose.position.y
        self.positionz = resp_coordinates.pose.position.z    
       
if __name__ == "__main__":
    rospy.init_node('pickplace', anonymous=True)
    obj = arm()


