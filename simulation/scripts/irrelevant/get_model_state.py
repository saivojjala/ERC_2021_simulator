#!/usr/bin/env python  

from gazebo_msgs.srv import GetModelState
import rospy

# class Block:
#     def __init__(self, name, relative_entity_name):
#         self._name = name
#         self._relative_entity_name = relative_entity_name

# class Tutorial:

#     _blockListDict = {
#         'block_a': Block('robot', 'wrist_3_link'),}

#     def show_gazebo_models(self):
#         try:
#             model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
#             for block in self._blockListDict.itervalues():
#                 blockName = str(block._name)
#                 resp_coordinates = model_coordinates(blockName, block._relative_entity_name)
        
#                 print(" POSE X : " + str(resp_coordinates.pose.position.x))
#                 print(" POSE Y : " + str(resp_coordinates.pose.position.y))
#                 print(" POSE Z : " + str(resp_coordinates.pose.position.z))
#                 print("ORIENTATION X : " + str(resp_coordinates.pose.orientation.x))
#                 print("ORIENTATION Y : " + str(resp_coordinates.pose.orientation.y))
#                 print("ORIENTATION Z : " + str(resp_coordinates.pose.orientation.z))
#                 print("ORIENTATION W : " + str(resp_coordinates.pose.orientation.w))

#         except rospy.ServiceException as e:
#             rospy.loginfo("Get Model State service call failed:  {0}".format(e))


# if __name__ == '__main__':
#     tuto = Tutorial()
#     tuto.show_gazebo_models()



if __name__ == '__main__':

    rospy.init_node("state", anonymous=True)
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    resp_coordinates = model_coordinates('robot', 'wrist_3_link')

    position_x = resp_coordinates.pose.position.x
    position_y = resp_coordinates.pose.position.y
    position_z = resp_coordinates.pose.position.z    

    print position_x, position_y, position_z