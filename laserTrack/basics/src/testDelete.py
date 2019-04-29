#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose


initial_pose = Pose()
initial_pose.position.x = 1
initial_pose.position.y = 1
initial_pose.position.z = 1

rospy.init_node("delete_object")
rospy.wait_for_service("gazebo/delete_model")
delete_model_pox = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
# delete_model_pox('some_robo_name')
delete_model_pox('my_wall')
# delete_model_pox('my_MB')


print("delete successfully")
