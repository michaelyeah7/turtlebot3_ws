#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose

def deleteModel(name):
    rospy.wait_for_service("gazebo/delete_model")
    delete_model_pox = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    # delete_model_pox('some_robo_name')
    delete_model_pox(name)
    # delete_model_pox('my_MB')

    print("delete successfully")