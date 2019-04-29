#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from cStringIO import StringIO
from gazebo_utils import  deleteModel
import  time
from furniture_model import FurnitureModel

def customizeWall():
    # read all lines into a list
    f.seek(0)
    lines = []
    for line in f:
        # lines.append(line.rstrip('\n'))
        lines.append(line)

    # # customize your size
    # x = 5
    # y = 0.15
    # z = 2.5
    # sizeLine = '            <size> ' + str(x) + ' ' + str(y) + ' ' + str(z) + '</size>'
    #
    # # write lines from beginning
    # f.seek(0)
    # for line in lines:
    #     if (line.find('size') is not -1):
    #         line = sizeLine
    #     print line
    #     f.writelines(line + '\n')

    return lines

rospy.init_node('insert_object')

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0
# initial_pose.orientation.w = 0 #cos(theta/2)
# initial_pose.orientation.z = 1 #sin(theta/2)


f = open('/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/trash.sdf','rw+')
sdff = f.read()

# lines = customizeWall()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
# spawn_model_prox("my_wall", StringIO(''.join(lines)).read(), " ", initial_pose, "world")
spawn_model_prox("my_trash", sdff, " ", initial_pose, "world")

f.close()
print("spawn successfully")

time.sleep(5)

deleteModel("my_trash")