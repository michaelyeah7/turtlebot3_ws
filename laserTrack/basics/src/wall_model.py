#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from cStringIO import StringIO
from gazebo_utils import deleteModel
import time
from furniture_model import FurnitureModel


class WallModel(object):
    def __init__(self,file=''):
        self.name=''
        self.size=Point()#size of x,y,z
        self.pos=Point()#pos of s,y,z
        self.angle=0#angle of yaw
        self.sdfFileLoc=file
        self.xmlLines=self.readXML(self.sdfFileLoc)

    def readXML(self,fileLoc):
        f = open(fileLoc, 'rw+')
        f.seek(0)
        lines = []
        for line in f:
            lines.append(line.rstrip('\n'))
        f.close()
        return lines

    def writeXML(self,fileLoc,lines):
        f = open(fileLoc, 'rw+')
        f.seek(0)
        for line in lines:
            print line
            f.writelines(line + '\n')
        f.close()

    def spawnXML(self,pos,angle,size,name):
        lines=[]
        FirstPoseFlag = True
        self.name=name
        self.size=size
        self.angle=angle
        self.pos=pos
        sizeLine = '            <size> ' + str(size.x) + ' ' + str(size.y) + ' ' + str(size.z) + '</size>'
        poseline='      <pose frame=\'\'>'+str(pos.x)+' '+str(pos.y)+' '+str(pos.z)+ ' 0 0 '+str(angle)+ ' </pose>'
        nameline='  <model name=\''+str(name)+'\'>'
        for line in self.xmlLines:
            if (line.find('size') is not -1):
                line = sizeLine
            if (line.find('pose') is not -1) & FirstPoseFlag:
                FirstPoseFlag=False
                line=poseline
            if (line.find('model name') is not -1):
                line = nameline
            lines.append(line)
            # print line
        return lines

    def spawnModel(self,pos,angle,size,name):
        ##pos is the x,y,z postion of the model, angel is the yaw of the model, angle is from x cordinate
        ## size is the length of x,y,z,
        ## name is the name of model
        lines = self.spawnXML(pos,angle,size,name)
        # self.writeXML(self.sdfFileLoc,lines)

        initial_pose = Pose()
        initial_pose.position.x = 0
        initial_pose.position.y = 0
        initial_pose.position.z = 0
        # f = open(self.sdfFileLoc, 'r')
        sdf = StringIO(''.join(lines)).read()

        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(self.name, sdf, " ", initial_pose, "world")

        # f.close()
        return

if __name__ == '__main__':
    rospy.init_node('insert_wall')

    wallLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/wall.sdf'

    size = Point()
    size.x = 10
    size.y = 0.25
    size.z = 1.5
    nameBefore = "my_wall"
    pos = Point()
    pos.x = 5
    pos.y = 5
    pos.z = (size.z) / 2
    angle = 1.57

    # wall_model.spawnXML(pos,angle,size,name)
    name = nameBefore + '1'
    pos.x = 0
    pos.y = 5
    angle = 1.57
    wall_model1 = WallModel(wallLoc)
    wall_model1.spawnModel(pos, angle, size, name)

    name = nameBefore + '2'
    pos.x = 5
    pos.y = 10
    angle = 0
    wall_model2 = WallModel(wallLoc)
    wall_model2.spawnModel(pos, angle, size, name)

    name = nameBefore + '3'
    pos.x = 10
    pos.y = 5
    angle = 1.57
    wall_model3 = WallModel(wallLoc)
    wall_model3.spawnModel(pos, angle, size, name)

    time.sleep(10)

    deleteModel(wall_model1.name)
    deleteModel(wall_model2.name)
    deleteModel(wall_model3.name)



