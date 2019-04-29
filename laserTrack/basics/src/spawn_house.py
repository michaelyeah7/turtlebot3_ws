#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from cStringIO import StringIO
from gazebo_utils import deleteModel
import time
from furniture_model import FurnitureModel
from wall_model import  WallModel



if __name__ == '__main__':
    rospy.init_node('spawn_house')



    wallLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/wall.sdf'
    trashLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/trash.sdf'
    bookshelfLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/bookshelf/model.sdf'
    cafeTableLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/cafe_table/model.sdf'
    cudeLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/cube_20k/model.sdf'
    tableLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table/model.sdf'
    tableMarbleLoc = '/home/qinjielin/RL_Ws/turtleBot3_ws/src/basics/models/house_element/table_marble/model.sdf'

    size=Point()
    size.x=10
    size.y=0.25
    size.z=1.5
    nameBefore="my_fur"
    pos=Point()
    pos.x=5
    pos.y=5
    pos.z=(size.z)/2
    angle=1.57

    # wall_model.spawnXML(pos,angle,size,name)
    name=nameBefore+'1'
    pos.x=0
    pos.y=5
    angle=1.57
    wall_model1=WallModel(wallLoc)
    wall_model1.spawnModel(pos,angle,size,name)

    name=nameBefore+'2'
    pos.x=5
    pos.y=10
    angle=0
    wall_model2=WallModel(wallLoc)
    wall_model2.spawnModel(pos,angle,size,name)

    name=nameBefore+'3'
    pos.x=10
    pos.y=5
    angle=1.57
    wall_model3=WallModel(wallLoc)
    wall_model3.spawnModel(pos,angle,size,name)

    name=nameBefore+'4'
    pos.x=5
    pos.y=2
    angle=0
    fm4=FurnitureModel(tableMarbleLoc)
    fm4.spawnModel(pos,angle,name)

    name=nameBefore+'5'
    pos.x=5
    pos.y=6
    angle=0
    fm5=FurnitureModel(trashLoc)
    fm5.spawnModel(pos,angle,name)

    name=nameBefore+'6'
    pos.x=5
    pos.y=8
    angle=0
    fm6=FurnitureModel(bookshelfLoc)
    fm6.spawnModel(pos,angle,name)

    time.sleep(10)



    deleteModel(wall_model1.name)
    deleteModel(wall_model2.name)
    deleteModel(wall_model3.name)
    deleteModel(fm4.name)
    deleteModel(fm5.name)
    deleteModel(fm6.name)