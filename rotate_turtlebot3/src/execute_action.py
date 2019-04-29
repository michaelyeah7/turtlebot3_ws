#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import std_msgs
import math
from rotate_turtlebot3.msg import action

roll = pitch = yaw = 0.0
kP = 0.5
host_action_vel=host_action_direction=0

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print yaw

def get_action (msg):
    global host_action_vel,host_action_direction
    host_action_vel = msg.data[0]
    host_action_direction = msg.data[1]
    #print host_action_direction

rospy.init_node('execute_action')

odom_sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
action_sub = rospy.Subscriber ('/host_action', action, get_action)
pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
r = rospy.Rate(50)
command = Twist()

while not rospy.is_shutdown():    
    if abs(host_action_direction - yaw)<0.1:
	command.angular.z = 0
	command.linear.x = host_action_vel
    else:
        command.angular.z = kP * (host_action_direction - yaw)
        command.linear.x = 0
    pub.publish(command)
    print("Target={} Current:{}".format(host_action_direction,yaw))
    r.sleep()
