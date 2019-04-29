#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def odom_cb(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rospy.loginfo("x: {}, y: {}".format(x,y))

def main():
    rospy.init_node('location_monitor')
    rospy.Subscriber("odom", Odometry, odom_cb)
    rospy.spin()

if __name__ == '__main__':
    main()
