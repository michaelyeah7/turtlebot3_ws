#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #1


def main():
    rospy.init_node('StopWander')


    cmd_vel_pub0 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    cmd_vel_pub1 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
    cmd_vel_pub2 = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=1)

    StopTwist = Twist()
    StopTwist.linear.x=0.0
    StopTwist.angular.z=0.0

    rospy.sleep(1)

    cmd_vel_pub0.publish(StopTwist)
    cmd_vel_pub1.publish(StopTwist)
    cmd_vel_pub2.publish(StopTwist)

    print("stop turtlebot successfully")



if __name__=='__main__':
    main()
