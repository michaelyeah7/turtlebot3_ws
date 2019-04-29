#!/usr/bin/env python
import rospy
from basics.msg import Complex
from random import random

def msg_pub():
    rospy.init_node('message_publisher')
    pub = rospy.Publisher('complex',Complex,queue_size=10)
    rate = rospy.Rate(1)
    count = 0
    while not rospy.is_shutdown():

     msg = Complex()
     # Alternatively we can use keyword arguments to initialize the message class. E.g.:
     # 'msg = Complex(real=2.3, imaginary=4.6)' or 'msg = Complex(real=1.2)' or
     # even 'msg = Complex(2.3, 4.6)'
     msg.real = random()
     msg.imaginary = random()
     rospy.loginfo("Complex number %d is %f + %fi", count, msg.real, msg.imaginary)

     pub.publish(msg)
     count +=1
     rate.sleep()

if __name__ == '__main__':
    try:
        msg_pub()
    except rospy.ROSInterruptException:
        pass
