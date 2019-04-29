#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from rotate_turtlebot3.msg import action
def action_pub():
    pub = rospy.Publisher('/host_action', action, queue_size=10)
    rospy.init_node('action_publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
	action_msg = action()
        action_msg.data = [0.1, math.pi/2]
        pub.publish(action_msg)
	print action
        rate.sleep()

if __name__ == '__main__':
    try:
        action_pub()
    except rospy.ROSInterruptException:
        pass
