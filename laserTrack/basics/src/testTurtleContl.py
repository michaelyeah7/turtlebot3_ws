#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan #1

class Wandering(object):
    def __init__(self,cmd_topic,scan_topic):
        ''' Initialize parameters'''
        rospy.Timer(rospy.Duration(0.5), self.my_callback)
        self.cmd_vel_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        self.twist = Twist()
        self.state_change_time = rospy.Time.now()  #2
        self.driving_forward = True #2
        self.scan_sub = rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        self.lastState = 'Moving'
        self.nowState = 'Moving'
        self.lastTime = rospy.Time.now()
        return

    def my_callback(self,event):
        if(self.nowState != self.lastState):
            self.lastState = self.nowState
            rospy.loginfo("State has Changed after %f seconds"%self.delaTime)
        else:
            #rospy.loginfo("State is still %s", self.nowState)
            rospy.loginfo("State is still %s"%self.nowState)


    def scan_callback(self,msg):
        # convert any NaN values to laser max range:
        ranges = [x if not math.isnan(x) else msg.range_max for x in msg.ranges]
        range_ahead = min(ranges) #3
        rospy.loginfo("range ahead: %0.1f" % range_ahead)
        if self.driving_forward: #4
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0
            rospy.loginfo('Moving')
            #self.nowState = 'Moving'
            if range_ahead < 0.15 or rospy.Time.now() > self.state_change_time: #5
                self.driving_forward = False
                self.state_change_time = rospy.Time.now() + rospy.Duration(5) #6
                self.delaTime=rospy.Time.now().to_sec() - self.lastTime.to_sec()
                self.lastTime = rospy.Time.now()
                self.nowState = 'Spinning'
        else:
            self.twist.angular.z = 0.15 #7
            self.twist.linear.x = 0
            rospy.loginfo('Spinning')
            #self.nowState = 'Spinning'
            if rospy.Time.now() > self.state_change_time: #8
                self.driving_forward = True
                rospy.loginfo('Switch')
                self.delaTime=rospy.Time.now().to_sec()  - self.lastTime.to_sec()
                self.lastTime = rospy.Time.now()
                self.nowState = 'Moving'
                self.state_change_time = rospy.Time.now() + rospy.Duration(10) #9
        self.cmd_vel_pub.publish(self.twist) #10
        return


def main():
    rospy.init_node('wander')
    #wandering1 = Wandering('/tb3_0/cmd_vel','/tb3_0/scan')
    wandering2 = Wandering('/tb3_1/cmd_vel','/tb3_1/scan')
    wandering3 = Wandering('/tb3_2/cmd_vel','/tb3_2/scan')

    try:
        rospy.spin()
    except KeyboardInterrupt,rospy.ROSInterruptException:
        print("Shutting down")

if __name__=='__main__':
    main()