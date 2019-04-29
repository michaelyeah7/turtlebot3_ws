#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from basics.msg import LandmarkDistance #1

def distance(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    return math.sqrt(xd*xd + yd*yd)

class LandmarkMonitor(object): #2
    def __init__(self, landmark_pub, landmarks):
        self._landmark_pub = landmark_pub
        self._landmarks = landmarks

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        closest_name = None
        closest_distance = None
        for l_name,l_x, l_y in self._landmarks:
            dist = distance(x, y, l_x, l_y)
            if closest_distance is None or dist < closest_distance:
                closest_name = l_name
                closest_distance = dist
        ld = LandmarkDistance() #3
        ld.name = closest_name
        ld.distance = closest_distance
        self._landmark_pub.publish(ld) #4
        if closest_distance < 0.5: #5
            rospy.loginfo("I'm near the {}".format(closest_name))


def main():
    rospy.init_node('location_monitor_node')
    landmarks = []
    landmarks.append(("Cube", 0.31, -0.99));
    landmarks.append(("Dumpster", 0.11, -2.42));
    landmarks.append(("Cylinder", -1.14, -2.88));
    landmarks.append(("Barrier", -2.59, -0.83));
    landmarks.append(("Bookshelf", -0.09, 0.53));

    landmark_pub = rospy.Publisher("closest_landmark", LandmarkDistance, queue_size=10)  #6
    monitor = LandmarkMonitor(landmark_pub, landmarks) #7
    rospy.Subscriber("odom", Odometry, monitor.callback) #8

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()

