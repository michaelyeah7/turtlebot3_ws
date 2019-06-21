#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path,Odometry
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from turtlesim.msg import Pose

rospy.init_node('test_nav', anonymous=True)
simplePub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
# pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
rospy.sleep(1)
def pubGoal(x,y):
    pub_msg = MoveBaseActionGoal()
    # pub_msg.header = data.header
    pub_msg.goal.target_pose.header.frame_id = 'map'
    pub_msg.header.frame_id  = 'map'
    # pub_msg.goal.target_pose.header.frame_id = data.poses[i].header.frame_id
    pub_msg.goal.target_pose.pose.position.x = x
    pub_msg.goal.target_pose.pose.position.y = y
    pub_msg.goal.target_pose.pose.position.z = 0
    print(pub_msg)
    pub.publish(pub_msg)

def pubSimpleGoal(x,y):
    pub_msg = PoseStamped()
    # pub_msg.header = data.header
    pub_msg.header.frame_id = 'map'
    pub_msg.header.frame_id  = 'map'
    # pub_msg.goal.target_pose.header.frame_id = data.poses[i].header.frame_id
    pub_msg.pose.position.x = x
    pub_msg.pose.position.y = y
    pub_msg.pose.position.z = 0
    pub_msg.pose.orientation.z = 1
    print(pub_msg)
    simplePub.publish(pub_msg)

# pubGoal(1,1)
pubSimpleGoal(1,1)