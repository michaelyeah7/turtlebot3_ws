#!/usr/bin/env python
import sys
sys.path.insert(0, '/home/yeguo/turtlebot3_ws/src/cadrl_ros/scripts')
import agent
import network
import util
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray
from obstacle_detector.msg import Obstacles
import math
from termcolor import colored
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32, ColorRGBA, Int32
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Point

class cadrl_control_host():
    def __init__(self):
        self.node_name = rospy.get_name()
        self.world_frame_id = 'host_tb3/odom'
        self.num_poses = 0
        self.obstacles_count = 0
        self.radius = 0.3
        self.pref_speed = 0.05
        self.host_heading_angle = 0.0
        self.other_agents_heading_angle = -3.14
        #host
        self.host_goal_x = 3.0
        self.host_goal_y = 0.0
        self.host_pos_x = 0.0
        self.host_pos_y = 0.0
        self.host_heading_angle = 0.0
        self.host_vx = 0.0
        self.host_vy = 0.0
        self.psi = 0.0
        self.host_cmd_vel = Twist()
        #other agents' pos and vel
        self.tb3_0_pos_x = 0.0
        self.tb3_0_pos_y = 0.0
        self.tb3_0_goal_x = 0.0
        self.tb3_0_goal_y = 0.0
        self.tb3_1_pos_x = 0.0
        self.tb3_1_pos_y = 0.0
        self.tb3_1_goal_x = 0.0
        self.tb3_1_goal_y = 0.0
        self.tb3_2_pos_x = 0.0
        self.tb3_2_pos_y = 0.0
        self.tb3_2_goal_x = 0.0
        self.tb3_2_goal_y = 0.0
        self.tb3_agents = {0: {'pos_x': 0.0, 'pos_y': 0.0, 'goal_x': 0.0, 'goal_y': 0.0, 'radius':self.radius, 'pref_speed':self.pref_speed, 'agents_heading_angle':self.other_agents_heading_angle}}

        #subscribers
        self.host_odom_sub = rospy.Subscriber('host_tb3/odom', Odometry, self.cb_host_tb3)
        #self.agent_pos_sub = rospy.Subscriber('viz', MarkerArray, self.cb_handle_obs)
        self.agent_pos_sub = rospy.Subscriber('tracked_obstacles', Obstacles, self.cb_handle_obs)

        #publisher
        self.host_cmd_vel_pub = rospy.Publisher('host_tb3/cmd_vel',Twist,queue_size=1)
        self.tb3_0_cmd_vel_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=1)
        self.tb3_1_cmd_vel_pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=1)
        self.tb3_2_cmd_vel_pub = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=1)
        self.pub_pose_marker = rospy.Publisher('~pose_marker', Marker, queue_size=1)

        self.nn_timer = rospy.Timer(rospy.Duration(0.1), self.control)

    def cb_host_tb3(self,msg):
        self.host_pos_x = msg.pose.pose.position.x
        self.host_pos_y = msg.pose.pose.position.y
        #self.host_goal_x = self.host_pos_x + 1.0
        #self.host_goal_y = self.host_pos_y
        self.host_vx = msg.twist.twist.linear.x
        self.host_vy = msg.twist.twist.linear.y
        q = msg.pose.pose.orientation
        self.psi = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.visualize_pose(msg.pose.pose.position, msg.pose.pose.orientation)
        self.num_poses += 1
        #self.host_heading_angle = self.quat_to_euler(msg)

    def cb_handle_obs(self,msg):
        self.obstacles_count = len(msg.circles)
        #print "agents_count:", self.obstacles_count
        for i in range(self.obstacles_count):
            x_pos = msg.circles[i].center.x
            y_pos = msg.circles[i].center.y
            x_vel = msg.circles[i].velocity.x
            y_vel = msg.circles[i].velocity.y
            #print ("rel x %f; rel y %f" % (x,y))
            x_global = self.host_pos_x + x_pos * math.cos(self.psi) - y_pos * math.sin(self.psi)
            y_global = self.host_pos_y + x_pos * math.sin(self.psi) + y_pos * math.cos(self.psi)
            x_vel_global = x_vel * math.cos(self.psi) - y_vel * math.sin(self.psi)
            y_vel_global = x_vel * math.cos(self.psi) - y_vel * math.sin(self.psi)
            self.tb3_agents[i] = {}
            self.tb3_agents[i]['pos_x'] = x_global
            self.tb3_agents[i]['pos_y'] = y_global
            self.tb3_agents[i]['goal_x'] = self.tb3_agents[i]['pos_x'] + x_vel_global
            self.tb3_agents[i]['goal_y'] = self.tb3_agents[i]['pos_y'] + y_vel_global
            self.tb3_agents[i]['radius'] = msg.circles[i - 1].true_radius
            self.tb3_agents[i]['pref_speed'] = np.linalg.norm(np.array([x_vel_global, y_vel_global]))
            self.tb3_agents[i]['other_agents_heading_angle'] = np.arctan2(y_vel_global, x_vel_global)
            #print("current vel of agent %f, vel:%f,%f" % (i, x_vel_global, y_vel_global))
            #print("current radius %f" % (self.tb3_agents[i]['radius']))

    def control(self,event):
        possible_actions = network.Actions()
        print "something 1"
        num_actions = possible_actions.num_actions
        nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
        nn.simple_load('/home/yeguo/turtlebot3_ws/src/cadrl_ros/checkpoints/network_01900000')
        print "something here"
        host_agent = agent.Agent(self.host_pos_x, self.host_pos_y, self.host_goal_x, self.host_goal_y, self.radius, self.pref_speed, self.host_heading_angle, 0)
        host_agent.vel_global_frame = np.array([self.host_vx, self.host_vy])
        other_agents = []
        for i in range(len(self.tb3_agents)):
            other_agents.append(agent.Agent(self.tb3_agents[i]['pos_x'], self.tb3_agents[i]['pos_y'], self.tb3_agents[i]['goal_x'], self.tb3_agents[i]['goal_y'], self.tb3_agents[i]['radius'], self.tb3_agents[i]['pref_speed'], self.tb3_agents[i]['other_agents_heading_angle'], i+1))

        obs = host_agent.observe(other_agents)[1:]
        obs = np.expand_dims(obs, axis=0)

        predictions = nn.predict_p(obs, None)[0]
        raw_action = possible_actions.actions[np.argmax(predictions)]
        action = np.array(
            [host_agent.pref_speed * raw_action[0], util.wrap(raw_action[1] + host_agent.heading_global_frame)])
        print colored('action:','red'), action
        command = Twist()
        command.linear.x = action[0]
        yaw_error = action[1] - self.psi
        print "self.psi:", self.psi
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        if yaw_error < -np.pi:
            yaw_error += 2 * np.pi
        print "yaw_error:", yaw_error
        command.angular.z = 2 * yaw_error
        print "host command:", command
        self.host_cmd_vel_pub.publish(command)
        #control agents
        tb3_0_command = Twist()
        tb3_0_command.linear.x = self.pref_speed
        self.tb3_0_cmd_vel_pub.publish(tb3_0_command)
        tb3_1_command = Twist()
        tb3_1_command.linear.x = self.pref_speed
        self.tb3_1_cmd_vel_pub.publish(tb3_1_command)
        tb3_2_command = Twist()
        tb3_2_command.linear.x = self.pref_speed
        self.tb3_2_cmd_vel_pub.publish(tb3_2_command)

    def visualize_pose(self,pos,orientation):
        # Yellow Box for Vehicle
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.world_frame_id
        marker.ns = 'agent'
        marker.id = 0
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = pos
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.7,y=0.42,z=1)
        marker.color = ColorRGBA(r=1.0,g=1.0,a=1.0)
        marker.lifetime = rospy.Duration(1.0)
        self.pub_pose_marker.publish(marker)

        # Red track for trajectory over time
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.world_frame_id
        marker.ns = 'agent'
        marker.id = self.num_poses
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position = pos
        marker.pose.orientation = orientation
        marker.scale = Vector3(x=0.2,y=0.2,z=0.2)
        marker.color = ColorRGBA(r=1.0,a=1.0)
        marker.lifetime = rospy.Duration(50.0)
        self.pub_pose_marker.publish(marker)



def run():
    print 'hello world from host_control.py'
    rospy.init_node('host_control', anonymous=False)
    host_control = cadrl_control_host()
    rospy.spin()

if __name__ == '__main__':
    run()