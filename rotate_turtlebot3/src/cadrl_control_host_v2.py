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

class cadrl_control_host():
    def __init__(self):
        self.node_name = rospy.get_name()
        self.obstacles_count = 0
        self.radius = 0.3
        self.pref_speed = 0.2
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
        self.tb3_agents = {1: {'pos_x': 0.0, 'pos_y': 0.0, 'goal_x': 0.0, 'goal_y': 0.0, 'radius':self.radius, 'pref_speed':self.pref_speed, 'agents_heading_angle':self.other_agents_heading_angle}}

        #subscribers
        self.host_odom_sub = rospy.Subscriber('host_tb3/odom', Odometry, self.cb_host_tb3)
        #self.agent_pos_sub = rospy.Subscriber('viz', MarkerArray, self.cb_handle_obs)
        self.agent_pos_sub = rospy.Subscriber('raw_obstacles', Obstacles, self.cb_handle_obs)

        #publisher
        self.host_cmd_vel_pub = rospy.Publisher('host_tb3/cmd_vel',Twist,queue_size=1)
        self.tb3_0_cmd_vel_pub = rospy.Publisher('tb3_0/cmd_vel', Twist, queue_size=1)
        self.tb3_1_cmd_vel_pub = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=1)
        self.tb3_2_cmd_vel_pub = rospy.Publisher('tb3_2/cmd_vel', Twist, queue_size=1)

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
        #self.host_heading_angle = self.quat_to_euler(msg)

    def cb_handle_obs(self,msg):
        self.obstacles_count = len(msg.circles)
        print "agents_count:", self.obstacles_count
        for i in range(1,self.obstacles_count+1):
            x = msg.circles[i - 1].center.x
            y = msg.circles[i - 1].center.y
            #print ("rel x %f; rel y %f" % (x,y))
            x_global = self.host_pos_x + x * math.cos(self.psi) - y * math.sin(self.psi)
            y_global = self.host_pos_y + x * math.sin(self.psi) + y * math.cos(self.psi)
            self.tb3_agents[i] = {}
            self.tb3_agents[i]['pos_x'] = x_global
            self.tb3_agents[i]['pos_y'] = y_global
            self.tb3_agents[i]['goal_x'] = self.tb3_agents[i]['pos_x'] - 1.0
            self.tb3_agents[i]['goal_y'] = self.tb3_agents[i]['pos_y']
            print("current pos of agent %f, pos:%f,%f" % (i, x_global, y_global))

    def control(self,event):
        possible_actions = network.Actions()
        num_actions = possible_actions.num_actions
        nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
        nn.simple_load('/home/yeguo/turtlebot3_ws/src/cadrl_ros/checkpoints/network_01900000')

        host_agent = agent.Agent(self.host_pos_x, self.host_pos_y, self.host_goal_x, self.host_goal_y, self.radius, self.pref_speed, self.host_heading_angle, 0)
        host_agent.vel_global_frame = np.array([self.host_vx, self.host_vy])
        other_agents = []
        for i in range(1,self.obstacles_count+1):
            other_agents.append(agent.Agent(self.tb3_agents[i]['pos_x'], self.tb3_agents[i]['pos_y'], self.tb3_agents[i]['goal_x'], self.tb3_agents[i]['goal_y'], self.radius, self.pref_speed, self.other_agents_heading_angle, i))

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
        tb3_0_command.linear.x = 0.2
        self.tb3_0_cmd_vel_pub.publish(tb3_0_command)
        tb3_1_command = Twist()
        tb3_1_command.linear.x = 0.2
        self.tb3_1_cmd_vel_pub.publish(tb3_1_command)
        tb3_2_command = Twist()
        tb3_2_command.linear.x = 0.2
        self.tb3_2_cmd_vel_pub.publish(tb3_2_command)





def run():
    print 'hello world from host_control.py'
    rospy.init_node('host_control', anonymous=False)
    host_control = cadrl_control_host()
    rospy.spin()

if __name__ == '__main__':
    run()