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

class cadrl_control_host():
    def __init__(self):
        self.node_name = rospy.get_name()
        self.radius = 0.5
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


        #subscribers
        self.host_odom_sub = rospy.Subscriber('host_tb3/odom', Odometry, self.cb_host_tb3)
        self.agent_pos_sub =  rospy.Subscriber('viz', Odometry, self.cb_host_tb3)
        self.tb3_0_odom_sub = rospy.Subscriber('tb3_0/odom', Odometry, self.cb_tb3_0)
        self.tb3_0_odom_sub = rospy.Subscriber('tb3_1/odom', Odometry, self.cb_tb3_1)
        self.tb3_0_odom_sub = rospy.Subscriber('tb3_2/odom', Odometry, self.cb_tb3_2)
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

    def cb_tb3_0(self,msg):
        self.tb3_0_pos_x = msg.pose.pose.position.x
        self.tb3_0_pos_y = msg.pose.pose.position.y
        self.tb3_0_goal_x = self.tb3_0_pos_x - 1.0
        self.tb3_0_goal_y = self.tb3_0_pos_y

    def cb_tb3_1(self,msg):
        self.tb3_1_pos_x = msg.pose.pose.position.x
        self.tb3_1_pos_y = msg.pose.pose.position.y
        self.tb3_1_goal_x = self.tb3_0_pos_x - 1.0
        self.tb3_1_goal_y = self.tb3_0_pos_y

    def cb_tb3_2(self,msg):
        self.tb3_2_pos_x = msg.pose.pose.position.x
        self.tb3_2_pos_y = msg.pose.pose.position.y
        self.tb3_2_goal_x = self.tb3_0_pos_x - 1.0
        self.tb3_2_goal_y = self.tb3_0_pos_y

    def control(self,event):
        possible_actions = network.Actions()
        num_actions = possible_actions.num_actions
        nn = network.NetworkVP_rnn(network.Config.DEVICE, 'network', num_actions)
        nn.simple_load('/home/yeguo/turtlebot3_ws/src/cadrl_ros/checkpoints/network_01900000')

        host_agent = agent.Agent(self.host_pos_x, self.host_pos_y, self.host_goal_x, self.host_goal_y, self.radius, self.pref_speed, self.host_heading_angle, 0)
        host_agent.vel_global_frame = np.array([self.host_vx, self.host_vy])
        other_agents = []
        #other_agents.append(agent.Agent(self.tb3_0_pos_x, self.tb3_0_pos_y, self.tb3_0_goal_x, self.tb3_0_goal_y, self.radius, self.pref_speed, self.other_agents_heading_angle, 1))
        other_agents.append(agent.Agent(self.tb3_1_pos_x, self.tb3_1_pos_y, self.tb3_1_goal_x, self.tb3_1_goal_y, self.radius, self.pref_speed, self.other_agents_heading_angle, 2))
        #other_agents.append(agent.Agent(self.tb3_2_pos_x, self.tb3_2_pos_y, self.tb3_2_goal_x, self.tb3_2_goal_y, self.radius, self.pref_speed, self.other_agents_heading_angle, 3))
        obs = host_agent.observe(other_agents)[1:]
        obs = np.expand_dims(obs, axis=0)

        predictions = nn.predict_p(obs, None)[0]
        raw_action = possible_actions.actions[np.argmax(predictions)]
        action = np.array(
            [host_agent.pref_speed * raw_action[0], util.wrap(raw_action[1] + host_agent.heading_global_frame)])
        print "action:", action
        command = Twist()
        command.linear.x = action[0]
        yaw_error = action[1] - self.psi
        if yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        if yaw_error < -np.pi:
            yaw_error += 2 * np.pi
        command.angular.z = 2 * yaw_error
        print "host command:", command
        self.host_cmd_vel_pub.publish(command)
        #control agents
        tb3_0_command = Twist()
        tb3_0_command.linear.x = 0.05
        self.tb3_0_cmd_vel_pub.publish(tb3_0_command)
        tb3_1_command = Twist()
        tb3_1_command.linear.x = 0.05
        self.tb3_1_cmd_vel_pub.publish(tb3_1_command)
        tb3_2_command = Twist()
        tb3_2_command.linear.x = 0.05
        self.tb3_2_cmd_vel_pub.publish(tb3_2_command)





def run():
    print 'hello world from host_control.py'
    rospy.init_node('host_control', anonymous=False)
    host_control = cadrl_control_host()
    rospy.spin()

if __name__ == '__main__':
    run()