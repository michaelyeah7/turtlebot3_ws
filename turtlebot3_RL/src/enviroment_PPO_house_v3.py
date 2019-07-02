#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
import numpy as np
import math
from math import pi
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from respawnGoal import Respawn
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import time

class Env():
    def __init__(self, action_size):
        self.goal_x = 0
        self.goal_y = 0
        self.heading = 0
        self.lastDistance = 0#QinjieLin
        self.currentDistance = 0
        self.action_size = action_size
        self.initGoal = True
        self.get_goalbox = False
        self.position = Pose()
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.sub_odom = rospy.Subscriber('odom', Odometry, self.getOdometry)
        # self.reset_proxy = rospy.ServiceProxy('gazebo/reset_simulation', Empty)
        # self.unpause_proxy = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        # self.pause_proxy = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.respawn_goal = Respawn()
        self.resetX = rospy.get_param('/x_pos',0.0)
        self.resetY = rospy.get_param('/y_pos',0.0)
        self.resetZ = rospy.get_param('/z_pos',0.0)
        self.resetYaw = rospy.get_param('/yam_angle',0.0)
        self.resetQua = quaternion_from_euler(0.0,0.0,self.resetYaw)
        self.startTime = time.time()
        self.endTime = 0
        self.lin_weight =  rospy.get_param('/lin_weight',0.0)
        self.obs_weight =  rospy.get_param('/obs_weight',0.0)
        self.ori_weight =  rospy.get_param('/ori_weight',0.0)



    def getGoalDistace(self):
        goal_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)

        return goal_distance

    def getOdometry(self, odom):
        self.position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)

        heading = goal_angle - yaw
        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        self.heading = round(heading, 2)

    def getState(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False
        full_scan_range = []
        range_dim = 60
        num_dim = int(len(scan.ranges) / range_dim)

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                full_scan_range.append(3.5)
            elif np.isnan(scan.ranges[i]):
                full_scan_range.append(0)
            else:
                full_scan_range.append(scan.ranges[i])

        for i in range(num_dim):
            begin = i * range_dim
            end = (i + 1) * range_dim - 1
            if (end >= len(scan.ranges)):
                end = -1
            scan_range.append(min(full_scan_range[begin:end]))

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = np.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = True

        self.lastDistance = self.currentDistance#qinjielin
        current_distance = round(math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2)
        self.currentDistance = current_distance#qinjielin
        if current_distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, current_distance, obstacle_min_range, obstacle_angle], done
        # return [heading, current_distance, obstacle_min_range, obstacle_angle], done


    def setReward(self, state, done, action):
        yaw_reward = []
        current_distance = state[-3]
        heading = state[-4]
        obs_min_range = state[-2]

        forward_distance = self.lastDistance - current_distance
        distance_reward = forward_distance*10
        if(forward_distance <= 0):
            distance_reward -= 0.5

        rot_reward = 0
        if(math.fabs(action[0])>0.7):
            rot_reward = -0.5 * math.fabs(action[0])

        ori_reward = 0
        ori_cos = math.cos(heading)
        if((heading < 1.0) and (forward_distance>0)):
            ori_reward = ori_cos * 0.2 * self.ori_weight

        lin_reward = 0
        if((action[1]>=0.2) and (forward_distance>0)):
            lin_reward = action[1] * 0.5 * self.lin_weight

        obs_reward = 0
        if ((obs_min_range > 0.13) and (obs_min_range < 0.25)):
            obs_reward -= (0.25 - obs_min_range) * 0.3 * self.obs_weight

        reward = distance_reward + rot_reward + lin_reward + ori_reward + obs_reward

        print("state:",np.round(state,2))
        print("choose action:",np.round(action,3))
        print("move:",round(distance_reward,3),"rot:",round(rot_reward,3), "ori:",round(ori_reward,2),"obs:",round(obs_reward,2),"total:",round(reward,3))


        if done:
            rospy.loginfo("Collision!!")
            reward = -10#-15
            self.pub_cmd_vel.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 10#15
            self.pub_cmd_vel.publish(Twist())
            self.goal_x, self.goal_y = self.respawn_goal.getPosition(True, delete=True)
            self.goal_distance = self.getGoalDistace()
            self.currentDistance = self.goal_distance#qinjielin
            self.lastDistance = self.currentDistance
            self.get_goalbox = False
            self.endTime = time.time()
            exeTime = self.endTime - self.startTime
            self.startTime = time.time()
            print("exetime:",exeTime)

        return reward

    def step(self, action):
        max_angular_vel = 2.0
        max_linear_vel = 0.3
        ang_vel = ((action[0]/2))*max_angular_vel#0.15
        linear_vel = ((action[1]+2)/4)*max_linear_vel#0.15
        # ang_vel=action[0]
        # linear_vel = action[1]

        if(math.fabs(ang_vel)<0.5):
            ang_vel = 0
        # if(linear_vel<0.15):
        #     linear_vel = 0
        vel_cmd = Twist()
        vel_cmd.linear.x = linear_vel # change when learning
        vel_cmd.angular.z = ang_vel  # change when learing
        tansAction =[ang_vel,linear_vel]
        self.pub_cmd_vel.publish(vel_cmd)
        rospy.sleep(0.5)#qinjielin

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.getState(data)
        reward = self.setReward(state, done, tansAction)

        return np.asarray(state), reward, done

    def reset(self):
        # print("waiting service")
        # rospy.wait_for_service('gazebo/reset_simulation')
        # # print("got service")
        # try:
        #     self.reset_proxy()
        # except (rospy.ServiceException) as e:
        #     print("gazebo/reset_simulation service call failed")
        self.reset_turtlebot3()

        # print("getting scan")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                # print("getting scan pass")
                pass

        # print("got scan")

        if self.initGoal:
            self.goal_x, self.goal_y = self.respawn_goal.getPosition()
            self.initGoal = False
        
        # print("init goal done ")

        self.goal_distance = self.getGoalDistace()
        self.currentDistance = self.goal_distance#qinjielin
        self.lastDistance = self.currentDistance
        self.startTime = time.time()
        state, done = self.getState(data)

        # print("env rest done")

        return np.asarray(state)

    def reset_turtlebot3(self):
        turState = ModelState()
        turState.model_name = "turtlebot3_burger"
        turState.pose.position.x = self.resetX
        turState.pose.position.y = self.resetY
        turState.pose.position.z = self.resetZ
        turState.pose.orientation.x = self.resetQua[0]
        turState.pose.orientation.y = self.resetQua[1]
        turState.pose.orientation.z = self.resetQua[2]
        turState.pose.orientation.w = self.resetQua[3]
        rospy.wait_for_service('/gazebo/set_model_state')
        set_model_prox = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_prox(turState)
        return
