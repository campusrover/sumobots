#!/usr/bin/env python
import rospy, math
import numpy as np
import tf2_ros
from math import pi, atan2
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg
from nav_msgs.msg import Odometry

'''
---------------------------------------------------------------------------------------------------
Defines the reward, observation, and done callbacks for the sumobots game to be passed to an
instance of the MultiAgentGazeboEnv class.

Author: Joseph Pickens, August Soderberg
---------------------------------------------------------------------------------------------------
'''
class SumoScenario():
    def __init__(self, num_robots=2):
        self.rate = rospy.Rate(10.0)
        self.num_robots = num_robots
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
    def reward(self, robot_index):
        rew = 0
        count_of_other_bot = self.num_robots - 1
        curr_obs = self.observation(robot_index)
        for i in range(self.num_robots):
            if i != robot_index:
                rew -= 0.1*curr_obs[2*count_of_other_bot]
                count_of_other_bot -= 1
                if self.boundary(i):
                    rew += 100
        if self.boundary(robot_index):
            rew -= 100
        return rew

    # return true if robot has fallen off the sumo arena platform
    def boundary(self, robot_index):
        while True:
            try:
                self_tf = self.tfBuffer.lookup_transform('world', 'robot%d' % (robot_index + 1), rospy.Time(0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                self.rate.sleep()
        z = self_tf.transform.translation.z
        if z < 0.29:
            return True
        return False

    def observation(self, robot_index):
        # get self transform relative to world
        while True:
            try:
                self_tf = self.tfBuffer.lookup_transform('world', 'robot%d' % (robot_index + 1), rospy.Time(0))
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                self.rate.sleep()
        # define distance and direction of center of arena
        dist = math.sqrt(self_tf.transform.translation.x ** 2 + self_tf.transform.translation.y ** 2)
        angle = math.atan2(self_tf.transform.translation.y, self_tf.transform.translation.x)
        center = [dist, angle]
        # get other robot's transform relative to self
        other = []
        for i in range(self.num_robots):
            if i != robot_index:
                while True:
                    try:
                        trans = self.tfBuffer.lookup_transform('robot%d' % (robot_index + 1), 'robot%d' % (i + 1), rospy.Time(0))
                        break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                        self.rate.sleep()
                        continue
                # define distance and direction of other robot
                dist = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
                angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
                other.extend([dist, angle])
        return center + other

    # done if robot has fallen off the arena platform
    def done(self, robot_index):
        return self.boundary(robot_index)
