#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
from math import pi, atan2
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg
from nav_msgs.msg import Odometry

class SumoScenario():
    def __init__(self, num_robots=2):
        self.num_robots = num_robots
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.obs = []
        # self.odom_subs = []
        for i in range(self.num_robots):
            self.obs.append([0,0])
        #     robot_name = 'robot%d' % (i + 1)
        #     self.odom_subs.append(rospy.Subscriber('%s/odom' % robot_name, Odometry, odom_cb, i))

    def reward(self, robot_index):
        # rew = 0
        # for i in range(self.num_robots):
        #     if i != robot_index and self.boundary(i):
        #         rew += 5
        # rew -= 0.5 * (rospy.get_time() - self.init_time)
        rew = 0
        if boundary(robot_index):
            rew -= 10
        else:
            rew -= self.obs[robot_index][0]
        return rew

    # agent penalty for exiting the sumo arena
    def boundary(self, robot_index):
        if self.obs[robot_index][0] > 1.4:
            return True
        return False

    def observation(self, robot_index):
        self_tf = self.tfBuffer.lookup_transform('robot%d' % (robot_index + 1), 'world', rospy.Time())
        dist = math.sqrt(self_tf.transform.translation.x ** 2 + self_tf.transform.translation.y ** 2)
        angle = math.atan2(self_tf.transform.translation.y, self_tf.transform.translation.x)
        center = [dist, angle]
        other = []
        for i in range(self.num_robots):
            if i != robot_index:
                trans = self.tfBuffer.lookup_transform('robot%d' % (robot_index + 1), 'robot%d' (i + 1), rospy.Time())
                dist = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
                angle = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
                other.extend([dist, angle])
        self.obs[robot_index] = np.concatenate(center + other)
        return self.obs[robot_index]

    def done(self, robot_index):
        for i in range(self.num_robots):
            if self.boundary(i):
                return True
        return False

    # def odom_cb(msg, robot_index):
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     orientation_q = msg.pose.pose.orientation
    #     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    #     (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    #     self.obs[robot_index] = [x, y, yaw]
