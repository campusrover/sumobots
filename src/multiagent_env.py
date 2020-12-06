import rospy
import numpy as np
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_connection import GazeboConnection

class MultiAgentGazeboEnv():

    def __init__(self, reset_callback=None,
                       reward_callback=None,
                       observation_callback=None,
                       info_callback=None,
                       done_callback=None):
        # scenario callbacks
        self.reset_callback = reset_callback
        self.reward_callback = reward_callback
        self.observation_callback = observation_callback
        self.info_callback = info_callback
        self.done_callback = done_callback
        self.num_agents = 2
        self.vel_pubs = []
        for i in range(self.num_agents):
            self.vel_pubs.append(rospy.Publisher('/robot%d/cmd_vel' % (i+1), Twist, queue_size=1))
        self.linear_speed = 2.0
        self.angular_speed = 2.0
        self.gazebo = GazeboConnection(False, 'WORLD')

    def step(self, action_n):
        obs_n = []
        reward_n = []
        done_n = []
        info_n = {'n': []}

        self.gazebo.unpauseSim()
        for i, action in enumerate(action_n):
            self._set_action(action, i)
        self.gazebo.pauseSim()

        # record observation, etc. for each agent
        for i, _ in enumerate(action_n):
            obs_n.append(self._get_obs(i))
            reward_n.append(self._get_reward(i))
            done_n.append(self._get_done(i))
            info_n['n'].append(self._get_info(i))

        return obs_n, reward_n, done_n, info_n

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        self.gazebo.resetSim()
        self.gazebo.pauseSim()
        obs_n = []
        for i in range(self.num_agents):
            obs_n.append(self._get_obs(i))
        return obs_n

    # get info used for benchmarking
    def _get_info(self, agent):
        if self.info_callback is None:
            return {}
        return self.info_callback(agent, self.world)

    # get observation for a particular agent
    def _get_obs(self, agent):
        if self.observation_callback is None:
            return np.zeros(0)
        return self.observation_callback(agent)

    # get dones for a particular agent
    # unused right now -- agents are allowed to go beyond the viewing screen
    def _get_done(self, agent):
        if self.done_callback is None:
            return False
        return self.done_callback(agent)

    # get reward for a particular agent
    def _get_reward(self, agent):
        if self.reward_callback is None:
            return 0.0
        return self.reward_callback(agent)

    # set env action for a particular agent
    def _set_action(self, action, agent):
        t = Twist()
        t.linear.x = (action[0] - action[1]) * self.linear_speed
        t.angular.z = (action[2] - action[3]) * self.angular_speed
        self.vel_pubs[agent].publish(t)
