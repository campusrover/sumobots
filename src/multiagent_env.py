import rospy
import numpy as np
import random
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_connection import GazeboConnection
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler

'''
---------------------------------------------------------------------------------------------------
Adapted and simplified from OpenAI's Multiagent Particle Environment class for use with a ROS
Gazebo world with a discrete action space. Continuous action space has not been implemented.
Steps through the Gazebo simulation, gathering observations and assigning actions and rewards
for each agent at each time step.

Author: Joseph Pickens, August Soderberg
---------------------------------------------------------------------------------------------------
'''
class MultiAgentGazeboEnv():
    def __init__(self, num_agents, reset_callback=None, reward_callback=None,
                 observation_callback=None, info_callback=None, done_callback=None):
        # scenario callbacks
        self.reset_callback = reset_callback
        self.reward_callback = reward_callback
        self.observation_callback = observation_callback
        self.info_callback = info_callback
        self.done_callback = done_callback

        self.num_agents = num_agents
        self.vel_pubs = []
        for i in range(self.num_agents):
            # robot namespaces are assumed to be 'robot1', 'robot2', ...
            self.vel_pubs.append(rospy.Publisher('/robot%d/cmd_vel' % (i+1), Twist, queue_size=1))
        
        # TODO: speed definition should be specific to the scenario from which callbacks are
        # defined, rather than be defined here in the general multiagent environment class.
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
        state_msg = ModelState()
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        for i in range(self.num_agents):
            state_msg.model_name = 'Robot%d' % (i + 1)
            state_msg.pose.position.x = random.uniform(-1.2, 1.2)
            state_msg.pose.position.y = random.uniform(-1.2, 1.2)
            state_msg.pose.position.z = 0.35
            quaternion = quaternion_from_euler(0, 0, random.uniform(0, 6.28))
            state_msg.pose.orientation.x = quaternion[0]
            state_msg.pose.orientation.y = quaternion[1]
            state_msg.pose.orientation.z = quaternion[2]
            state_msg.pose.orientation.w = quaternion[3]
            rospy.wait_for_service('/gazebo/set_model_state')
            try:
                set_state(state_msg)
            except rospy.ServiceException, e:
                print('Service call failed: %s' % e)
        self.gazebo.unpauseSim()
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
    def _get_done(self, agent):
        if self.done_callback is None:
            return False
        return self.done_callback(agent)

    # get reward for a particular agent
    def _get_reward(self, agent):
        if self.reward_callback is None:
            return 0.0
        return self.reward_callback(agent)

    # set env discrete action for a particular agent
    # action must be a list of 4 binary variables: [forward, backward, left, right]
    def _set_action(self, action, agent):
        t = Twist()
        t.linear.x = (action[0] - action[1]) * self.linear_speed
        t.angular.z = (action[2] - action[3]) * self.angular_speed
        self.vel_pubs[agent].publish(t)
