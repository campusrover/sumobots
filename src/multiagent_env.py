import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils, spaces
#from gym_gazebo.spaces import multi_discrete
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

class MultiAgentGazeboEnv(gazebo_env.GazeboEnv):

    def __init__(self, launchfile):
        # Launch the simulation with the given launchfile name
        # Launch file must include the following line:
        #       <rosparam ns="/namespaces" param="$(arg robot_name)" subst_value="true" > $(arg robot_name) </rosparam>
        gazebo_env.GazeboEnv.__init__(self, launchfile)
        # scenario callbacks
        self.set_callbacks()
        self.num_agents = 0
        self.vel_pubs = []
        for i, robot_name in rospy.get_param('/namespaces'):
            self.num_agents += 1
            self.vel_pubs.append(rospy.Publisher('/%s/cmd_vel' % robot_name, Twist, queue_size=1))
        self.linear_speed = 1.8
        self.angular_speed = 1.8
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        # environment parameters
        self.discrete_action_space = True
        # if true, action is a number 0...N, otherwise action is a one-hot N-dimensional vector
        self.discrete_action_input = False
        # configure spaces
        self.action_space = []
        self.observation_space = []
        for agent in range(self.num_agents):
            total_action_space = []
            # physical action space
            if self.discrete_action_space:
                u_action_space = spaces.Discrete(4)
            #else:
                # TODO: implement continuous action space
            total_action_space.append(u_action_space)
            # communication action space
            # if self.discrete_action_space:
            #     c_action_space = spaces.Discrete(world.dim_c)
            # else:
            #     c_action_space = spaces.Box(low=0.0, high=1.0, shape=(world.dim_c,), dtype=np.float32)
            # if not agent.silent:
            #     total_action_space.append(c_action_space)
            # total action space
            # if len(total_action_space) > 1:
            #     # all action spaces are discrete, so simplify to MultiDiscrete action space
            #     if all([isinstance(act_space, spaces.Discrete) for act_space in total_action_space]):
            #         act_space = multi_discrete.MultiDiscrete([[0, act_space.n - 1] for act_space in total_action_space])
            #     else:
            #         act_space = spaces.Tuple(total_action_space)
            #     self.action_space.append(act_space)
            # else:
            self.action_space.append(total_action_space[0])
            # observation space
            obs_dim = len(observation_callback(agent))
            self.observation_space.append(spaces.Box(low=-np.inf, high=+np.inf, shape=(obs_dim,), dtype=np.float32))
            agent.action.c = np.zeros(self.world.dim_c)
        spaces.Discrete(4) #F,B,L,R
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def set_callbacks(reset_callback=None, reward_callback=None, observation_callback=None,
                      info_callback=None, done_callback=None):
        self.reset_callback = reset_callback
        self.reward_callback = reward_callback
        self.observation_callback = observation_callback
        self.info_callback = info_callback
        self.done_callback = done_callback

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action_n):
        obs_n = []
        reward_n = []
        done_n = []
        info_n = {'n': []}

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        for i, action in enumerate(action_n):
            self._set_action(action, i)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # record observation for each agent
        for i, _ in enumerate(action_n):
            obs_n.append(self._get_obs(i))
            reward_n.append(self._get_reward(i))
            done_n.append(self._get_done(i))
            info_n['n'].append(self._get_info(i))

        return obs_n, reward_n, done_n, info_n

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        
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
        # process action
        # if isinstance(action_space, MultiDiscrete):
        #     act = []
        #     size = action_space.high - action_space.low + 1
        #     index = 0
        #     for s in size:
        #         act.append(action[index:(index+s)])
        #         index += s
        #     action = act
        # else:
        #     action = [action]

        # physical action
        if self.discrete_action_input:
            # process discrete action
            if action[0] == 1: t.linear.x = self.linear_speed
            if action[0] == 2: t.linear.x = -self.linear_speed
            if action[0] == 3: t.angular.z = self.angular_speed
            if action[0] == 4: t.angular.z = -self.angular_speed
        else:
            if self.discrete_action_space:
                t.linear.x = (action[0][0] - action[0][1]) * self.linear_speed
                t.angular.z = (action[0][2] - action[0][3]) * self.angular_speed
            #else:
                # TODO: implement for continuous action space
        
        # if not agent.silent:
        #     # communication action
        #     if self.discrete_action_input:
        #         agent.action.c = np.zeros(self.world.dim_c)
        #         agent.action.c[action[0]] = 1.0
        #     else:
        #         agent.action.c = action[0]
        #     action = action[1:]
        self.vel_pubs[agent].publish(t)
