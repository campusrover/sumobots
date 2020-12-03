import rospy
import numpy as np
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        self.num_agents = 0
        self.vel_pubs = []
        for i, robot_name in enumerate(['robot1', 'robot2']):
            self.num_agents += 1
            self.vel_pubs.append(rospy.Publisher('/%s/cmd_vel' % robot_name, Twist, queue_size=1))
        self.linear_speed = 2.0
        self.angular_speed = 2.0
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

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

        for i in range(self.num_agents):
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message('robot%d/odom' % (i + 1), Odometry, timeout=5)
                except:
                    pass   

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # record observation, etc. for each agent
        for i, _ in enumerate(action_n):
            obs_n.append(self._get_obs(i))
            reward_n.append(self._get_reward(i))
            done_n.append(self._get_done(i))
            info_n['n'].append(self._get_info(i))

        return obs_n, reward_n, done_n, info_n

    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        obs_n = []

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

        for i in range(self.num_agents):
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message('robot%d/odom' % (i + 1), Odometry, timeout=5)
                except:
                    pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
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
        t.linear.x = (action[0][0] - action[0][1]) * self.linear_speed
        t.angular.z = (action[0][2] - action[0][3]) * self.angular_speed
        self.vel_pubs[agent].publish(t)
