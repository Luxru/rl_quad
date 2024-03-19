import time
from cv2 import log
import gymnasium as gym
import numpy as np
from gymnasium import spaces
from sympy import limit
import quad
from loguru import logger

class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    N_ACTIONS = 4
    N_OBS = 19
    
    def __init__(self):
        super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        g = 9.8
        limit_omega = 60/180*np.pi;
        limit_acc = 5*g;
        self.action_space_limit = np.array([limit_acc,limit_omega,limit_omega,limit_omega])
        self.action_space = spaces.Box(low=np.array([0,-1,-1,-1]), high=np.array([1, 1,1,1]), dtype=np.float32)
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=-np.Infinity,high=np.Infinity,shape=(self.N_OBS,))
        
        self.quad = quad.Quad()

        self.info_dict = {}
  
    def step(self, action):
        # Execute one time step within the environment
        action = action*self.action_space_limit
        self.quad.act(action)
        #logger.warning(f"action:{action}")
        self.quad.step()
        truncated = bool(self.quad.observe_collision())
        terminated = bool(self.quad.done())
        #Input: 需要修改
        observation = self.quad.get_state()
        #!TODO 
        reward = self.get_reward(action,terminated,truncated)
        #return state, reward, collision or target 
        return observation, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        self.quad.reset()  
        self.quad.step()
        #位置、姿态（四元素）、速度、角速度、加速度，目标点的位置差异、角度差异(这个怎么定义呢？)、如果沿着
        while(not self.quad.ready()):
            time.sleep(0.1)
        observation = self.quad.get_state()
        return observation, {}

    def render(self):
        pass

    def close(self):
        pass

    def get_reward(self,action,terminated,truncated):
        if(terminated):
            return 10
        if(truncated):
            return -5
        goal_index = self.quad.get_goal_index()
        pos_coeff= 1.0       # reward coefficient for position 
        reward = 0
        dist_goal = np.linalg.norm(np.array(self.quad.get_current_pose())-np.array(self.quad.get_waypoint(goal_index)))
        if(self.info_dict.get("dist_goal_t-1") is None):
            #t-1
            self.info_dict["dist_goal_t-1"] = dist_goal
        else:
            #closer
            err_dist = self.info_dict["dist_goal_t-1"] - dist_goal
            reward += err_dist*pos_coeff
        reward += 0.01 #survival reward
        assert not np.isnan(reward).any()
        #logger.info("reward:{}".format(reward))
        return reward