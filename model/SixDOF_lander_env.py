import gym
import numpy as np 
import pybullet as p 

class SixDOFLanderEnv(gym.Env):
    metadata = {'render.modes' : ['human']}

    def __init__(self):
        self.action_space = gym.spaces.multi_discrete.MultiDiscrete(
            nvec = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
        ) #States that action space is discrete, with 17 dimensions and each dimension can 
        ###take 2 values: 0 or 1

        self.observation_space = gym.spaces.box.Box(
            low = np.array([]), #This gives the lower bound of all the dimensions
            high = np.array([]) #This gives the upper bound of all the dimensions   
        ) #This obsercation space is continuous

       # self.np_random, _ = gym.utils.seeding.np.random()

        self.client = p.connect(p.DIRECT)

    def step(self, action):
        pass

    def reset(self):
        pass

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return[seed]
