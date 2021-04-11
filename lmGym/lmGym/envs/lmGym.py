import gym
from gym import error, spaces, utils
from gym.utils import seeding

class lmEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        #self.action_space = gym.spaces.box.Box(
        #    low = np.array()
        #    high = np.array()
        #)

        self.action_space = gym.spaces.multi_binary.MultiBinary(17)

        self.action_space = gym.spaces.box.Box(
            low = np.array()
            high = np.array()
        )
        self.np_random, _ = gym.utils.seeding.np_random()

        self.client = p.connect(p.DIRECT)

        # Reduce length of episodes for RL algorithms
        # timestep is 50 ms
        p.setTimeStep(1/20, self.client)

        self.lm = None
        self.goal = None
        self.done = False
        self.prev_dist_to_goal = None
        self.rendered_img = None
        self.render_rot_matrix = None
        self.reset()

    def step(self, action):
        # Feed action to the lm and get observations on the lm state
        self.lm.apply_action(action)
        p.stepSimulation()
        lm_ob = self.lm.get_observation()

        # Compute Rewards
        # For now the reward per timestep is simply how closer
        # lm module is compared to last setTimeStep
        # This reward is flawed as this would just crash
        # the lm module
        dist_to_goal = math.sqrt(((lm_ob[0] - self.goal[0]) ** 2 +
                                  (lm_ob[1] - self.goal[1]) ** 2))
        reward = max(self.prev_dist_to_goal - dist_to_goal, 0)
        self.prev_dist_to_goal = dist_to_goal

        # A lot of other reward function will have to be
        # considered along with this

        ob = np.array(lm_ob + self.goal, dtype = np.float32)

        return ob, reward, self.done, dict()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self):
        p.resetSimulation(self.client)
        p.setgravity(0, 0, -1.62)

        # Reload the URDFs
        Plane(self.client)
        self.lm = lm(self.client)

        # Set goal to random point on XY Plane
        x = (self.np_random.uniform(5, 9) if self.np_random.randint(2) else
             self.np_random.uniform(-5, -9))
        y = (self.np_random.uniform(5, 9) if self.np_random.randint(2) else
             self.np_random.uniform(-5, -9))
        self.goal = (x, y)
        self.done = False

        # Visual element of the goal
        Goal(self.client, self.goal)

        # Get observation to return
        lm_ob = self.lm.get_observation()

        # Reward that will be changed later
        self.prev_dist_to_goal = math.sqrt(((car_ob[0] - self.goal[0]) ** 2 +
                                           (car_ob[1] - self.goal[1]) ** 2))

        return np.array(lm_ob + self.goal, dtype=np.float32)

    # Note completely sure what role this plays exactly
    # Taken directly form the Github repo i was following
    def render(self, mode='human', close=False):

        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros(100,100,4))

        # Base information
        lm_id, client_id = self.car.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1,
                                                   nearVal=0.01, farVal=100)

        pos, ori = [list(l) for l in
                    p.getBasePositionAndOrientation(car_id, client_id)]
        pos[2] = 0.2

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)

    def close(self):
        p.disconnect(self.client)    
