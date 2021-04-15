import gym
import lmGym
import numpy as np 
from dqn_keras import Agent
from gym import wrappers

if __name__ == '__main__':

    agent = Agent(alpha = 0.0005, gamma = 0.99, n_actions = 17,
                epsilon = 0.9, batch_size=32, input_dims=13)
    env = gym.make('lmGym-v0')
    score_history = []
    score = 0
    n_episodes = 2500

    #env = wrappers.Monitor(env, 'tmp/lunar-lander',
    #                     video_callable=lambda episode_id: True, force = True)

    for i in range(n_episodes):

        print('episode', i, 'score', score)
        done = False
        score = 0
        observation = env.reset()

        while not done:
            act = agent.choose_action(observation)
            act_string = bin(act)[2:].zfill(17)
            act_mat  = []
            for i in act_string:
                act_mat.append(int(i))
            action = np.array(act_mat)
            observation_, reward, done, info = env.step(action)
            agent.store_transitions(observation, action, state)
            observation = observation_
            score += reward

        score_history.append(score)
        agent.learn()
        agent.save_checkpoint()

    file = 'lunar-lander.png'
