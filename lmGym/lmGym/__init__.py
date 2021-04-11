from gym.envs.registration import register

register(
     id='lmGym-v0',
     entry_point='lmGym.envs:lmEnv',
 )
