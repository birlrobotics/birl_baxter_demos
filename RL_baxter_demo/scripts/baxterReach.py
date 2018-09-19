import gym
from gym.envs.robotics.baxter_env import BaxterEnv

env = gym.make('BaxterReacher-v0')
for i_episode in range(100):
    observation = env.reset()
    for t in range(100):
        env.render()
#        print('observation:'+str(observation))
#        print('env.action_space:'+str(env.action_space))
#        print('env.observation_space:'+str(env.observation_space))
        action = env.action_space.sample()
#        print('action:'+str(action))
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode finished after {} timesteps".format(t+1))
            break
