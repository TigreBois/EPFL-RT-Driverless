from utilities import normalizeToRange, plotData
from PPO_agent import PPOAgent, Transition

from gym.spaces import Box, Discrete, Dict
from vehicle import Driver
import numpy as np
from math import inf

import gym
import gym_race

env = gym.make("race-v0")
print(env)
print(env.observation_space.shape)
print(env.action_space)
agent = PPOAgent(numberOfInputs=5, numberOfActorOutputs=2)

solved = False

episodeCount = 0
episodeLimit = 2
episodeScore = 0

for _ in range(1000):
    env.step(env.action_space.sample())

env.close()