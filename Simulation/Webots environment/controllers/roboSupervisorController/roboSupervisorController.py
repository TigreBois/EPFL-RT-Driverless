from utilities import normalizeToRange, plotData
from PPO_agent import PPOAgent, Transition

from gym.spaces import Box, Discrete, Dict
from vehicle import Driver
from math import inf

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess

import numpy as np
import gym

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam


env = gym.make("gym_race:race-v0")
print(env)
print(env.observation_space.shape)
print(env.action_space)

solved = False

episodeCount = 0
episodeLimit = 2
episodeScore = 0

nb_actions = env.action_space.shape[0]

nb_actions = env.action_space.shape[0]

#NN for actor
actor = Sequential()
actor.add(Flatten(input_shape=(1,)+env.observation_space.shape))
actor.add(Dense(32))
actor.add(Activation('relu'))
actor.add(Dense(32))
actor.add(Activation('relu'))
actor.add(Dense(32))
actor.add(Activation('relu'))
actor.add(Dense(nb_actions))
actor.add(Activation('linear'))
print(actor.summary())

# NN for critic
action_input = Input(shape=env.action_space.shape,name='action_input')
observation_input = Input(shape=(1,)+env.observation_space.shape, name = 'observation_input')
flattened_observation = Flatten()(observation_input)
x = Concatenate()([action_input, flattened_observation])
x = Dense(64)(x)
x = Activation('relu')(x)
x = Dense(64)(x)
x = Activation('relu')(x)
x = Dense(64)(x)
x = Activation('relu')(x)
x = Dense(1)(x)
x = Activation('linear')(x)
critic = Model(inputs=[action_input, observation_input], outputs=x)
print(critic.summary())

memory = SequentialMemory(limit=100000, window_length=1)

random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=1, mu=[0,0], sigma=.1)

agent = DDPGAgent(nb_actions = nb_actions, actor = actor, critic=critic, critic_action_input = action_input,
                 memory = memory, nb_steps_warmup_critic=100, nb_steps_warmup_actor=100,
                 random_process=random_process, gamma=.99, target_model_update=1e-3)
agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])

agent.fit(env, nb_steps=40000, visualize=False, verbose=0, nb_max_episode_steps=1000)



for _ in range(1000):
    env.step(env.action_space.sample())


env.close()