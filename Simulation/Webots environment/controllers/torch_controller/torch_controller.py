# %%
import gym
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
import matplotlib.pyplot as plt
from torch_utils import *
from ou_action_noise import OUActionNoise


# %%
torch.set_default_dtype(torch.float64)


# %%
problem = "gym_race:race-v0"
env = gym.make(problem)

num_states = env.observation_space.shape[0]
print("Size of State Space ->  {}".format(num_states))
num_actions = env.action_space.shape[0]
print("Size of Action Space ->  {}".format(num_actions))


# %%
class Buffer:
    def __init__(self, buffer_capacity=100000, batch_size=64):

        # Number of "experiences" to store at max
        self.buffer_capacity = buffer_capacity
        # Num of tuples to train on.
        self.batch_size = batch_size

        # Its tells us num of times record() was called.
        self.buffer_counter = 0

        # Instead of list of tuples as the exp.replay concept go
        # We use different np.arrays for each tuple element
        self.state_buffer = np.zeros((self.buffer_capacity, num_states))
        self.action_buffer = np.zeros((self.buffer_capacity, num_actions))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.next_state_buffer = np.zeros((self.buffer_capacity, num_states))

    # Takes (s,a,r,s') obervation tuple as input
    def record(self, obs_tuple):
        # Set index to zero if buffer_capacity is exceeded,
        # replacing old records
        index = self.buffer_counter % self.buffer_capacity

        self.state_buffer[index] = obs_tuple[0]
        self.action_buffer[index] = obs_tuple[1][0]
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = obs_tuple[3]

        self.buffer_counter += 1

    # We compute the loss and update parameters
    def learn(self):
        # Get sampling range
        record_range = min(self.buffer_counter, self.buffer_capacity)
        # Randomly sample indices
        batch_indices = np.random.choice(record_range, self.batch_size)

        # Convert to tensors
        state_batch = torch.from_numpy(self.state_buffer[batch_indices])
        action_batch = torch.from_numpy(self.action_buffer[batch_indices])
        reward_batch = torch.from_numpy(self.reward_buffer[batch_indices])
        next_state_batch = torch.from_numpy(self.next_state_buffer[batch_indices])

        # Training and updating Actor & Critic networks.
        # See Pseudo Code.
        target_actions = target_actor(next_state_batch)

        y = reward_batch + gamma * target_critic([next_state_batch, target_actions.detach()])
        
        critic_optimizer.zero_grad()
        critic_value = critic_model([state_batch, action_batch])
        critic_loss = F.mse_loss(critic_value, y.detach())
        critic_loss.backward()
        critic_optimizer.step()

        actor_optimizer.zero_grad()
        actions = actor_model(state_batch)
        critic_value = critic_model([state_batch, actions])
        # Used `-value` as we want to maximize the value given
        # by the critic for our actions
        actor_loss = -critic_value.mean()
        actor_loss.backward()
        actor_optimizer.step()

    def __len__(self):
        return self.buffer_counter


# This update target parameters slowly
# Based on rate `tau`, which is much less than one.
def update_target(tau):
    soft_update(target_critic, critic_model, tau)
    soft_update(target_actor, actor_model, tau)

# %%
class Actor(nn.Module):
    def __init__(self):
        super().__init__()

        self.model = nn.Sequential(
            nn.Linear(num_states, 512),
            nn.ReLU(),
            # Layer Norm is used instead of Batch Norm,
            # as Batch Norm does not work in the same way as in Keras
            nn.LayerNorm(512), 
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.LayerNorm(512),
            nn.Linear(512, num_actions),
            nn.Tanh()
        )

        self.model[-2].weight.data.uniform_(-0.003, 0.003)
        self.model[-2].bias.data.uniform_(-0.003, 0.003)

    def forward(self, inputs):
        return self.model(inputs)


def get_actor():
    return Actor()

class Critic(nn.Module):
    def __init__(self):
        super().__init__()

        self.state_model = nn.Sequential(
            nn.Linear(num_states, 16),
            nn.ReLU(),
            nn.LayerNorm(16),
            nn.Linear(16, 32),
            nn.ReLU(),
            nn.LayerNorm(32)
        )

        self.action_model = nn.Sequential(
            nn.Linear(num_actions, 32),
            nn.ReLU(),
            nn.LayerNorm(32)
        )

        self.out_model = nn.Sequential(
            nn.Linear(64, 512),
            nn.ReLU(),
            nn.LayerNorm(512),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.LayerNorm(512),
            nn.Linear(512, num_actions)
        )

    def forward(self, inputs):
        model_input = self.state_model(inputs[0])
        action_input = self.action_model(inputs[1])

        return self.out_model(torch.cat((model_input, action_input), dim=1))

def get_critic():
    return Critic()

# %%
def policy(state, noise_object):
    actor_model.eval()
    sampled_actions = actor_model(state).squeeze()
    actor_model.train()

    noise = torch.from_numpy(noise_object())
    # Adding noise to action
    sampled_actions = sampled_actions + noise

    return [sampled_actions.squeeze().detach().numpy()]


# %%
# Hyperparameters
std_dev = 0.5
ou_noise = OUActionNoise(theta=0.5,mean=np.array([0, 1]), std_deviation=float(std_dev) * np.ones(num_actions))

actor_model = get_actor()
critic_model = get_critic()

target_actor = get_actor()
target_critic = get_critic()

target_actor.eval()
target_critic.eval()

# Making the weights equal initially
hard_update(target_actor, actor_model)
hard_update(target_critic, critic_model)

# Learning rate for actor-critic models
critic_lr = 0.002
actor_lr = 0.001

critic_optimizer = optim.Adam(critic_model.parameters(), lr=critic_lr)
actor_optimizer = optim.Adam(actor_model.parameters(), lr=actor_lr)

total_episodes = 100
# Discount factor for future rewards
gamma = 0.99
# Used to update target networks
tau = 0.005

buffer = Buffer(50000, 64)

# %%
# Training loop
# To store reward history of each episode
ep_reward_list = []
# To store average reward history of last few episodes
avg_reward_list = []

# Takes about 20 min to train
for ep in range(total_episodes):

    prev_state = env.reset()
    episodic_reward = 0

    while True:
        # Uncomment this to see the Actor in action
        # But not in a python notebook.
        env.render()

        torch_prev_state = torch.tensor(prev_state).unsqueeze(dim=0)

        action = policy(torch_prev_state, ou_noise)
        # Recieve state and reward from environment.
        state, reward, done, info = env.step(action[0])

        buffer.record((prev_state, action, reward, state))
        episodic_reward += reward

        buffer.learn()
        update_target(tau)

        # End this episode when `done` is True
        if done:
            break

        prev_state = state

    ep_reward_list.append(episodic_reward)

    # Mean of last 40 episodes
    avg_reward = np.mean(ep_reward_list[-40:])
    print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))
    avg_reward_list.append(avg_reward)

