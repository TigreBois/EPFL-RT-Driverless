from deepbots.supervisor.controllers.robot_supervisor import RobotSupervisor
from utilities import normalizeToRange, plotData
from PPO_agent import PPOAgent, Transition

from gym.spaces import Box, Discrete, Dict
from vehicle import Driver
import numpy as np
from math import inf

import gym

START_THRESHOLD = 5 # in seconds. Max time to start the car.

env = gym.make("gym_race:race-v0")
print(env)
#agent = PPOAgent(numberOfInputs=env.observation_space.shape[0], numberOfActorOutputs=env.action_space.shape[0])

solved = False

episodeCount = 0
episodeLimit = 2000

# Run outer loop until the episodes limit is reached or the task is solved
while not solved and episodeCount < episodeLimit:
    observation = env.reset()  # Reset robot and get starting observation
    env.episodeScore = 0
    
    for step in range(env.stepsPerEpisode):
        # In training mode the agent samples from the probability distribution, naturally implementing exploration
        #selectedAction, actionProb = agent.work(observation, type_="selectAction")
        selectedAction = [5,0]
        # Step the supervisor to get the current selectedAction's reward, the new observation and whether we reached 
        # the done condition
        
        newObservation, reward, done, info = env.step(selectedAction)

        # Save the current state transition in agent's memory
        #trans = Transition(observation, selectedAction, actionProb, reward, newObservation)
        #agent.storeTransition(trans)
        
        if done:
            # Save the episode's score
            env.episodeScoreList.append(env.episodeScore)
            #agent.trainStep(batchSize=step)
            solved = env.solved()  # Check whether the task is solved
            break

        env.episodeScore += reward  # Accumulate episode reward
        observation = newObservation  # observation for next step is current step's newObservation
        
    print("Episode #", episodeCount, "score:", env.episodeScore)
    episodeCount += 1  # Increment episode counter

if not solved:
    print("Task is not solved, deploying agent for testing...")
elif solved:
    print("Task is solved, deploying agent for testing...")
observation = env.reset()
#while True:
    #selectedAction, actionProb = agent.work(observation, type_="selectActionMax")
    #observation, _, _, _ = env.step([selectedAction])