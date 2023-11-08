#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep  8 11:33:32 2020
Modified on Sat Oct  9 17:00:00 2021
Author: Rogério S. Silva
Email: rogeriosousaesilva@gmail.com
"""

import argparse
import random
import matplotlib.pyplot as plt
import numpy as np
from ns3gym import ns3env
from colorama import Fore, Back, Style

__author__ = "Rogério S. Silva"
__copyright__ = "Copyright (c) 2023, NumbERS - Federal Institute of Goiás, Inhumas - IFG"
__version__ = "0.1.0"
__email__ = "rogerio.sousa@ifg.edu.br"


def get_state_QIndex(n, o, nl, nc, na):
    # index arrays
    x = []
    y = []
    z = []
    for i in range(0, n * 3, 3):
        x.append(o[i] // 1000)
        y.append(o[i + 1] // 1000)
        z.append(o[i + 2] // 10 - 3)

    if not (len(x) == len(y) == len(z) == n):
        raise ValueError("The lengths of x, y, z, and N should be the same")

    storage_index = 0
    for i in range(n):
        storage_index = (x[i] + y[i] * nl + z[i] * (nl * nc)) * (nl * nc * na) ** i + storage_index

    return storage_index


def print_state(action, obs, reward, done, info):
    if reward > 0:
        print(
            Fore.LIGHTGREEN_EX + f"Step: {step} Action: {action} "
                                 f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
    else:
        if reward == -1:
            print(
                Fore.LIGHTWHITE_EX + f"Step: {step} Action: {action} "
                                     f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
        else:
            print(
                Fore.RED + f"Step: {step} Action: {action} "
                           f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")


port = 5555
nDevices = 10
# seed x #GW: [1,3], [2,1], [3,2], [4,2], [5,2], [6,2], [7,2], [8,1], [9,1], [10,2]
nGateways = 2
seed = 7
state = 0  # Initial state
simTime = 600  # seconds
stepTime = 600  # seconds
simArgs = {"--nDevices": nDevices,
           "--nGateways": nGateways}
startSim = 1
debug = 0

env = ns3env.Ns3Env(port=port, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)

ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space, ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

## Q-Learning
# Inicializando a "q-table"
nl = 10  # Number of lines
nc = 10  # Number of columns
na = 1  # Number of altitude levels
# STATE SIZE
# The states_size equation ignores possible collisions (drones occupying the same position)
state_size = (nl * nc * na) ** nGateways
# (500..9500, 500..9500 ) Step: 1000 according to te number of drones
# First drone can move 100 times, second 99 times (except to the first drone position),
# third 98 times (except to the first and second drone positions)
action_size = 4 * nGateways
# 0: up, 1: down, 2: left, 3: right, for each drone
qtable = np.zeros((state_size, action_size))

# Hiperparâmetros
learning_rate = 0.9
discount_rate = 0.8
epsilon = 1.0
decay_rate = 0.005

# Variáveis de treinamento
num_episodes = 10
max_steps = 10  # por episodio
action = -1
rewards = []

try:
    for episode in range(num_episodes):
        print(f"Episode: {episode}")
        obs, reward, done, info = env.get_state()
        sum_reward = 0
        for step in range(max_steps):

            print_state(action, obs, reward, done, info)

            if random.uniform(0, 1) < epsilon:
                # exploration
                action = env.action_space.sample()
            else:
                # exploitation
                action = np.argmax(qtable[state, :])

            # print(f"Action: {action}")
            obs, reward, done, info = env.step(action)
            new_state = get_state_QIndex(nGateways, obs, nl, nc, na)

            # Q-learning
            old_value = qtable[state, action]  # Value of the chosen action in the current state
            next_max = np.max(qtable[new_state])  # Maximum value of the next state

            # Q-learning equation
            new_value = (1 - learning_rate) * old_value + learning_rate * (reward + discount_rate * next_max)
            qtable[state, action] = new_value
            sum_reward = sum_reward + reward
            # Update to our new state
            state = new_state
            # print(f"State: {state}")

        rewards.append(sum_reward)
        env.reset()

        # Decrease epsilon
        epsilon = np.exp(-decay_rate * episode)

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    np.save("qtable.npy", qtable)
    env.close()
    print("Done")
    # Plot rewards
    average_reward = []
    for idx in range(len(rewards)):
        avg_list = np.empty(shape=(1,), dtype=int)
        if idx < 50:
            avg_list = rewards[:idx + 1]
        else:
            avg_list = rewards[idx - 49:idx + 1]
        average_reward.append(np.average(avg_list))
    # Plot
    plt.plot(rewards)
    plt.plot(average_reward)
