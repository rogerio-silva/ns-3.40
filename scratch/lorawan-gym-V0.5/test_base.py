#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from ns3gym import ns3env
from colorama import Fore, Back, Style

__author__ = "Piotr Gawlowicz"
__copyright__ = "Copyright (c) 2018, Technische Universität Berlin"
__version__ = "0.1.0"
__email__ = "gawlowicz@tkn.tu-berlin.de"

parser = argparse.ArgumentParser(description='Start simulation script on/off')
parser.add_argument('--start',
                    type=int,
                    default=1,
                    help='Start ns-3 simulation script 0/1, Default: 1')
parser.add_argument('--iterations',
                    type=int,
                    default=10,
                    help='Number of iterations, Default: 10')
parser.add_argument('--steps',
                    type=int,
                    default=200,
                    help='Number of steps, Default: 500')
args = parser.parse_args()
startSim = bool(args.start)
iterations = int(args.iterations)
steps = int(args.steps)

port = 5555
nDevices = 10
nGateways = 1
startX = 5000
startY = 5000
startZ = 30
simTime = 600  # seconds
stepTime = 600  # seconds
seed = 1
simArgs = {"--nDevices": nDevices,
           "--nGateways": nGateways,
           "--startX": startX,
           "--startY": startY,
           "--startZ": startZ}
startSim = True
debug = True

env = ns3env.Ns3Env(port=port, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)

ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space, ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

stepIdx = 0
currIt = 0

try:
    while True:
        print("Start iteration: ", currIt)
        # print("Step: ", stepIdx)
        obs, reward, done, info = env.step(4)
        if reward > 0:
            print(
                Fore.RED + f"Step: {stepIdx} Action: none ---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
        else:
            print(
                Fore.LIGHTWHITE_EX + f"Step: {stepIdx} Action: none ---obs: {obs}, reward: {reward}, done: {done}, info: {info}")

        while True:
            stepIdx += 1
            action = env.action_space.sample()
            # print("---action: ", action)

            # print("Step: ", stepIdx)
            obs, reward, done, info = env.step(action)
            if reward > 0:
                print(
                    Fore.LIGHTGREEN_EX + f"Step: {stepIdx} Action: none "
                                         f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
            else:
                if reward == -1:
                    print(
                        Fore.LIGHTWHITE_EX + f"Step: {stepIdx} Action: none "
                                             f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
                else:
                    print(
                        Fore.RED + f"Step: {stepIdx} Action: none "
                                   f"---obs: {obs}, reward: {reward}, done: {done}, info: {info}")
            if stepIdx >= steps:
                env.reset()
                stepIdx = 0
                print()
                break

        currIt += 1
        if currIt == iterations:
            print()
            print()
            break

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    env.close()
    print("Done")
