#!/usr/bin/env python3
import torch
torch.backends.cudnn.deterministic = True
torch.manual_seed(5)
torch.cuda.manual_seed_all(5)
import random
random.seed(5)
import numpy as np
np.random.seed(5)

import gym
import argparse

import torch
import torch.optim as optim

from tensorboardX import SummaryWriter

from lib import dqn_model, common
from other import actions, agent, experience
import other
import csv

import os; os.environ["CUDA_VISIBLE_DEVICES"]="1"


class RewardTracker:
    def __init__(self, length=100, stop_reward=20):
        self.stop_reward = stop_reward
        self.length = length
        self.rewards = []
        self.position = 0
        self.stop_reward = stop_reward

    def add(self, reward):
        if len(self.rewards) < self.length:
            self.rewards.append(reward)
        else:
            self.rewards[self.position] = reward
            self.position = (self.position + 1) % self.length
        if np.mean(self.rewards) >= self.stop_reward:
            return True
        return False


class EpsilonTracker:
    def __init__(self, epsilon_greedy_selector, params):
        self.epsilon_greedy_selector = epsilon_greedy_selector
        self.epsilon_start = params['epsilon_start']
        self.epsilon_final = params['epsilon_final']
        self.epsilon_frames = params['epsilon_frames']
        self.frame(0)

    def frame(self, frame):
        self.epsilon_greedy_selector.epsilon = \
            max(self.epsilon_final, self.epsilon_start - frame / self.epsilon_frames)


class Trainer(object):
    def __init__(self):
        self.params = common.HYPERPARAMS['pong']
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.env = gym.make('PongNoFrameskip-v4')
        self.env = other.common.wrappers.wrap_dqn(self.env)
        self.policy_net = dqn_model.DQN(self.env.observation_space.shape, self.env.action_space.n).to(self.device)
        self.target_net = agent.TargetNet(self.policy_net)
        self.selector = actions.EpsilonGreedyActionSelector(epsilon=self.params['epsilon_start'])
        self.epsilon_tracker = EpsilonTracker(self.selector, self.params)
        self.agent = agent.DQNAgent(self.policy_net, self.selector, device=self.device)
        self.exp_source = experience.ExperienceSourceFirstLast(self.env, self.agent, gamma=self.params['gamma'], steps_count=1)
        self.buffer = experience.ExperienceReplayBuffer(self.exp_source, buffer_size=self.params['replay_size'])
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.params['learning_rate'])
        self.reward_tracker = RewardTracker()
        csv_file = open('losses.csv', 'r')
        csv_reader = csv.reader(csv_file)
        self.losses = []
        for row in csv_reader:
            self.losses.append(row[0])
        self.episode = 0


    def train(self):
        frame_idx = 0
        counter = 0
        while True:
            frame_idx += 1
            self.buffer.populate()
            self.epsilon_tracker.frame(frame_idx)

            new_rewards = self.exp_source.pop_total_rewards()
            if new_rewards:
                self.episode += 1
                done = self.reward_tracker.add(new_rewards[0])
                print('Game: %s Score: %s Mean Score: %s' % (
                self.episode, self.reward_tracker.rewards[-1],
                np.mean(self.reward_tracker.rewards)))
                if (len(self.reward_tracker.rewards) % 100 == 0):
                    self.target_net.save('pong_%s.pth' % len(self.reward_tracker.rewards))
                    print('Model Saved!')
                if done:
                    break

            if len(self.buffer) < self.params['replay_initial']:
                continue
            self.optimizer.zero_grad()
            batch = self.buffer.sample(self.params['batch_size'])
            loss_v = common.calc_loss_dqn(batch, self.policy_net, self.target_net.target_model, gamma=self.params['gamma'])

            if loss_v.item() != float(self.losses[counter]):
                print('FAILURE')
                import pdb;
                pdb.set_trace()
            counter += 1
            if counter == 1:
                print("CHECKING NOW")
            if counter == 2000:
                print('ALL GOOD')
                break
            loss_v.backward()
            self.optimizer.step()

            if frame_idx % self.params['target_net_sync'] == 0:
                self.target_net.sync()


def playback(path):
    target_net = torch.load(path)
    env = gym.make('PongNoFrameskip-v4')





if __name__ == "__main__":
    trainer = Trainer()
    print('Trainer Initialized')
    trainer.train()






