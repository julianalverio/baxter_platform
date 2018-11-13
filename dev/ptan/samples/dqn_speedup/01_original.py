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


class Trainer(object):
    def __init__(self):
        self.params = common.HYPERPARAMS['pong']
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.CUDA = True if torch.cuda.is_available() else False
        self.env = gym.make('PongNoFrameskip-v4')
        self.env = other.common.wrappers.wrap_dqn(self.env)
        self.policy_net = dqn_model.DQN(self.env.observation_space.shape, self.env.action_space.n).to(self.device)
        self.target_net = agent.TargetNet(self.policy_net)
        self.selector = actions.EpsilonGreedyActionSelector(epsilon=self.params['epsilon_start'])
        self.epsilon_tracker = common.EpsilonTracker(self.selector, self.params)
        self.agent = agent.DQNAgent(self.policy_net, self.selector, device=self.device)
        self.exp_source = experience.ExperienceSourceFirstLast(self.env, self.agent, gamma=self.params['gamma'], steps_count=1)
        self.buffer = experience.ExperienceReplayBuffer(self.exp_source, buffer_size=self.params['replay_size'])
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.params['learning_rate'])
        self.reward_tracker = common.RewardTracker()
        csv_file = open('losses.csv', 'w+')
        # csv_reader = csv.reader(csv_file)
        self.csv_writer = csv.writer(csv_file)
        # self.losses = []
        # for row in csv_reader:
        #     self.losses.append(row[0])


    def train(self):
        frame_idx = 0
        counter = 0
        while True:
            print(len(self.buffer))
            frame_idx += 1
            self.buffer.populate(1)
            self.epsilon_tracker.frame(frame_idx)

            new_rewards = self.exp_source.pop_total_rewards()
            if new_rewards:
                done = self.reward_tracker.add(new_rewards[0])
                if done:
                    break

            if len(self.buffer) < self.params['replay_initial']:
                continue
            self.optimizer.zero_grad()
            batch = self.buffer.sample(self.params['batch_size'])
            loss_v = common.calc_loss_dqn(batch, self.policy_net, self.target_net.target_model, gamma=self.params['gamma'], cuda=self.CUDA)
            self.csv_writer.writerow([loss_v.item()])
            counter += 1
            if counter == 5000:
                print('DONE')
                break
            # if loss_v.item() != float(self.losses[counter]):
            #     print('FAILURE')
            #     import pdb;
            #     pdb.set_trace()
            # counter += 1
            # if counter == 1:
            #     print("CHECKING NOW")
            # if counter == 2000:
            #     print('ALL GOOD')
            #     break
            loss_v.backward()
            self.optimizer.step()

            if frame_idx % self.params['target_net_sync'] == 0:
                self.target_net.sync()


if __name__ == "__main__":
    trainer = Trainer()
    print('Trainer Initialized')
    trainer.train()






