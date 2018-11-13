#!/usr/bin/env python3
import gym
import ptan
import argparse

import torch
import torch.optim as optim

from tensorboardX import SummaryWriter

from lib import dqn_model, common
from itertools import count

import os; os.environ["CUDA_VISIBLE_DEVICES"] = '1'

torch.backends.cudnn.deterministic = True
torch.manual_seed(999)
torch.cuda.manual_seed_all(999)
import csv



class Trainer(object):
    def __init__(self):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.params = common.HYPERPARAMS['pong']
        self.env = gym.make('PongNoFrameskip-v4')
        #switch to wrappers from https://github.com/dxyang/DQN_pytorch (deep mind) #TODO
        self.env = ptan.common.wrappers.wrap_dqn(self.env)
        self.writer = SummaryWriter(comment="-" + self.params['run_name'] + "-01_original")
        self.policy_net = dqn_model.DQN(self.env.observation_space.shape, self.env.action_space.n).to(self.device)
        self.target_net = ptan.agent.TargetNet(self.policy_net)
        self.selector = ptan.actions.EpsilonGreedyActionSelector(epsilon=self.params['epsilon_start'])
        self.epsilon_tracker = common.EpsilonTracker(self.selector, self.params)
        agent = ptan.agent.DQNAgent(self.policy_net, self.selector, device=self.device)
        self.exp_source = ptan.experience.ExperienceSourceFirstLast(self.env, agent, gamma=self.params['gamma'], steps_count=1)
        self.buffer = ptan.experience.ExperienceReplayBuffer(self.exp_source, buffer_size=self.params['replay_size'])
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.params['learning_rate'])
        self.reward_tracker = common.RewardTracker()
        csv_file = open('losses.csv', 'w+')
        self.writer = csv.writer(csv_file)


    def train(self):
        counter = 0
        frame_idx = 0
        game_count = 0
        for episode in count():
            frame_idx += 1
            self.buffer.populate(1)
            self.epsilon_tracker.frame(frame_idx)

            new_rewards = self.exp_source.pop_total_rewards()
            if new_rewards:
                if len(self.buffer) >= self.params['replay_initial']:
                    game_count += 1
                    print('Game: %s Score: %s' % (game_count, self.reward_tracker.rewards[-1]))
                done = self.reward_tracker.add(new_rewards[0])
                if done:
                    break

            if len(self.buffer) < self.params['replay_initial']:
                continue

            self.optimizer.zero_grad()
            batch = self.buffer.sample(self.params['batch_size'])
            loss_v = common.calc_loss_dqn(batch, self.policy_net, self.target_net.target_model, gamma=self.params['gamma'], cuda=self.device)
            self.writer.writerow([loss_v])
            counter += 1
            if counter == 10000:
                print("done")
                return
            loss_v.backward()
            self.optimizer.step()

            if frame_idx % self.params['target_net_sync'] == 0:
                self.target_net.sync()
            if len(self.reward_tracker.rewards) % 100 == 0:
                torch.save(self.target_net, 'ptan_%s.pth' % len(self.reward_tracker.rewards))


if __name__ == "__main__":
    trainer = Trainer()
    trainer.train()







