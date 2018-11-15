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
import torch.nn as nn
import collections
import copy
from collections import namedtuple

import os; os.environ["CUDA_VISIBLE_DEVICES"]="1"



class DQN(nn.Module):
    def __init__(self, input_shape, n_actions):
        super(DQN, self).__init__()

        self.conv = nn.Sequential(
            nn.Conv2d(input_shape[0], 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU()
        )

        conv_out_size = self._get_conv_out(input_shape)
        self.fc = nn.Sequential(
            nn.Linear(conv_out_size, 512),
            nn.ReLU(),
            nn.Linear(512, n_actions)
        )

    def _get_conv_out(self, shape):
        o = self.conv(Variable(torch.zeros(1, *shape)))
        return int(np.prod(o.size()))

    # input is a lazyframes object
    def forward(self, state):
        import pdb; pdb.set_trace()
        x = torch.tensor(np.expand_dims(state, 0)).to(self.device)
        x = x.float() / 256
        x = self.conv(x).view(x.size()[0], -1)
        return self.fc(x)


# class DQNAgent(object):
#     def __init__(self, dqn_model, device="cpu"):
#         self.dqn_model = dqn_model
#         self.device = device
#
#     def __call__(self, state):
#         state = torch.tensor(np.expand_dims(state, 0)).to(self.device)
#         q_v = self.dqn_model(state)
#         return q_v


class TargetNet:
    def __init__(self, model):
        self.model = model
        self.target_model = copy.deepcopy(model)

    def sync(self):
        self.target_model.load_state_dict(self.model.state_dict())

    def save(self, name):
        torch.save(self.target_model, name)


class RewardTracker:
    def __init__(self, length=100, stop_reward=20):
        self.stop_reward = stop_reward
        self.length = length
        self.rewards = []
        self.position = 0
        self.stop_reward = stop_reward
        self.mean_score = 0

    def add(self, reward):
        if len(self.rewards) < self.length:
            self.rewards.append(reward)
        else:
            self.rewards[self.position] = reward
            self.position = (self.position + 1) % self.length
        self.mean_score = np.mean(self.rewards)

    def meanScore(self):
        return self.mean_score


class EpsilonTracker:
    def __init__(self, params):
        self._epsilon = params['epsilon_start']
        self.epsilon_final = params['epsilon_final']
        self.epsilon_delta = 1.0 * (params['epsilon_start'] - params['epsilon_final']) / params['epsilon_frames']

    def epsilon(self):
        old_epsilon = self._epsilon
        self._epsilon -= self.epsilon_delta
        return max(old_epsilon, self.epsilon_final)


# def unpack_batch(batch):
#     states, actions, rewards, dones, last_states = [], [], [], [], []
#     for exp in batch:
#         state = np.array(exp.state, copy=False)
#         states.append(state)
#         actions.append(exp.action)
#         rewards.append(exp.reward)
#         dones.append(exp.last_state is None)
#         if exp.last_state is None:
#             last_states.append(state)       # the result will be masked anyway
#         else:
#             last_states.append(np.array(exp.last_state, copy=False))
#     return np.array(states, copy=False), np.array(actions), np.array(rewards, dtype=np.float32), \
#            np.array(dones, dtype=np.uint8), np.array(last_states, copy=False)
#
#
# def calc_loss_dqn(batch, net, tgt_net, gamma, cuda=True, cuda_async=False):
#     ExperienceFirstLast = collections.namedtuple('ExperienceFirstLast', ('state', 'action', 'reward', 'next_state'))
#     batch = ExperienceFirstLast(*zip(*batch))
#     non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
#                                             batch.next_state)), device=torch.device('cuda'), dtype=torch.uint8)
#     non_final_next_states = [np.array(s, copy=False) for s in batch.next_state if s is not None]
#     import pdb; pdb.set_trace()
#     state_batch = torch.tensor(np.array([np.array(state, copy=False) for state in batch.state], copy=False))
#     action_batch = torch.cat([torch.tensor(int(x)).view((1,1)) for x in batch.action])
#     reward_batch = torch.cat([torch.tensor(int(x)).view((1,1)) for x in batch.reward])
#     state_action_values = self.policy_net(state_batch).gather(1, action_batch)
#     next_state_values = torch.zeros(self.batch_size * self.steps_before_optimize, device=torch.device('cuda'))
#     next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
#     expected_state_action_values = (next_state_values * gamma) + reward_batch
#     return nn.MSELoss()(state_action_values, expected_state_action_values)

    #From my old pong:
    # self.optimizer.zero_grad()
    # loss.backward()
    # for param in self.policy_net.parameters():
    #     param.grad.data.clamp_(-1, 1)
    # self.optimizer.step()


    # states, actions, rewards, dones, next_states = unpack_batch(batch)
    #
    # states_v = torch.tensor(states)
    # next_states_v = torch.tensor(next_states)
    # actions_v = torch.tensor(actions)
    # rewards_v = torch.tensor(rewards)
    # done_mask = torch.ByteTensor(dones)
    # if cuda:
    #     states_v = states_v.cuda(non_blocking=cuda_async)
    #     next_states_v = next_states_v.cuda(non_blocking=cuda_async)
    #     actions_v = actions_v.cuda(non_blocking=cuda_async)
    #     rewards_v = rewards_v.cuda(non_blocking=cuda_async)
    #     done_mask = done_mask.cuda(non_blocking=cuda_async)
    #
    # state_action_values = net(states_v).gather(1, actions_v.unsqueeze(-1)).squeeze(-1)
    # next_state_values = tgt_net(next_states_v).max(1)[0]
    # next_state_values[done_mask] = 0.0
    #
    # expected_state_action_values = next_state_values.detach() * gamma + rewards_v
    # return nn.MSELoss()(state_action_values, expected_state_action_values)




class ReplayMemory(object):

    def __init__(self, capacity, transition):
        self.capacity = capacity
        self.memory = []
        self.position = 0
        self.transition = transition

    def push(self, *args):
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = self.transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)



class Trainer(object):
    def __init__(self):
        self.params = common.HYPERPARAMS['pong']
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.env = gym.make('PongNoFrameskip-v4')

        self.env = other.common.wrappers.wrap_dqn(self.env)
        self.policy_net = DQN(self.env.observation_space.shape, self.env.action_space.n).to(self.device)
        self.target_net = TargetNet(self.policy_net)
        # self.selector = actions.EpsilonGreedyActionSelector(epsilon=self.params['epsilon_start'])
        self.epsilon_tracker = EpsilonTracker(self.params)
        # self.agent = DQNAgent(self.policy_net, device=self.device)
        # self.exp_source = experience.ExperienceSourceFirstLast(self.env, self.agent, gamma=self.params['gamma'], steps_count=1)
        # self.buffer = experience.ExperienceReplayBuffer(self.exp_source, buffer_size=self.params['replay_size'])
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.params['learning_rate'])
        self.reward_tracker = RewardTracker()
        self.transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))
        # csv_file = open('losses.csv', 'r')
        # csv_reader = csv.reader(csv_file)
        self.memory = ReplayMemory(self.params['replay_size'], self.transition)
        # self.losses = []
        # for row in csv_reader:
        #     self.losses.append(row[0])
        self.episode = 0
        self.state = self.env.reset()
        self.score = 0
        self.batch_size = self.params['batch_size']



    def addExperience(self):
        if random.random() < self.epsilon_tracker.epsilon():
            action = random.randrange(self.env.action_space.n)
        else:
            action = torch.argmax(self.policy_net(self.state), dim=1)
        next_state, reward, done, _ = self.env.step(action)
        self.score += reward
        if done:
            self.memory.push(self.state, torch.tensor([action]), torch.tensor([reward], device=self.device), None)
            self.state = self.env.reset()
            self.episode += 1
        else:
            self.memory.push(self.state, torch.tensor([action]), torch.tensor([reward], device=self.device), next_state)
            self.state = next_state
        return done


    def calculateLoss(self):
        transitions = self.memory.sample(self.batch_size)
        batch = self.transition(*zip(*transitions))

        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                                batch.next_state)), device=self.device, dtype=torch.uint8)
        non_final_next_states = torch.cat([s for s in batch.next_state
                                           if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        state_action_values = self.policy_net(state_batch).gather(1, action_batch)
        next_state_values = torch.zeros(self.batch_size * self.steps_before_optimize, device=self.device)
        next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
        expected_state_action_values = (next_state_values * self.gamma) + reward_batch
        return nn.MSELoss()(state_action_values, expected_state_action_values)




    def train(self):
        frame_idx = 0
        while True:
            frame_idx += 1
            game_over = self.addExperience()

            # is this round over?
            if game_over:
                print('Game: %s Score: %s Mean Score: %s' % (self.episode, self.score, self.reward_tracker.meanScore()))
                if (self.episode % 100 == 0):
                    self.target_net.save('pong_%s.pth' % self.episode)
                    print('Model Saved!')
                if self.reward_tracker.meanScore() > 20:
                    print('Challenge Won in %s Episodes' % self.episode)

            # are we done prefetching?
            if len(self.memory) < self.params['replay_initial']:
                continue

            loss = self.calculateLoss()
            self.optimizer.zero_grad()
            loss.backward()
            # for param in self.policy_net.parameters():
            #     param.grad.data.clamp_(-1, 1)

            self.optimizer.step()

            if frame_idx % self.params['target_net_sync'] == 0:
                self.target_net.sync()

#
# def playback(path):
#     target_net = torch.load(path)
#     env = gym.make('PongNoFrameskip-v4')
#
#



if __name__ == "__main__":
    trainer = Trainer()
    print('Trainer Initialized')
    trainer.train()






