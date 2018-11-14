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
import collections

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

def unpack_batch(batch):
    states, actions, rewards, dones, last_states = [], [], [], [], []
    for exp in batch:
        state = np.array(exp.state, copy=False)
        states.append(state)
        actions.append(exp.action)
        rewards.append(exp.reward)
        dones.append(exp.last_state is None)
        if exp.last_state is None:
            last_states.append(state)       # the result will be masked anyway
        else:
            last_states.append(np.array(exp.last_state, copy=False))
    return np.array(states, copy=False), np.array(actions), np.array(rewards, dtype=np.float32), \
           np.array(dones, dtype=np.uint8), np.array(last_states, copy=False)


def calc_loss_dqn(batch, net, tgt_net, gamma, cuda=True, cuda_async=False):
    ExperienceFirstLast = collections.namedtuple('ExperienceFirstLast', ('state', 'action', 'reward', 'next_state'))
    batch = ExperienceFirstLast(*zip(*batch))
    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                            batch.next_state)), device=torch.device('cuda'), dtype=torch.uint8)
    non_final_next_states = [np.array(s, copy=False) for s in batch.next_state if s is not None]
    import pdb; pdb.set_trace()
    state_batch = torch.tensor(np.array([np.array(state, copy=False) for state in batch.state], copy=False))
    action_batch = torch.cat([torch.tensor(x) for x in batch.action])
    reward_batch = torch.cat([torch.tensor(x) for x in batch.reward])
    state_action_values = self.policy_net(state_batch).gather(1, action_batch)
    next_state_values = torch.zeros(self.batch_size * self.steps_before_optimize, device=torch.device('cuda'))
    next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
    expected_state_action_values = (next_state_values * gamma) + reward_batch
    return nn.MSELoss()(state_action_values, expected_state_action_values)

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
            loss_v = calc_loss_dqn(batch, self.policy_net, self.target_net.target_model, gamma=self.params['gamma'])

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






