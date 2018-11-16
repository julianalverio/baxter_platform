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

# from lib import dqn_model, common
# from other import actions, agent, experience
import other
import csv
import torch.nn as nn
import collections
import copy
from collections import namedtuple
from torch.autograd import Variable
import cv2


import os; os.environ["CUDA_VISIBLE_DEVICES"]="1"


HYPERPARAMS = {
        'env_name':         "PongNoFrameskip-v4",
        'stop_reward':      18.0,
        'run_name':         'pong',
        'replay_size':      100000,
        'replay_initial':   10000,
        'target_net_sync':  1000,
        'epsilon_frames':   10**5,
        'epsilon_start':    1.0,
        'epsilon_final':    0.02,
        'learning_rate':    0.0001,
        'gamma':            0.99,
        'batch_size':       32
}



class DQN(nn.Module):
    def __init__(self, input_shape, n_actions, device):
        super(DQN, self).__init__()
        self.device = device

        self.conv = nn.Sequential(
            nn.Conv2d(input_shape[0], 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU()
        )

        conv_out = self.conv(Variable(torch.zeros(1, *input_shape)))
        conv_out_size = int(np.prod(conv_out.size()))
        self.fc = nn.Sequential(
            nn.Linear(conv_out_size, 512),
            nn.ReLU(),
            nn.Linear(512, n_actions)
        )


    def forward(self, x):
        x = self.conv(x).view(x.size()[0], -1)
        return self.fc(x)


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
        self.params = HYPERPARAMS
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.env = gym.make('FetchPush-v1').unwrapped

        self.action_space = 6
        self.observation_space = [1, 163, 210]
        self.policy_net = DQN(self.observation_space, self.action_space, self.device).to(self.device)
        self.target_net = copy.deepcopy(self.policy_net)
        self.epsilon_tracker = EpsilonTracker(self.params)
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.params['learning_rate'])
        self.reward_tracker = RewardTracker()
        self.transition = namedtuple('Transition', ('state', 'action', 'reward', 'next_state'))
        self.memory = ReplayMemory(self.params['replay_size'], self.transition)
        self.episode = 0
        self.state = self.preprocess(self.reset())
        self.score = 0
        self.batch_size = self.params['batch_size']
        self.task = 1
        self.initial_object_position = copy.deepcopy(self.env.sim.data.get_site_xpos('object0'))
        self.movement_count = 0


    def reset(self):
        self.env.reset()
        self.env.render()
        self.env.viewer.cam.lookat[0] = 1.
        self.env.viewer.cam.lookat[1] = 1.5
        self.env.viewer.cam.lookat[2] = 1.1
        self.env.viewer.cam.azimuth = 165.
        self.env.viewer.cam.elevation = 10.
        self.env.viewer.cam.distance = 2.5
        self.env.sim.nsubsteps = 2
        self.env.block_gripper = True
        return self.env.render(mode='rgb_array')

    def preprocess(self, state):
        state = state[30:450, 100:425]
        state = cv2.cvtColor(state, cv2.COLOR_RGB2GRAY)
        state = cv2.resize(state, (210, 163), interpolation=cv2.INTER_AREA).transpose().astype(np.float32)/256
        return torch.tensor(state, device=self.device).unsqueeze(0).unsqueeze(0)


    def convertAction(self, action):
        movement = np.zeros(4)
        if action.item() % 2 == 0:
            movement[action.item() // 2] += 1
        else:
            movement[action.item() // 2] -= 1
        return movement


    def addExperience(self):
        self.movement_count += 1
        if random.random() < self.epsilon_tracker.epsilon():
            action = torch.tensor([random.randrange(self.action_space)], device=self.device)
        else:
            action = torch.argmax(self.policy_net(self.state), dim=1).to(self.device)
        self.env.step(self.convertAction(action))
        self.movement_count += 1
        next_state = self.preprocess(self.env.render(mode='rgb_array'))
        reward, done = self.getReward()
        done = done or self.movement_count == 1500
        self.score += reward
        if done:
            self.memory.push(self.state, action, torch.tensor([reward], device=self.device), None)
            self.state = self.preprocess(self.reset())
            self.initial_object_position = copy.deepcopy(self.env.sim.data.get_site_xpos('object0'))
            self.episode += 1
            self.movement_count = 0
        else:
            self.memory.push(self.state, action, torch.tensor([reward], device=self.device), next_state)
            self.state = next_state
        return done


    def optimizeModel(self):
        transitions = self.memory.sample(self.batch_size)
        batch = self.transition(*zip(*transitions))
        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None, batch.next_state)), device=self.device, dtype=torch.uint8)
        non_final_next_states = torch.cat([s for s in batch.next_state if s is not None])
        state_batch = torch.cat(list(batch.state))
        action_batch = torch.cat(list(batch.action))
        reward_batch = torch.cat(list(batch.reward))
        state_action_values = self.policy_net(state_batch).gather(1, action_batch.unsqueeze(1))
        next_state_values = torch.zeros(self.batch_size, device=self.device)
        next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
        expected_state_action_values = (next_state_values * self.params['gamma']) + reward_batch
        loss = nn.MSELoss()(state_action_values, expected_state_action_values.unsqueeze(1))
        self.optimizer.zero_grad()
        loss.backward()
        # for param in self.policy_net.parameters():
        #     param.grad.data.clamp_(-1, 1)
        self.optimizer.step()

    '''
    Task 1: Touch the block, discrete reward
    Task 2: Touch the block, continuous reward
    '''
    def getReward(self):
        gripper_position = self.env.sim.data.get_site_xpos('robot0:grip')
        object_position = self.env.sim.data.get_site_xpos('object0')
        if self.task == 1:
            if np.linalg.norm(self.initial_object_position - object_position) > 1e-3:
                return 1, True
            return 0, False
        if self.task == 2:
            distance =  np.linalg.norm(gripper_position - object_position)
            if distance <= 0.5:
                reward = 1.-distance
            if np.linalg.norm(self.initial_object_position - object_position) > 1e-3:
                return reward, True
            else:
                return reward, False

    def train(self):
        frame_idx = 0
        while True:
            frame_idx += 1
            # play one move
            done = self.addExperience()

            # are we done prefetching?
            if len(self.memory) < self.params['replay_initial']:
                continue
            if len(self.memory) == self.params['replay_initial']:
                print("Done Prefetching.")


            # is this round over?
            if done:
                import pdb; pdb.set_trace()
                self.reward_tracker.add(self.score)
                print('Episode: %s Score: %s Mean Score: %s' % (self.episode, self.score, self.reward_tracker.meanScore()))
                if (self.episode % 100 == 0):
                    torch.save(self.target_net, 'pong_%s.pth' % self.episode)
                    print('Model Saved!')
                if self.reward_tracker.meanScore() > 20:
                    print('Challenge Won in %s Episodes' % self.episode)
                    torch.save(self.target_net, 'pong_%s.pth' % self.episode)
                    print('Model Saved!')
                self.score = 0
                self.movement_count = 0


            self.optimizeModel()
            if frame_idx % self.params['target_net_sync'] == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())


    def playback(self, path):
        target_net = torch.load(path, map_location='cpu')
        env = gym.make('FetchPush-v1')
        env = other.common.wrappers.wrap_dqn(env)
        state = self.preprocess(env.reset())
        done = False
        score = 0
        import time
        while not done:
            env.render(mode='human')
            time.sleep(0.1)
            action = torch.argmax(target_net(state), dim=1).to(self.device)
            state, _, done, _ = env.step(action.item())
            score += self.getReward()
        print("Score: ", score)



if __name__ == "__main__":
    trainer = Trainer()
    print('Trainer Initialized')
    print("Pretaining Now...")
    trainer.train()
    # trainer.playback('pong_500.pth')






