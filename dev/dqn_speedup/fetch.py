#!/usr/bin/env python3
import torch
torch.backends.cudnn.deterministic = True
torch.manual_seed(5)
torch.cuda.manual_seed_all(5)
import random
random.seed(5)
import numpy as np
np.random.seed(5)

import torch
import torch.optim as optim

import torch.nn as nn
from PIL import Image
import copy
from collections import namedtuple
from torch.autograd import Variable
import cv2
import time


import os; os.environ["CUDA_VISIBLE_DEVICES"]="3"

import sys
# sys.path.pop(0)
# import gym
from gym.envs.robotics import fetch_env
from gym import utils
from gym.wrappers.time_limit import TimeLimit


HYPERPARAMS = {
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
        print(old_epsilon)
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
        # self.env = gym.make('FetchPush-v1').unwrapped
        self.env = self.makeEnv()
        self.env = self.env.unwrapped

        self.action_space = 6
        self.observation_space = [3, 102, 205]
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
        self.task = 2
        self.initial_object_position = copy.deepcopy(self.env.sim.data.get_site_xpos('object0'))
        self.movement_count = 0


    def makeEnv(self):
        initial_qpos = {
            'robot0:slide0': 0.405,
            'robot0:slide1': 0.48,
            'robot0:slide2': 0.0,
            'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
        }
        env = fetch_env.FetchEnv('fetch/push.xml', has_object=True, block_gripper=True, n_substeps=20,
            gripper_extra_height=0.0, target_in_the_air=False, target_offset=0.0,
            obj_range=0.15, target_range=0.15, distance_threshold=0.05,
            initial_qpos=initial_qpos, reward_type='sparse')
        return TimeLimit(env)


        # MODEL_XML_PATH = 'fetch/push.xml'
        # initial_qpos = {
        #     'robot0:slide0': 0.405,
        #     'robot0:slide1': 0.48,
        #     'robot0:slide2': 0.0,
        #     'object0:joint': [1.25, 0.53, 0.4, 1., 0., 0., 0.],
        # }
        # env = fetch_env.FetchEnv.__init__(
        #     self, MODEL_XML_PATH, has_object=True, block_gripper=True, n_substeps=20,
        #     gripper_extra_height=0.0, target_in_the_air=False, target_offset=0.0,
        #     obj_range=0.15, target_range=0.15, distance_threshold=0.05,
        #     initial_qpos=initial_qpos, reward_type='sparse')
        # utils.EzPickle.__init__(self)


    def reset(self):
        self.env.reset()
        self.env.render()
        self.env.sim.nsubsteps = 2
        return self.env.render(mode='rgb_array')

    def preprocess(self, state):
        state = state[230:435, 50:460]
        # state = cv2.cvtColor(state, cv2.COLOR_RGB2GRAY)
        state = cv2.resize(state, (state.shape[1]//2, state.shape[0]//2), interpolation=cv2.INTER_AREA).astype(np.float32)/256
        np.swapaxes(state, 0, 2)
        return torch.tensor(state, device=self.device).unsqueeze(0)

    # indices are x, y, z, gripper
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
        if done:
            self.memory.push(self.state, action, torch.tensor([reward], device=self.device), None)
            self.state = self.preprocess(self.reset())
            self.initial_object_position = copy.deepcopy(self.env.sim.data.get_site_xpos('object0'))
            self.episode += 1
            self.movement_count = 0
        else:
            self.memory.push(self.state, action, torch.tensor([reward], device=self.device), next_state)
            self.state = next_state
        if self.state == next_state:
            print('CURRENT STATE EQUAL TO NEXT STATE')
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
        # if self.task == 1:
        #     if np.linalg.norm(self.initial_object_position - object_position) > 1e-3:
        #         self.score += 1.
        #         return 1., True
        #     return 0., False
        if self.task == 2:
            distance = np.linalg.norm(gripper_position - object_position)
            reward = 1./distance
            if self.env.sim.data.get_site_xpos('robot0:grip') > 0.6:
                reward = -1.
            if np.linalg.norm(self.initial_object_position - object_position) > 1e-3:
                self.score += 1.
                return reward, True
            else:
                return reward, False

    def train(self):
        frame_idx = 0
        while True:
            frame_idx += 1
            # execute one move
            done = self.addExperience()

            # are we done prefetching?
            if len(self.memory) < self.params['replay_initial']:
                continue
            if len(self.memory) == self.params['replay_initial']:
                self.episode, self.movement_count = 0, 0
                print("Done Prefetching.")


            # is this round over?
            if done:
                self.reward_tracker.add(self.score)
                print('Episode: %s Score: %s Mean Score: %s' % (self.episode, self.score, self.reward_tracker.meanScore()))
                if (self.episode % 100 == 0):
                    torch.save(self.target_net, 'pong_%s.pth' % self.episode)
                    print('Model Saved!')
                self.score = 0
                self.movement_count = 0


            self.optimizeModel()
            if frame_idx % self.params['target_net_sync'] == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())


    def playback(self, path):
        target_net = torch.load(path)
        state = self.preprocess(self.reset())
        self.env.render()
        done = False
        while not done:
            self.env.render(mode='human')
            action = self.convertAction(torch.argmax(target_net(state), dim=1).to(self.device))
            self.env.step(action)
            state = self.preprocess(self.env.render(mode='rgb_array'))



if __name__ == "__main__":
    trainer = Trainer()
    print('Trainer Initialized')
    print("Prefetching Now...")
    trainer.train()
    # trainer.playback('pong_3200.pth')






