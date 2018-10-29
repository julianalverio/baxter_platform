import gym
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from collections import namedtuple
from itertools import count
from PIL import Image

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T
import yagmail
import os
import gc


NUM_EPISODES = 10000
os.environ['CUDA_VISIBLE_DEVICES']='1,2,3'



class ReplayMemory(object):

    def __init__(self, capacity, transition):
        self.capacity = capacity
        self.memory = []
        self.position = 0
        self.transition = transition

    def push(self, *args):
        """Saves a transition."""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = self.transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)

class DQN(nn.Module):

    def __init__(self, num_actions, device):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(6, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)
        self.head = nn.Linear(2240, num_actions)
        self.device = device

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        return self.head(x.view(x.size(0), -1))


class Trainer(object):
    def __init__(self, num_episodes=NUM_EPISODES):
        self.env = gym.make('FetchPush-v1').unwrapped
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy_net = DQN(8, self.device).to(self.device)
        self.target_net = DQN(8, self.device).to(self.device)

        self.transition = namedtuple('Transition',
                                ('state', 'action', 'next_state', 'reward'))

        self.BATCH_SIZE = 128
        self.GAMMA = 0.999
        self.EPS_START = 0.9
        self.EPS_END = 0.05
        self.EPS_DECAY = 200
        self.TARGET_UPDATE = 10

        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.Adam(self.policy_net.parameters())
        self.memory = ReplayMemory(1000, self.transition)
        self.episode_durations = []

        self.num_episodes = num_episodes
        self.steps_done = 0
        self.state = [0,0,0,0]
        self.out_of_bounds = False


    # Grab and image, crop it, downsample and resize, then convert to tensor
    def getScreen(self):
        screen = Image.fromarray(self.env.render(mode='rgb_array')).crop((30, 100, 450, 425)).resize((105, 81), Image.NEAREST)
        return torch.from_numpy(np.array(screen, dtype=np.float32).transpose((2, 1, 0))).unsqueeze(0).to(self.device)


    def selectAction(self, state):
        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
            math.exp(-1. * self.steps_done / self.EPS_DECAY)
        self.steps_done += 1
        if sample > eps_threshold:
            with torch.no_grad():
                idx = self.policy_net(state).max(1)[1]
        else:
            idx = random.randrange(0, 8)
        if idx % 2 == 0:
            if self.state[idx//2] > 0.9:
                self.state[idx//2] = 1
                self.out_of_bounds = True
            else:
                self.state[idx//2] += 0.1
        else:
            if self.state[idx//2] < -0.9:
                self.state[idx//2] = -1.
                self.out_of_bounds = True
            else:
                self.state[idx//2] -= 0.1
        return idx



    def optimizeModel(self):
        if len(self.memory) < self.BATCH_SIZE:
            return
        transitions = self.memory.sample(self.BATCH_SIZE)
        batch = self.transition(*zip(*transitions))

        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                              batch.next_state)), device=self.device, dtype=torch.uint8)
        non_final_next_states = torch.cat([s for s in batch.next_state
                                                    if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        state_action_values = self.policy_net(state_batch).gather(1, action_batch)

        next_state_values = torch.zeros(self.BATCH_SIZE, device=self.device)
        next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
        expected_state_action_values = (next_state_values * self.GAMMA) + reward_batch

        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))

        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()

        del loss


    def getState(self, current_screen, last_screen):
        difference = current_screen - last_screen
        return torch.cat([current_screen, difference], 1)

    def reset(self):
        self.env.reset()
        self.env.viewer.cam.lookat[0] = 1.
        self.env.viewer.cam.lookat[1] = 1.5
        self.env.viewer.cam.lookat[2] = 1.1
        self.env.viewer.cam.azimuth = 165.
        self.env.viewer.cam.elevation = 10.
        self.env.viewer.cam.distance = 2.5
        self.state = [0,0,0,0]
        self.env.step(self.state)


    def train(self):
        for i_episode in range(self.num_episodes):
            print('Episode %s' % i_episode)
            self.steps_done = 0
            self.getScreen()
            self.reset()
            last_screen = self.getScreen()
            current_screen = self.getScreen()
            state = self.getState(current_screen, last_screen)
            for t in count():
                # import pdb; pdb.set_trace()
                # print(t)
                # print(torch.cuda.memory_allocated() / 1.049e+6, torch.cuda.max_memory_allocated() / 1.049e+6, torch.cuda.memory_cached() / 1.049e+6, torch.cuda.max_memory_cached() / 1.049e+6)
                # print('next expected: ', torch.cuda.memory_allocated()/1.049e+6 + 6030.336/1.049e+6)
                action = torch.tensor(self.selectAction(state), device=self.device).view(1, 1)
                _, reward, done, _ = self.env.step(self.state)
                if self.out_of_bounds:
                    reward -= 1.
                    self.out_of_bounds = False
                reward = torch.tensor([float(reward)], device=self.device)

                last_screen = current_screen
                current_screen = self.getScreen()
                if t == 1000:
                    done = True
                if not done:
                    next_state = self.getState(current_screen, last_screen)
                else:
                    next_state = None
                self.memory.push(state, action, next_state, reward)

                state = next_state

                self.optimizeModel()
                if done:
                    self.episode_durations.append(t + 1)
                    print(self.steps_done)
                    break
            if i_episode % self.TARGET_UPDATE == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())
        torch.save(self.target_net, 'openai_target_net_%s.pth' % self.num_episodes)

    def showResults(self, target_net_path):
        self.target_net = torch.load(target_net_path, map_location='cpu')
        self.env = gym.make('FetchPush-v1').unwrapped
        self.env.reset()
        steps_done = 0
        done = False
        current_screen = self.getScreen()
        for _ in range(1000):
            self.env.render(mode='human')
            previous_screen = current_screen
            current_screen = self.getScreen()
            state = self.getState(current_screen, previous_screen)
            action = self.target_net(state).max(1)[1].view(1, 1).type(torch.LongTensor)
            _, reward, done, _ = self.env.step(action.item())
            steps_done += 1
        print("Steps Done: ", steps_done)



def completionEmail(message=''):
  yag = yagmail.SMTP('infolab.rl.bot@gmail.com', 'baxter!@')
  yag.send('julian.a.alverio@gmail.com', 'Training Completed', [message])



trainer = Trainer(num_episodes=NUM_EPISODES)
print("Trainer Initialized")
try:
    trainer.train()
    completionEmail('%s done' % NUM_EPISODES)
except Exception as e:
    import pdb; pdb.set_trace()
    print(e)
