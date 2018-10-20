import gym
import math
import random
import numpy as np
import matplotlib
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
        self.conv1 = nn.Conv2d(4, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)
        self.head = nn.Linear(9792, num_actions)
        self.device = device

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        return self.head(x.view(x.size(0), -1))



class Trainer(object):
    def __init__(self, num_episodes=500, view=False):
        self.env = gym.make('MsPacman-v0').unwrapped
        plt.ion()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.policy_net = DQN(self.env.action_space.n, self.device).to(self.device)
        self.target_net = DQN(self.env.action_space.n, self.device).to(self.device)

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

        self.optimizer = optim.RMSprop(self.policy_net.parameters())
        self.memory = ReplayMemory(10000, self.transition)
        self.episode_durations = []

        self.num_episodes = num_episodes
        self.steps_done = 0
        self.view = view

    def rgb2gray(self, rgb):
        img = Image.fromarray(rgb.transpose((1,2,0))).convert(mode='L')
        return np.array(img, dtype=np.float64)


    def getScreen(self):
        # original size: 210x160x3
        if self.view:
            self.env.render()
        screen = self.env.render(mode='rgb_array')[0:170, :, ].transpose((2, 0, 1))
        screen = np.ascontiguousarray(screen, dtype=np.uint8)
        return torch.from_numpy(screen).to(self.device)


    def selectAction(self, state):
        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
            math.exp(-1. * self.steps_done / self.EPS_DECAY)
        self.steps_done += 1
        if sample > eps_threshold:
            with torch.no_grad():
                return self.policy_net(state).max(1)[1].view(1, 1).type(torch.LongTensor).to(self.device)
        else:
            return torch.tensor([[self.env.action_space.sample()]], dtype=torch.long).to(self.device)


    def plotDurations(self):
        plt.plot(range(len(self.durations), self.durations))
        plt.title('Training Episode Durations')
        plt.xlabel('Episode')
        plt.ylabel('Duration')
        plt.savefig('episode_durations.png')


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
        print('Loss: ', loss)

        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()


    def getState(self, current_screen, last_screen):
        difference = np.array(current_screen) - np.array(last_screen)
        gray = torch.tensor(self.rgb2gray(difference), device=self.device) / 255
        current_screen = current_screen / 255
        return torch.tensor(np.concatenate((current_screen.unsqueeze(0), gray.unsqueeze(0).unsqueeze(0)), axis=1), device=self.device).type(torch.FloatTensor).to(self.device)


    def train(self):
        for i_episode in xrange(self.num_episodes):
            self.steps_done = 0
            self.env.reset()
            last_screen = self.getScreen()
            current_screen = self.getScreen()
            state = self.getState(current_screen, last_screen)
            for t in count():
                print('Episode %s Movement %s' % (i_episode, t))
                action = self.selectAction(state)
                _, reward, done, _ = self.env.step(action.item())
                reward = torch.tensor([reward], device=self.device)

                last_screen = current_screen
                current_screen = self.getScreen()
                if not done:
                    next_state = self.getState(current_screen, last_screen)
                else:
                    next_state = None

                self.memory.push(state, action, next_state, reward)

                state = next_state

                self.optimizeModel()
                if done:
                    self.episode_durations.append(t + 1)
                    break
            if i_episode % self.TARGET_UPDATE == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())
        torch.save(self.target_net, 'target_net_%s.pth' % num_episodes)
        self.plotDurations()

    def showPacman(self, target_net_path):
        self.target_net = torch.load(target_net_path, map_location='cpu')
        self.env = gym.make('MsPacman-v0').unwrapped
        self.env.reset()
        self.steps_done = 0
        done = False
        current_screen = self.getScreen()
        while not done:
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




trainer = Trainer(view=False)
trainer.train()
# trainer.showPacman('target_net_100.pth')
