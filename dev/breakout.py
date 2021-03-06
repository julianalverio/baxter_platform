import sys
sys.path.insert(0, '/Users/julianalverio/venv/lib/python3.7/site-packages')

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import time


import math
import random
import numpy as np
from collections import namedtuple
from itertools import count
from PIL import Image
import os
import datetime
import yagmail
import pickle


# THIS IS A HACK SPECIFIC TO BAFFIN
import sys
sys.path.pop(0)
sys.path.insert(0, '/afs/csail.mit.edu/u/j/jalverio/.local/lib/python3.5/site-packages')
import gym


GPU_NUM = '0,1,2,3'
NUM_EPISODES = 25000
os.environ["CUDA_VISIBLE_DEVICES"] = GPU_NUM

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

class DQN(nn.Module):
    def __init__(self, num_actions, device, x):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(2, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        self.head = nn.Linear(np.product(x.size()), num_actions)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        return self.head(x.view(x.size(0), -1))



class Trainer(object):
    def __init__(self, num_episodes=5000, warm_start_path=''):
        self.env = gym.make('Breakout-v0').unwrapped
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.transition = namedtuple('Transition',
                                    ('state', 'action', 'next_state', 'reward'))
        self.num_episodes = num_episodes

        if not warm_start_path:
            test_state = self.getState(self.getScreen(), self.getScreen()).to(torch.device('cpu'))
            self.policy_net = DQN(self.env.action_space.n, self.device, test_state).to(self.device)
            self.target_net = DQN(self.env.action_space.n, self.device, test_state).to(self.device)
            torch.save(self.target_net, 'delete_initial_target_net')

            self.batch_size = 32
            self.gamma = 0.99
            self.eps_start = 0.999
            self.eps_end = 0.05
            self.eps_decay = 200
            self.target_update = 1000

            self.target_net.load_state_dict(self.policy_net.state_dict())
            self.target_net.eval()
            self.memory = ReplayMemory(100000, self.transition)

            self.steps_done = 0

        else:
            f = open(warm_start_path + '_params', 'r')
            param_dict = eval(f.read())
            self.batch_size = param_dict['batch_size']
            self.gamma = param_dict['gamma']
            self.eps_start = param_dict['eps_start']
            self.eps_end = param_dict['eps_end']
            self.eps_decay = param_dict['eps_decay']
            self.target_update = param_dict['target_update']
            self.steps_done = param_dict['steps_done']
            self.memory = ReplayMemory(10000, self.transition)

            self.policy_net = torch.load(warm_start_path + '_model.pth')
            self.target_net = torch.load(warm_start_path + '_model.pth')

        self.optimizer = optim.Adam(self.policy_net.parameters(), lr = 0.0001)

        self.param_dict = {
        'batch_size' : self.batch_size,
        'gamma': self.gamma,
        'eps_start' : self.eps_start,
        'eps_end': self.eps_end,
        'eps_decay': self.eps_decay,
        'target_update': self.target_update
        }


    def rgb2gray(self, rgb):
        img = Image.fromarray(rgb.transpose((1, 2, 0))).convert(mode='L')
        return np.array(img, dtype=np.float64)


    # All images in black and white
    def getScreen(self):
        # original size: 210x160x3
        image = Image.fromarray(self.env.render(mode='rgb_array')[25:195, 8:152, :]).resize((72, 85)).convert(mode='L')
        return torch.from_numpy(np.array(image)/255.).unsqueeze(0).unsqueeze(0).type(torch.FloatTensor).to(self.device)


    def selectAction(self, state):
        sample = random.random()
        eps_threshold = self.eps_end + (self.eps_start - self.eps_end) * \
            math.exp(-1. * self.steps_done / self.eps_decay)
        self.steps_done += 1
        if sample > eps_threshold:
            with torch.no_grad():
                return self.policy_net(state).max(1)[1].view(1, 1).type(torch.LongTensor).to(self.device)
        else:
            return torch.tensor([[self.env.action_space.sample()]], dtype=torch.long).to(self.device)



    def optimizeModel(self):
        if len(self.memory) < self.batch_size:
            return
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

        next_state_values = torch.zeros(self.batch_size, device=self.device)
        next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
        expected_state_action_values = (next_state_values * self.gamma) + reward_batch

        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))

        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()


    def getState(self, current_screen, last_screen):
        difference = current_screen - last_screen
        return torch.cat([current_screen, difference], dim=1)


    def train(self):
        for i_episode in range(self.num_episodes+1):
            start = datetime.datetime.now()
            print('Beginning Episode %s' % i_episode)
            self.steps_done = 0
            self.env.reset()
            last_screen = self.getScreen()
            current_screen = self.getScreen()
            state = self.getState(current_screen, last_screen)
            for t in count():
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
                    duration = (datetime.datetime.now() - start).total_seconds()
                    print('DURATION: %s' % duration)
                    break
            if i_episode % self.target_update == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())
            if i_episode % 500 == 0:
                try:
                    torch.save(self.target_net, 'breakout_%s.pth' % (i_episode))
                    # f = open('pacman_%s_params' % i_episode, 'w+')
                    # self.param_dict['steps_done'] = self.steps_done
                    # f.write(str(self.param_dict))
                    # f.close()
                    completionEmail('Breakout %s episodes completed' % i_episode)
                except Exception as e:
                    completionEmail('ERROR IN BREAKOUT %s' % i_episode)
                    print(e)
                    import pdb; pdb.set_trace()

    def playback(self, target_net_path):
        self.target_net = torch.load(target_net_path, map_location='cpu')
        self.env = gym.make('Breakout-v0').unwrapped
        self.env.reset()
        steps_done = 0
        done = False
        current_screen = self.getScreen()
        while not done:
            self.env.render(mode='human')
            time.sleep(0.025)
            previous_screen = current_screen
            current_screen = self.getScreen()
            state = self.getState(current_screen, previous_screen)
            action = self.target_net(state).max(1)[1].view(1, 1).type(torch.LongTensor)
            _, reward, done, _ = self.env.step(action.item())
            steps_done += 1
        print("Steps Done: ", steps_done)
        import sys; sys.exit()







def completionEmail(message=''):
  yag = yagmail.SMTP('infolab.rl.bot@gmail.com', 'baxter!@')
  yag.send('julian.a.alverio@gmail.com', 'Training Completed', [message])




trainer = Trainer(num_episodes=NUM_EPISODES)
print("Trainer Initialized")
# trainer.train()
# completionEmail('%s done' % NUM_EPISODES)
trainer.playback('breakout_24500.pth')
