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
import csv
import copy
#HACK SPECIFIC TO MELVILLE
import sys; sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages')

# THIS IS A HACK SPECIFIC TO BAFFIN
# import sys
# sys.path.pop(0)
# sys.path.insert(0, '/afs/csail.mit.edu/u/j/jalverio/.local/lib/python3.5/site-packages')
import gym


GPU_NUM = '1'
NUM_EPISODES = 1000
os.environ["CUDA_VISIBLE_DEVICES"] = GPU_NUM

torch.backends.cudnn.deterministic = True
torch.manual_seed(999)
torch.backends.cudnn.deterministic = True
torch.cuda.manual_seed_all(999)
np.random.seed(999)

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
        self.conv = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU()
        )
        # self.conv1 = nn.Conv2d(1, 16, kernel_size=5, stride=2)
        # self.bn1 = nn.BatchNorm2d(16)
        # self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        # self.bn2 = nn.BatchNorm2d(32)
        # self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        # self.bn3 = nn.BatchNorm2d(32)
        # x = F.relu(self.bn1(self.conv1(x)))
        # x = F.relu(self.bn2(self.conv2(x)))
        # x = F.relu(self.bn3(self.conv3(x)))
        # self.head = nn.Linear(np.product(x.size()), 3)
        conv_out_size = np.product(self.conv(x).size())
        self.fc = nn.sequential(
            nn.Linear(conv_out_size, 512),
            nn.ReLU(),
            nn.Linear(512, 3)
        )

    def forward(self, x):
        # x = F.relu(self.bn1(self.conv1(x)))
        # x = F.relu(self.bn2(self.conv2(x)))
        # x = F.relu(self.bn3(self.conv3(x)))
        # return self.head(x.view(x.size(0), -1))
        conv_out = self.conv(x).view(x.size()[0], -1)
        self.fc(conv_oput)


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

    def forward(self, x):
        fx = x.float() / 256
        conv_out = self.conv(fx).view(fx.size()[0], -1)
        return self.fc(conv_out)


class Trainer(object):
    def __init__(self, num_episodes=5000, warm_start_path=''):
        self.env = gym.make('Pong-v0').unwrapped
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.transition = namedtuple('Transition',
                                    ('state', 'action', 'next_state', 'reward'))
        self.num_episodes = num_episodes

        self.VALID = [0, 2, 3]

        if not warm_start_path:
            test_state =  self.preprocess(self.env.render(mode='rgb_array')).to(torch.device('cpu'))
            self.policy_net = DQN(self.env.action_space.n, self.device, test_state).to(self.device)
            self.target_net = DQN(self.env.action_space.n, self.device, test_state).to(self.device)

            self.batch_size = 32
            self.gamma = 0.99
            self.eps_start = 1.
            self.eps_end = 0.2
            # self.eps_decay = 200
            self.decay_steps = 100
            self.target_update = 100

            self.target_net.load_state_dict(self.policy_net.state_dict())
            self.target_net.eval()
            self.memory = ReplayMemory(100000, self.transition)

            self.steps_done = 0
            self.prefetch_episodes = 10000
            # print('Prefetching %s Random State Transitions...' % self.prefetch_episodes)
            # self.prefetch()
            self.steps_before_optimize = 1
            csv_file = open("results_not_optimized_no_prefetch.csv", 'w+')
            self.writer = csv.writer(csv_file)

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


    def prefetch(self):
        counter = 0
        while 1:
            done = False
            self.env.reset()
            while not done:
                counter += 1
                state = self.getScreen()
                action = torch.tensor([[random.randint(0,2)]], dtype=torch.long).to(self.device, non_blocking=True)
                action_number = self.VALID[action]
                _, reward, done, _ = self.env.step(action_number)
                reward = torch.tensor([reward], device=self.device)
                next_state = self.getScreen()
                self.memory.push(state, action, next_state, reward)
                if counter == self.prefetch_episodes:
                    return



    # original size: 210x160x3
    # final dims: 1x1x85x72
    def getScreen(self):
        # new size: 170 x 146
        img = self.env.render(mode='rgb_array')[25:195, 8:152, :]
        img = img[:, :, 0] * 0.299 + img[:, :, 1] * 0.587 + img[:, :, 2] * 0.114
        img = np.array(Image.fromarray(img).resize((72, 85)))
        # return torch.from_numpy(img/255.).unsqueeze(0).unsqueeze(0).type(torch.FloatTensor).to(self.device, non_blocking=True) #modified
        return torch.from_numpy(img).unsqueeze(0).unsqueeze(0).type(torch.FloatTensor).to(self.device, non_blocking=True)


    def preprocess(self, img):
        # new size: 170 x 146
        img = img[25:195, 8:152, :]
        img = img[:, :, 0] * 0.299 + img[:, :, 1] * 0.587 + img[:, :, 2] * 0.114
        img = np.array(Image.fromarray(img).resize((72, 85)))
        return torch.from_numpy(img/255.).unsqueeze(0).unsqueeze(0).type(torch.FloatTensor).to(self.device, non_blocking=True)



    def selectAction(self, state):
        sample = random.random()
        eps_threshold = self.eps_start - (self.eps_start - self.eps_end)/self.decay_steps*self.steps_done
        # eps_threshold = self.eps_end + (self.eps_start - self.eps_end) * \
        #     math.exp(-1. * self.steps_done / self.eps_decay)
        if sample > eps_threshold:
            with torch.no_grad():
                return self.policy_net(state).max(1)[1].view(1, 1).type(torch.LongTensor).to(self.device, non_blocking=True)
        else:
            return torch.tensor([[random.randint(0, 2)]], dtype=torch.long).to(self.device, non_blocking=True)



    def optimizeModel(self):
        if len(self.memory) < self.batch_size * self.steps_before_optimize:
            return
        transitions = self.memory.sample(self.batch_size * self.steps_before_optimize)
        batch = self.transition(*zip(*transitions))

        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                              batch.next_state)), device=self.device, dtype=torch.uint8)
        non_final_next_states = torch.cat([s for s in batch.next_state
                                                  if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        state_action_values = self.policy_net(state_batch).gather(1, action_batch)
        # import pdb; pdb.set_trace()
        next_state_values = torch.zeros(self.batch_size*self.steps_before_optimize, device=self.device)
        # import pdb; pdb.set_trace()
        next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].detach()
        expected_state_action_values = (next_state_values * self.gamma) + reward_batch

        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))

        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()


    def SARS(self, state):
        action = self.selectAction(state)
        # action[0][0] = self.VALID[action]
        action_number = [0, 2, 3][action]
        next_state, reward, done, _ = self.env.step(action_number)
        reward = torch.tensor([reward], device=self.device)
        if not done:
            next_state = self.preprocess(next_state)
        else:
            next_state = None
        self.memory.push(state, action, next_state, reward)
        self.steps_done += 1
        return next_state, done


    def saveModel(self, i_episode):
        try:
            torch.save(self.target_net, 'pong_%s.pth' % (i_episode))
            completionEmail('Pong %s episodes completed' % i_episode)
        except Exception as e:
            completionEmail('ERROR IN PONG %s' % i_episode)
            import pdb; pdb.set_trace()

    def getScore(self):
        current_screen = self.preprocess(self.env.reset())
        done = False
        score = 0
        while not done:
            action = self.target_net(current_screen).max(1)[1].view(1, 1).type(torch.LongTensor)
            action_number = self.VALID[action]
            current_screen, reward, done, _ = self.env.step(action_number)
            current_screen = self.preprocess(current_screen)
            score += reward
        return score



    def train(self):
        self.steps_done = 0
        for i_episode in range(self.num_episodes+1):
            start = datetime.datetime.now()
            print("Beginning Episode: ", i_episode)
            state = self.preprocess(self.env.reset())
            done = False
            while not done:
                state, done = self.SARS(state)
                self.optimizeModel()
                if self.steps_done % self.target_update == 0:
                    self.target_net.load_state_dict(self.policy_net.state_dict())
            print('Score at end of epoch %s: %s' % (i_episode, self.getScore()))
            if i_episode % 100 == 0:
                self.saveModel(i_episode)
            print("Time Elapsed: ", (datetime.datetime.now() - start).total_seconds())
            score = self.getScore()
            print("Score for Episode %s: %s" % (i_episode, score))
            self.writer.writerow([score])


    def playback(self, target_net_path):
        self.target_net = torch.load(target_net_path, map_location='cpu')
        self.env = gym.make('Pong-v0').unwrapped
        current_screen = self.preprocess(self.env.reset())
        steps_done = 0
        done = False
        total_reward = 0
        while not done:
            self.env.render(mode='human')
            time.sleep(0.01)
            action = self.target_net(current_screen).max(1)[1].view(1, 1).type(torch.LongTensor)
            action_number = [0,2,3][action]
            current_screen, reward, done, _ = self.env.step(action_number)
            current_screen = self.preprocess(current_screen)
            steps_done += 1
            total_reward += reward
        print("Steps Done: ", steps_done)
        print(total_reward)


def completionEmail(message=''):
  yag = yagmail.SMTP('infolab.rl.bot@gmail.com', 'baxter!@')
  yag.send('julian.a.alverio@gmail.com', 'Training Completed', [message])


trainer = Trainer(num_episodes=NUM_EPISODES)
print("Trainer Initialized")
# trainer.train()
# completionEmail('%s done' % NUM_EPISODES)
trainer.playback('pong_1000.pth')