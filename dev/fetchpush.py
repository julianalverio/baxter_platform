import sys
import os
sys.path.insert(0, os.getcwd() + '/gym')
import gym
import math
import random
import numpy as np
from collections import namedtuple
from itertools import count
from PIL import Image

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import yagmail
import os
import datetime


NUM_EPISODES = 2000
os.environ['CUDA_VISIBLE_DEVICES']='1,2,3'

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

    def __init__(self, num_actions, device, setupState):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)
        setupState = F.relu(self.bn1(self.conv1(setupState)))
        setupState = F.relu(self.bn2(self.conv2(setupState)))
        setupState = F.relu(self.bn3(self.conv3(setupState)))
        self.head = nn.Linear(np.prod(setupState.size()), num_actions)
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
        setupState = self.getScreen().to(torch.device("cpu"))
        self.policy_net = DQN(6, self.device, setupState).to(self.device)
        self.target_net = DQN(6, self.device, setupState).to(self.device)

        self.transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))

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

        self.num_episodes = num_episodes
        self.steps_done = 0


    # Grab and image, crop it, downsample and resize, then convert to tensor
    def getScreen(self):
        screen = Image.fromarray(self.env.render(mode='rgb_array')).crop((30, 100, 450, 425)).resize((105, 81), Image.NEAREST)
        return torch.from_numpy(np.array(screen, dtype=np.float32).transpose((2, 1, 0))).unsqueeze(0).to(self.device)


    def selectAction(self, state):
        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
            math.exp(-1. * self.steps_done / self.EPS_DECAY)
        if sample > eps_threshold:
            with torch.no_grad():
                return self.policy_net(state).max(1)[1]
        else:
            return random.randrange(0, 6)



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
        next_state_values[non_final_mask] = self.target_net(non_final_next_states).max(1)[0].view(-1).detach()
        expected_state_action_values = (next_state_values.view(-1, 1) * self.GAMMA) + reward_batch

        loss = F.smooth_l1_loss(state_action_values.unsqueeze(0), expected_state_action_values.unsqueeze(0))
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
        self.env.sim.nsubsteps = 3
        self.env.block_gripper = True
        self.getScreen()


    '''
    Task 1: Touch the block
    Task 2: Move the block in the positive y direction
    Task 3: Move the block in positive y until it falls
    Task 4: Pick up the block
    Task 5: Pick up block and move to location
    '''
    def getReward(self, task=1):
        gripper_position = self.env.sim.data.get_site_xpos('robot0:grip')
        object_position = self.env.sim.data.get_site_xpos('object0')
        reward = 0
        if task == 1:
            reward += 1./np.linalg.norm(gripper_position - object_position)
        print(reward)
        return torch.tensor(reward, device=self.device).view(1, 1)


    # initial_block_position = self.env.sim.data.get_site_xpos('object0')
    # initial_block_joint_position = self.env.sim.data.get_site_xpos('object0:joint')
    # initial_gripper_position = self.env.sim.data.get_site_xpos('robot0:grip')
    # initial_object_qpos = self.env.sim.data.get_joint_qpos('object0:joint')
    # initial_site_poses = self.env.sim.model.site_pos
    #to see all "joint names" look at self.env.sim.model.joint_names --> only object0:joint doesn't have 'robot' as a prefix
    #last thing I ran: (Pdb) [x for x in dir(self.env.sim.data) if 'get' in x and 'joint' in x]
    def train(self):
        gripper_position = self.env.sim.data.get_site_xpos('robot0:grip')
        object_position = self.env.sim.data.get_site_xpos('object0')
        for i_episode in range(self.num_episodes):
            print('Episode %s' % i_episode)
            start = datetime.datetime.now()
            self.reset()
            self.steps_done = 0
            for t in count():
                # print(np.linalg.norm(self.env.sim.data.get_site_xpos('robot0:grip') - gripper_position))
                # print(np.linalg.norm(self.env.sim.data.get_site_xpos('object0') - object_position))
                done = False
                state = self.getScreen()
                action = torch.tensor(self.selectAction(state), device=self.device).view(1, 1)
                movement = np.zeros(4)
                if action.item() % 2 == 0:
                    movement[action.item() // 2] += 1
                else:
                    movement[action.item() // 2] -= 1
                self.env.step(movement)
                reward = self.getReward(task=1)

                if t == 1000:
                    done = True
                if not done:
                    next_state = self.getScreen()
                else:
                    next_state = None

                self.memory.push(state, action, next_state, reward)
                self.optimizeModel()
                if done:
                    duration = (datetime.datetime.now() - start).total_seconds()
                    print('Episode Duration: %s seconds ' % duration)
                    break
            if i_episode % self.TARGET_UPDATE == 0:
                self.target_net.load_state_dict(self.policy_net.state_dict())
            if i_episode % 250 == 0:
                try:
                    torch.save(self.target_net, 'fetchpush_%s.pth' % i_episode)
                    completionEmail('SUCCESS OPENAI GYM')
                except Exception as e:
                    completionEmail('ERROR IN OPENAI GYM')
                    import pdb; pdb.set_trace()


    def playback(self, target_net_path):
        self.target_net = torch.load(target_net_path, map_location='cpu')
        self.env = gym.make('FetchPush-v1').unwrapped
        self.env.reset()
        steps_done = 0
        current_screen = self.getScreen()
        for _ in range(1000):
            self.env.render(mode='human')
            previous_screen = current_screen
            current_screen = self.getScreen()
            action = self.target_net(state).max(1)[1].view(1, 1).type(torch.LongTensor)
            _, reward, done, _ = self.env.step(action.item())
            steps_done += 1
        print("Steps Done: ", steps_done)



def completionEmail(message=''):
  yag = yagmail.SMTP('infolab.rl.bot@gmail.com', 'baxter!@')
  yag.send('julian.a.alverio@gmail.com', 'Training Completed', [message])


trainer = Trainer(num_episodes=NUM_EPISODES)
print("Trainer Initialized")
trainer.train()
completionEmail('%s done' % NUM_EPISODES)
trainer.playback('fetchpush_1000.pth')
