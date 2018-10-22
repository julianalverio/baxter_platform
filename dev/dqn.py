# -*- coding: utf-8 -*-

import math
import random
import numpy as np
import matplotlib
# import matplotlib.pyplot as plt
from collections import namedtuple
from itertools import count
from PIL import Image

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torchvision.transforms as T

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError

from scene_generator import *
import yagmail
import traceback
import genpy
from PIL import ImageChops



NUM_EPISODES=5000


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

    def __init__(self):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(4, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)
        self.head = nn.Linear(31976, 16)

    def forward(self, x_input):
        robot_state = x_input[:,-1,0,0:8]
        x = x_input[:, 0:-1, :, :]
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        concatenated = torch.cat([x.view(x.size(0), -1),  robot_state], dim=1)
        return self.head(concatenated)



class screenHandler(object):
  def __init__(self, task):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/cameras/camera_0/image', RosImage, self.callback)
    self.most_recent = None
    self.initialized = False
    self.green_x = None
    self.green_y = None
    self.blue_x = None
    self.blue_y = None
    self.updated = True
    self.task = task

  def getScreen(self):
    if not self.initialized:
      print("screenHandler is not yet initialized. Hanging.")
    while not self.initialized:
      rospy.sleep(1)
    return self.most_recent

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)

    pil_image = Image.fromarray(cv_image)
    width, height = pil_image.size

    # cropped = pil_image.crop((0, 300, width, height))
    # cropped.show()
    self.most_recent = pil_image
    self.initialized = True
    self.findColorPixels()
    if self.task == 2:
        self.blue_x, self.blue_y = self.findBluePixels()
    self.updated = True



  def getReward(self, out_of_bounds, redundant_grip, no_movement):
    reward = 0.
    if out_of_bounds:
        reward -= 1.
    if redundant_grip:
        reward -= 1.
    if no_movement:
        reward -= 1.
    if self.task == 1:
        width, _ = self.most_recent.size
        if self.green_x <= width/2.:
          reward += 1000.
    if self.task == 2:
        if self.green_x < self.blue_x:
            reward += 1000.
    return reward

  def getNeighbors(self, x, y):
    neighbors  = []
    neighbors.append([x+1, y+1])
    neighbors.append([x+1, y])
    neighbors.append([x+1, y-1])
    neighbors.append([x, y+1])
    neighbors.append([x, y-1])
    neighbors.append([x-1, y+1])
    neighbors.append([x-1, y])
    neighbors.append([x-1, y-1])
    return neighbors


  # find all the green and blue blocks' pixels. If the block are touching, the pixels will be contiguous
  # which DFS will find
  def checkContiguous(self, pixels):
    queue = pixels[0]
    found = set(pixels[0])
    while queue:
        x, y = queue.pop()
        children = [pixel for pixel in self.getNeighbors(x,y) if pixel in pixels and pixel not in found]
        queue.extend(children)
        for child in children:
            found.add(child)
    return len(found) == len(pixels)



  def findColorPixels(self):
    found = False
    while not found:
        green_x_coords = []
        green_y_coords = []
        blue_x_coords = []
        blue_y_coords = []
        image = self.most_recent
        if not image:
            return
        pixels = image.getdata()
        width, height = image.size
        for idx, (r,g,b) in enumerate(pixels):
            x_coord = idx % width
            y_coord = idx // width
            if g>100 and r<50 and b<50:
                green_x_coords.append(x_coord)
            green_y_coords.append(y_coord)
            if self.task > 1:
                if b>100 and r<50 and g<50:
                    blue_x_coords.append(x_coord)
                    blue_y_coords.append(y_coord)

        if not sum(green_x_coords):
            self.green_x = None
            self.green_y = None
        else:
            self.green_x = sum(green_x_coords)/len(green_x_coords)
            self.green_y = sum(green_y_coords)/len(green_y_coords)
        if not sum(blue_x_coords):
            self.blue_x = None
            self.blue_y = None
        else:
            self.blue_x = sum(blue_x_coords)/len(blue_x_coords)
            self.blue_y = sum(blue_y_coords)/len(blue_y_coords)
        if self.task == 3:
            pixels = set()
            image = self.most_recent
            image_pixels = self.most_recent.getdata()
            width, height = image.size
            for idx, (r,g,b) in enumerate(image_pixels):
                x_coord = idx % width
                y_coord = idx // width
                if (g>100 and r<50 and b<50) or (b>100 and r<50 and g<50):
                    pixels.add([x_coord, y_coord])
            if self.checkContiguous(pixels) and (self.green_y > self.blue_y) and (abs(self.green_x - self.blue_x) < 20):
                return 1000.
            

  def showGreenPixels(self):
    image = self.most_recent
    newimdata = []
    whitecolor = (255, 255, 255)
    greencolor = (0, 255, 0)
    blackcolor = (0,0,0)
    for color in image.getdata():
      r,g,b = color
      if g > 100 and r<50 and b<50:
        newimdata.append(whitecolor)
      else:
        newimdata.append(blackcolor)
    newim = Image.new(image.mode, image.size)
    newim.putdata(newimdata)
    image.show()
    newim.show()


def completionEmail():
  message = 'Training completed!'
  yag = yagmail.SMTP('infolab.rl.bot@gmail.com', 'baxter!@')
  yag.send('julian.a.alverio@gmail.com', 'Training Completed', [message])



#task=1: slide a green block to the left
#task=2: slide a green block to the left of a blue block
#task=3: stack a green block on top of a blue block
class Trainer(object):
    # interpolation can be NEAREST, BILINEAR, BICUBIC, or LANCZOS
    def __init__(self, interpolation=Image.BILINEAR, batch_size=64, gamma=0.999, eps_start=0.9, eps_end=0.05,
                 eps_decay=200, target_update=10, replay_memory_size=1000, timeout=5, num_episodes=1000, resize=40,
                 one_move_timeout=1., move_precision=0.02, count_timeout=100, movement_threshold=0.005, task=1):
        self.params_dict = {
        'interpolation' : interpolation,
        'batch_size' : batch_size,
        'gamma' : gamma,
        'eps_start' : eps_start,
        'eps_end' : eps_end,
        'eps_decay' : eps_decay,
        'target_update' : target_update,
        'replay_memory_size' : replay_memory_size,
        'timeout' : timeout,
        'num_episodes' : num_episodes,
        'resize' : resize,
        'one_move_timeout' : one_move_timeout,
        'move_precision' : move_precision
        }
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.Transition = namedtuple('Transition',
                                     ('state', 'action', 'next_state', 'reward'))

        self.preprocess = T.Compose([T.Resize(resize, interpolation=interpolation),
                                     T.ToTensor()])

        self.BATCH_SIZE = batch_size
        self.GAMMA = gamma
        self.EPS_START = eps_start
        self.EPS_END = eps_end
        self.EPS_DECAY = eps_decay
        self.TARGET_UPDATE = target_update

        self.policy_net = DQN().to(self.device)
        self.target_net = DQN().to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.RMSprop(self.policy_net.parameters())
        self.memory = ReplayMemory(replay_memory_size, self.Transition)

        self.steps_done = 0

        self.TIMEOUT = timeout  # in seconds
        self.manager = Manager()
        rospy.on_shutdown(self.manager.shutdown)
        self.screen_handler = screenHandler(task)
        self.num_episodes = num_episodes
        self.one_move_timeout = one_move_timeout
        self.move_precision = move_precision
        self.out_of_bounds = False
        self.count_timeout = count_timeout
        self.hand_open = True
        self.redundant_grip = False
        # self.log = open('log.txt', 'w+')
        self.no_movement = False
        self.movement_threshold = movement_threshold

    def resetScene(self, sleep=False):
        self.manager.scene_controller.deleteAllModels(cameras=False)
        self.manager.scene_controller.makeModel(name='table', shape='box', roll=0., pitch=0., yaw=0.,
                                                restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.8, y=0.,
                                                z=.35, mass=5000, ambient_r=0.1, ambient_g=0.1, ambient_b=0.1,
                                                ambient_a=0.1, mu1=1, mu2=1, reference_frame='', static=True)
        self.manager.scene_controller.makeModel(name='testObject', shape='box', size_x=0.1, size_y=0.1, size_z=0.1,
                                                x=0.8, y=0.1, z=0.75, mass=1, mu1=1000, mu2=2000,
                                                restitution_coeff=0.5, roll=0.1, pitch=0.2, yaw=0.3, ambient_r=0,
                                                ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1,
                                                diffuse_b=0, diffuse_a=1)
        self.manager.scene_controller.spawnAllModels()
        self.manager.robot_controller.moveToStart(threshold=0.1)
        self.screen_handler.most_recent = None
        if sleep:
            rospy.sleep(1.)
        while not self.screen_handler.most_recent:
          # print('Waiting for scene to re-render')
          rospy.sleep(0.1)
        image = self.screen_handler.most_recent
        pixels = image.getdata()
        green_pixels = 0
        for idx, (r, g, b) in enumerate(pixels):
            if g > 100 and r < 50 and b < 50:
              green_pixels += 1
        if green_pixels < 50:
          print("Re-rending Scene.")
          # image.show()
          self.resetScene(sleep=True)
        # else:
        #   print("I found green pixels!")
          # image.show()



    def selectAction(self, state):
        sample = random.random()
        eps_threshold = self.EPS_END + (self.EPS_START - self.EPS_END) * \
          math.exp(-1. * self.steps_done / self.EPS_DECAY)
        self.steps_done += 1
        if sample < eps_threshold or len(self.memory) < self.BATCH_SIZE:
            return torch.tensor(random.randrange(0, 16), device=self.device).view(1, 1)

        else:
          with torch.no_grad():
            return torch.tensor(self.policy_net(state).max(1)[1], device=self.device).view(1, 1)
        



    #inputs will be [s0+, s0-, s1+, s1-, e0+, e0-, e1+, e1-, w0+, w0-, w1+, w1-, w2+, w2-, open_gripper, close_gripper] == indices
    def performAction(self, action):
        angles_dict = self.manager.robot_controller._left_limb.joint_angles();
        joints = self.manager.robot_controller.getJointNames()
        angles_list = [angles_dict[joint] for joint in joints]
        if action == 14:
            if not self.hand_open:
                self.manager.robot_controller.gripperOpen()
                self.hand_open = True
            else:
                self.redundant_grip = True
        elif action == 15:
            if self.hand_open:
                self.manager.robot_controller.gripperClose()
                self.hand_open = False
            else:
                self.redundant_grip = True
        #if you're not opening/closing the gripper
        else:
            bounded_angles = self.checkBounds(angles_list, action)
            start_angles = self.manager.robot_controller.getJointAngles(numpy=True)
            self.manager.robot_controller.followTrajectoryFromJointAngles([bounded_angles], timeout=self.one_move_timeout)
            end_angles = self.manager.robot_controller.getJointAngles(numpy=True)
            if np.linalg.norm(start_angles - end_angles) < self.movement_threshold:
                self.no_movement = True
                print("NO MOVEMENT!!")



    def checkBounds(self, old_angles, action):
        action_arr = np.zeros(7)
        if action % 2 == 0:
            action_arr[action//2] = 1.
        else:
            action_arr[action//2] = -1.
        angles = np.array(old_angles) + action_arr
        valid = True
        #s0
        if not (-97.4 < angles[0] < 97.4):
            valid = False
        #s1
        if not (-123 < angles[1] < 60):
            valid = False
        #e0
        if not (-174.9 < angles[2] < 174.9):
            valid = False
        #e1
        if not (-2.8 < angles[3] < 150):
            valid = False
        #w0
        if not (-175.2 < angles[4] < 175.2):
            valid = False
        #w1
        if not (-90 < angles[5] < 120):
            valid = False
        #w2
        if not (-175.2 < angles[6] < 175.2):
            valid = False
        if valid:
            return angles.tolist()
        self.out_of_bounds = True
        return old_angles


    def getRobotState(self):
        hand_tensor = torch.tensor(int(self.hand_open)).view(1, -1).type(torch.FloatTensor)
        return torch.cat((torch.tensor(self.manager.robot_controller.getJointAngles()).view(1,-1).type(torch.FloatTensor), hand_tensor), 1).to(self.device)

    def rgb2gray(self, rgb):
        img = Image.fromarray(rgb.transpose((1,2,0))).convert(mode='L')
        return np.array(img, dtype=np.float64)


    def optimizeModel(self):
        if len(self.memory) < self.BATCH_SIZE:
            return
        transitions = self.memory.sample(self.BATCH_SIZE)
        batch = self.Transition(*zip(*transitions))

        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                              batch.next_state)), device=self.device, dtype=torch.uint8)

        non_final_next_states = [s for s in batch.next_state
                                                    if s is not None]

        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
        # columns of actions taken
        state_action_values = self.policy_net(state_batch).gather(1, action_batch)

        # Compute V(s_{t+1}) for all next states.
        next_state_values = torch.zeros(self.BATCH_SIZE, device=self.device)
        next_state_values[non_final_mask] = self.target_net(state_batch).max(1)[0].detach()
        # Compute the expected Q values
        expected_state_action_values = (next_state_values.view(64,1) * self.GAMMA) + reward_batch

        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values)
        print('Loss: %s' % loss.item())

        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()


    def getState(self, previous_screen, current_screen):
        difference = np.array(ImageChops.subtract(current_screen, previous_screen).convert(mode='L'))
        robot_state = self.getRobotState()
        robot_state_2d = np.zeros(difference.shape)
        robot_state_2d[0,:robot_state.size()[1]] = robot_state
        return torch.tensor(np.concatenate((np.array(current_screen).transpose((2,0,1)), np.expand_dims(difference, axis=0), np.expand_dims(robot_state_2d, axis=0)), axis=0)).unsqueeze(0).type(torch.FloatTensor).to(self.device)
            


    def train(self):
        self.manager.scene_controller.externalCamera(quat_x=0., quat_y=0., quat_z=1., quat_w=0., x=1.7, y=0., z=1.)
        for i_episode in xrange(self.num_episodes):
            self.resetScene(self.manager)
            previous_screen = self.screen_handler.getScreen()
            current_screen = self.screen_handler.getScreen()
            state = self.getState(previous_screen, current_screen)
            for movement_idx in count():
                action_tensor = self.selectAction(state)
                print(action_tensor.item())
                print("Episode: %s Action #: %s" % (i_episode, movement_idx))
                self.performAction(action_tensor.item())

                reward = self.screen_handler.getReward(self.out_of_bounds, self.redundant_grip, self.no_movement)
                self.out_of_bounds = False
                self.redundant_grip = False
                self.no_movement = False

                done = (reward > 0) or (movement_idx >= self.count_timeout - 1)

                if reward <= 0:
                    previous_screen = current_screen
                    current_screen = self.screen_handler.getScreen()
                    next_state = self.getState(previous_screen, current_screen)
                else:
                  next_state = None

                reward = torch.tensor(reward, device=self.device).view(1, 1)
                self.memory.push(state, action_tensor, next_state, reward)

                state = next_state

                self.optimizeModel()
                if done:
                  break
                if i_episode % self.TARGET_UPDATE == 0:
                  self.target_net.load_state_dict(self.policy_net.state_dict())

        torch.save(self.target_net.state_dict(), 'target_net_state')
        torch.save(self.target_net, 'target_net')



    def loadModel(path):
      target_net = DQN()
      target_net.load_state_dict(torch.load(path))
      target_net.eval()



pre_grip_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}

trainer = Trainer(num_episodes=NUM_EPISODES)
trainer.train()
completionEmail()


