# -*- coding: utf-8 -*-

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

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge, CvBridgeError

from scene_generator import *





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
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)
        self.head = None
        # self.head = nn.Linear(448, 7)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        if self.head == None:
            input_dim = x.size(0)*x.size(1)*x.size(2)*x.size(3)
            self.head = nn.Linear(input_dim, 8)
        return self.head(x.view(x.size(0), -1))



class screenHandler(object):
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/cameras/camera_0/image', RosImage, self.callback)
    self.most_recent = None
    self.initialized = False
    self.green_x = None
    self.green_y = None
    self.updated = True

  def getScreen(self):
    if not self.initialized:
      print("screenHandler is not yet initialized. Hanging.")
    while not self.initialized:
      rospy.sleep(1)
    print("Grabbed most recent screen.")
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
    self.green_x, self.green_y = self.findGreenPixels()
    self.updated = True


  def getReward_slide_right(self):
    width, _ = self.most_recent.size
    if self.green_x <= width/2.:
      return 1
    return 0


  def findGreenPixels(self):
    found = False
    while not found:
      x_coords = []
      y_coords = []
      image = self.most_recent
      pixels = image.getdata()
      width, height = image.size
      for idx, (r,g,b) in enumerate(pixels):
        x_coord = idx % width
        y_coord = idx // width
        if g>100 and r<50 and b<50:
          found = True
          x_coords.append(x_coord)
          y_coords.append(y_coord)
      if len(x_coords) < 50: return None, None
      return sum(x_coords)/len(x_coords), sum(y_coords)/len(y_coords)


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



# reference http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications#Range_of_Motion_-_Bend_Joints
def getRandomState():
  joint_angles = []
  joint_angles_dict = dict()
  # joint_angles.append(random.uniform(-97.4, 97.4)) #s0
  joint_angles.append(random.uniform(-30, 30)) #s0
  joint_angles.append(random.uniform(-123, 60)) #s1
  joint_angles.append(random.uniform(--174, 174)) #e0
  joint_angles.append(random.uniform(-2.8, 150)) #e1
  joint_angles.append(random.uniform(--175, 175)) #w0
  joint_angles.append(random.uniform(-90, 120)) #w1
  joint_angles.append(random.uniform(-175, 175)) #w2
  joint_angles.append(random.random()) # gripper. 0 = close, 1 = open
  return torch.from_numpy(np.array(joint_angles)).view(1, 8)



class Trainer(object):
    # interpolation can be NEAREST, BILINEAR, BICUBIC, or LANCZOS
    def __init__(self, interpolation=Image.BILINEAR, batch_size=128, gamma=0.999, eps_start=0.9, eps_end=0.05,
                 eps_decay=200, target_update=10, replay_memory_size=10000, timeout=5, num_episodes=1000, resize=40,
                 one_move_timeout=4., move_precision=0.02, ):
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
        import pdb; pdb.set_trace()
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        self.optimizer = optim.RMSprop(self.policy_net.parameters())
        self.memory = ReplayMemory(replay_memory_size, self.Transition)

        self.steps_done = 0

        self.TIMEOUT = timeout  # in seconds
        self.manager = Manager()
        rospy.on_shutdown(self.manager.shutdown)
        self.screen_handler = screenHandler()
        self.num_episodes = num_episodes
        self.one_move_timeout = one_move_timeout
        self.move_precision = move_precision

    def resetScene(self, sleep=False):
        self.manager.scene_controller.deleteAllModels(cameras=False)
        self.manager.scene_controller.makeModel(name='table', shape='box', roll=0., pitch=0., yaw=0.,
                                                restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.8, y=0.,
                                                z=.35, mass=5000, ambient_r=0.1, ambient_g=0.1, ambient_b=0.1,
                                                ambient_a=0.1, mu1=1, mu2=1, reference_frame='')
        self.manager.scene_controller.makeModel(name='testObject', shape='box', size_x=0.1, size_y=0.1, size_z=0.1,
                                                x=0.8, y=0.1, z=0.75, mass=20000, mu1=1000, mu2=2000,
                                                restitution_coeff=0.5, roll=0.1, pitch=0.2, yaw=0.3, ambient_r=0,
                                                ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1,
                                                diffuse_b=0, diffuse_a=1)
        self.manager.scene_controller.spawnAllModels()
        self.manager.robot_controller.moveToStart(threshold=0.1)
        self.screen_handler.most_recent = None
        if sleep:
            rospy.sleep(1.)
        while not self.screen_handler.most_recent:
          print('Waiting for scene to re-render')
          rospy.sleep(0.1)
        image = self.screen_handler.most_recent
        pixels = image.getdata()
        green_pixels = 0
        for idx, (r, g, b) in enumerate(pixels):
            if g > 100 and r < 50 and b < 50:
              green_pixels += 1
        if green_pixels < 50:
          print("I did not find enough green pixels.")
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
        if sample > eps_threshold:
          with torch.no_grad():
            return self.policy_net(state).view(1, 8)
        else:
          return getRandomState()


    # This first samples a batch, concatenates
    # all the tensors into a single one, computes :math:`Q(s_t, a_t)` and
    # :math:`V(s_{t+1}) = \max_a Q(s_{t+1}, a)`, and combines them into our
    # loss. By defition we set :math:`V(s) = 0` if :math:`s` is a terminal
    # state. We also use a target network to compute :math:`V(s_{t+1})` for
    # added stability. The target network has its weights kept frozen most of
    # the time, but is updated with the policy network's weights every so often.
    # This is usually a set number of steps but we shall use episodes for
    # simplicity.
    #

    def optimize_model(self):
        if len(self.memory) < self.BATCH_SIZE:
            return
        transitions = self.memory.sample(self.BATCH_SIZE)
        # Transpose the batch (see http://stackoverflow.com/a/19343/3343043 for
        # detailed explanation).
        batch = self.Transition(*zip(*transitions))

        # Compute a mask of non-final states and concatenate the batch elements
        non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                              batch.next_state)), device=self.device, dtype=torch.uint8)
        non_final_next_states = torch.cat([s for s in batch.next_state
                                                    if s is not None])
        state_batch = torch.cat(batch.state)
        action_batch = torch.cat(batch.action)
        reward_batch = torch.cat(batch.reward)

        # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
        # columns of actions taken
        state_action_values = self.policy_net(state_batch).gather(1, action_batch)

        # Compute V(s_{t+1}) for all next states.
        next_state_values = torch.zeros(self.BATCH_SIZE, device=self.device)
        next_state_values[non_final_mask] =self.target_net(non_final_next_states).max(1)[0].detach()
        # Compute the expected Q values
        expected_state_action_values = (next_state_values * self.GAMMA) + reward_batch

        # Compute Huber loss
        loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))

        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        for param in self.policy_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()


    def train(self):
        self.manager.scene_controller.externalCamera(quat_x=0., quat_y=0., quat_z=1., quat_w=0., x=1.7, y=0., z=1.)
        for i_episode in xrange(self.num_episodes):
          print "Beginning episode: ", i_episode
          # Initialize the environment and state
          self.resetScene(self.manager)
          state = self.preprocess(self.screen_handler.getScreen()).unsqueeze(0).to(self.device)
          start = rospy.Time.now()
          for _ in count():
            # Select and perform an action
            action_tensor = self.selectAction(state)
            action_list = np.array(action_tensor).tolist()[0]
            angles_list = action_list[:-1]
            print "unrounded gripper action", action_list[-1]
            gripper_action = int(round(action_list[-1]))
            print 'gripper action:', gripper_action
            joints = self.manager.robot_controller.getJointNames()
            angles_dict = dict(zip(joints, angles_list))
            print("Started moving")
            if gripper_action == 1:
              self.manager.robot_controller.gripperOpen()
            elif gripper_action == 0:
              self.manager.robot_controller.gripperClose()
            else:
              assert False, "invalid gripper command"
            self.manager.robot_controller._left_limb.move_to_joint_positions(angles_dict, timeout=self.one_move_timeout, threshold=self.move_precision)
            print("Done moving")

            reward = self.screen_handler.getReward_slide_right()

            # Observe new state
            done = reward or (rospy.Time.now() - start > rospy.Duration(self.TIMEOUT))
            if not reward:
              next_state = self.preprocess(self.screen_handler.getScreen()).unsqueeze(0).to(self.device)
            else:
              next_state = None

            reward = torch.tensor([reward], device=self.device)

            # Store the transition in memory
            self.memory.push(state, action_tensor, next_state, reward)

            # Move to the next state
            state = next_state

            # Perform one step of the optimization (on the target network)
            self.optimize_model()
            if done:
              break
            # Update the target network
            if i_episode % self.TARGET_UPDATE == 0:
              self.target_net.load_state_dict(self.policy_net.state_dict())

        torch.save(self.policy_net, "policy_net.pth")
        torch.save(self.target_net, "target_net.pth")


trainer = Trainer()
trainer.train()
