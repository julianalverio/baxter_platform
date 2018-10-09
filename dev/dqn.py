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



# # set up matplotlib
# is_ipython = 'inline' in matplotlib.get_backend()
# if is_ipython:
#     from IPython import display

# plt.ion()

# if gpu is to be used
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))


class ReplayMemory(object):

    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = []
        self.position = 0

    def push(self, *args):
        """Saves a transition."""
        if len(self.memory) < self.capacity:
            self.memory.append(None)
        self.memory[self.position] = Transition(*args)
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


# :math:`R_{t_0} = \sum_{t=t_0}^{\infty} \gamma^{t - t_0} r_t`, where
# :math:`R_{t_0}` is also known as the *return*. The discount,
# :math:`\gamma`, should be a constant between :math:`0` and :math:`1`
# that ensures the sum converges. It makes rewards from the uncertain far
# future less important for our agent than the ones in the near future
# that it can be fairly confident about.
#

class DQN(nn.Module):

    def __init__(self):
        super(DQN, self).__init__()
        self.conv1 = nn.Conv2d(3, 16, kernel_size=5, stride=2)
        self.bn1 = nn.BatchNorm2d(16)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=5, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, kernel_size=5, stride=2)
        self.bn3 = nn.BatchNorm2d(32)
        self.head = nn.Linear(448, 2)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        return self.head(x.view(x.size(0), -1))



class screenHandler(object):
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/cameras/head_camera/image', RosImage, self.callback)
    self.most_recent = None
    self.initialized = False
    self.green_x = None
    self.green_y = None

  def getScreen(self):
    if not self.initialized:
      print("screenHandler is not yet initialized. Hanging.")
    while not self.initialized:
      rospy.sleep(1)
    print("Initialized.")
    return self.most_recent

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
      print(e)

    pil_image = Image.fromarray(cv_image)
    width, height = pil_image.size

    cropped = pil_image.crop((0, 300, width, height))
    # cropped.show()
    self.most_recent = cropped
    self.initialized = True
    result = self.findGreenPixels()
    if result == False:
    	print("not enough green pixels found. Resetting")
    else:
      self.green_x, self.green_y = result


  def getReward_slide_right(self):
    width, _ = self.most_recent.size
    if self.green_x > width/2.:
      return 1
    return 0


  def findGreenPixels(self, threshold=20):
    found = False
    while not found:
      x_coords = []
      y_coords = []
      image = self.most_recent
      pixels = image.getdata()
      width, height = image.size
      found = False
      for idx, (r,g,b) in enumerate(pixels):
        x_coord = idx % width
        y_coord = idx // width
        if g>100 and r<50 and b<50:
          found = True
          x_coords.append(x_coord)
          y_coords.append(y_coord)
        if not found:
          rospy.sleep(0.1)
          continue
        if len(x_coords) < 50: return False
        return sum(x_coords)/len(x_coords), sum(y_coords)/len(y_coords) 


  def showGreenPixels(self, threshold=20):
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



# resize = T.Compose([T.ToPILImage(),
#                     T.Resize(40, interpolation=Image.CUBIC),
#                     T.ToTensor()])

# # This is based on the code from gym.
# screen_width = 600




# def get_screen():
#     screen = env.render(mode='rgb_array').transpose(
#         (2, 0, 1))  # transpose into torch order (CHW)
#     # Strip off the top and bottom of the screen
#     screen = screen[:, 160:320]
#     view_width = 320
#     cart_location = get_cart_location()
#     if cart_location < view_width // 2:
#         slice_range = slice(view_width)
#     elif cart_location > (screen_width - view_width // 2):
#         slice_range = slice(-view_width, None)
#     else:
#         slice_range = slice(cart_location - view_width // 2,
#                             cart_location + view_width // 2)
#     # Strip off the edges, so that we have a square image centered on a cart
#     screen = screen[:, :, slice_range]
#     # Convert to float, rescare, convert to torch tensor
#     # (this doesn't require a copy)
#     screen = np.ascontiguousarray(screen, dtype=np.float32) / 255
#     screen = torch.from_numpy(screen)
#     # Resize, and add a batch dimension (BCHW)
#     return resize(screen).unsqueeze(0).to(device)


# env.reset()
# plt.figure()
# plt.imshow(get_screen().cpu().squeeze(0).permute(1, 2, 0).numpy(),
#            interpolation='none')
# plt.title('Example extracted screen')
# plt.show()


######################################################################
# Training
# --------
#
# Hyperparameters and utilities
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# This cell instantiates our model and its optimizer, and defines some
# utilities:
#
# -  ``select_action`` - will select an action accordingly to an epsilon
#    greedy policy. Simply put, we'll sometimes use our model for choosing
#    the action, and sometimes we'll just sample one uniformly. The
#    probability of choosing a random action will start at ``EPS_START``
#    and will decay exponentially towards ``EPS_END``. ``EPS_DECAY``
#    controls the rate of the decay.
# -  ``plot_durations`` - a helper for plotting the durations of episodes,
#    along with an average over the last 100 episodes (the measure used in
#    the official evaluations). The plot will be underneath the cell
#    containing the main training loop, and will update after every
#    episode.
#

BATCH_SIZE = 128
GAMMA = 0.999
EPS_START = 0.9
EPS_END = 0.05
EPS_DECAY = 200
TARGET_UPDATE = 10

policy_net = DQN().to(device)
target_net = DQN().to(device)
target_net.load_state_dict(policy_net.state_dict())
target_net.eval()

optimizer = optim.RMSprop(policy_net.parameters())
memory = ReplayMemory(10000)


steps_done = 0


# reference http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications#Range_of_Motion_-_Bend_Joints
def getRandomState():
  joint_angles = []
  joint_angles_dict = dict()
  # joint_angles.append(random.uniform(-97.4, 97.4)) #s0
  joint_angles.append(random.uniform(-30, 30)) #s0
  joint_angles_dict['left_s0'] = joint_angles[-1]
  joint_angles.append(random.uniform(-123, 60)) #s1
  joint_angles_dict['left_s1'] = joint_angles[-1]
  joint_angles.append(random.uniform(--174, 174)) #e0
  joint_angles_dict['left_e0'] = joint_angles[-1]
  joint_angles.append(random.uniform(-2.8, 150)) #e1
  joint_angles_dict['left_e1'] = joint_angles[-1]
  joint_angles.append(random.uniform(--175, 175)) #w0
  joint_angles_dict['left_w0'] = joint_angles[-1]
  joint_angles.append(random.uniform(-90, 120)) #w1
  joint_angles_dict['left_w1'] = joint_angles[-1]
  joint_angles.append(random.uniform(-175, 175)) #w2
  joint_angles_dict['left_w2'] = joint_angles[-1]
  return joint_angles, joint_angles_dict


def selectAction(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * \
      math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
      with torch.no_grad():
        import pdb; pdb.set_trace()
        return policy_net(state).max(1)[1].view(1, 1)
    else:
      return getRandomState()
      # return torch.tensor(getRandomState(), device=device, dtype=torch.long)


######################################################################
# Training loop
# ^^^^^^^^^^^^^
#
# Finally, the code for training our model.
#
# Here, you can find an ``optimize_model`` function that performs a
# single step of the optimization. It first samples a batch, concatenates
# all the tensors into a single one, computes :math:`Q(s_t, a_t)` and
# :math:`V(s_{t+1}) = \max_a Q(s_{t+1}, a)`, and combines them into our
# loss. By defition we set :math:`V(s) = 0` if :math:`s` is a terminal
# state. We also use a target network to compute :math:`V(s_{t+1})` for
# added stability. The target network has its weights kept frozen most of
# the time, but is updated with the policy network's weights every so often.
# This is usually a set number of steps but we shall use episodes for
# simplicity.
#

def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)
    # Transpose the batch (see http://stackoverflow.com/a/19343/3343043 for
    # detailed explanation).
    batch = Transition(*zip(*transitions))

    # Compute a mask of non-final states and concatenate the batch elements
    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                          batch.next_state)), device=device, dtype=torch.uint8)
    non_final_next_states = torch.cat([s for s in batch.next_state
                                                if s is not None])
    state_batch = torch.cat(batch.state)
    action_batch = torch.cat(batch.action)
    reward_batch = torch.cat(batch.reward)

    # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
    # columns of actions taken
    state_action_values = policy_net(state_batch).gather(1, action_batch)

    # Compute V(s_{t+1}) for all next states.
    next_state_values = torch.zeros(BATCH_SIZE, device=device)
    next_state_values[non_final_mask] = target_net(non_final_next_states).max(1)[0].detach()
    # Compute the expected Q values
    expected_state_action_values = (next_state_values * GAMMA) + reward_batch

    # Compute Huber loss
    loss = F.smooth_l1_loss(state_action_values, expected_state_action_values.unsqueeze(1))

    # Optimize the model
    optimizer.zero_grad()
    loss.backward()
    for param in policy_net.parameters():
        param.grad.data.clamp_(-1, 1)
    optimizer.step()


def resetScene(manager):
  manager.robot_controller.moveToStart(threshold=0.1)
  manager.scene_controller.deleteAllModels()
  manager.scene_controller.makeModel(name='table', shape='box', roll=0., pitch=0., yaw=0., restitution_coeff=0., size_x=.7, size_y=1.5, size_z=.7, x=.8, y=0., z=.35, mass=5000, ambient_r=0.1, ambient_g=0.1, ambient_b=0.1, ambient_a=0.1, mu1=1, mu2=1, reference_frame='')
  manager.scene_controller.makeModel(name='testObject', shape='box', size_x=0.1, size_y=0.1, size_z=0.1, x=0.8, y=0.3, z=0.75, mass=20000, mu1=1000, mu2=2000, restitution_coeff=0.5, roll=0.1, pitch=0.2, yaw=0.3, ambient_r=0, ambient_g=1, ambient_b=0, ambient_a=1, diffuse_r=0, diffuse_g=1, diffuse_b=0, diffuse_a=1)
  manager.scene_controller.spawnAllModels()


TIMEOUT = 5 #seconds
manager = Manager()
print(manager.robot_controller._left_limb.joint_names())
rospy.on_shutdown(manager.shutdown)
screen_handler = screenHandler()
num_episodes = 1000
for i_episode in xrange(num_episodes):
  print "beginning episode: ", i_episode
  # Initialize the environment and state
  start = rospy.Time.now()
  resetScene(manager)
  state = screen_handler.getScreen()
  for t in count():
    # Select and perform an action
    action, action_dict = selectAction(state)
    print("Started moving")
    manager.robot_controller._left_limb.move_to_joint_positions(action_dict, timeout=8., threshold=0.02)
    print("Done moving at time: " + str(rospy.Time.now()))
    reward = screen_handler.getReward_slide_right()

    # Observe new state
    done = reward or (rospy.Time.now() - start > rospy.Duration(TIMEOUT))
    if not reward:
      converter = T.ToTensor()
      next_state = converter(screen_handler.getScreen())
      # next_state = T.ToTensor(screen_handler.getScreen())
    else:
      next_state = None

    reward = torch.tensor([reward], device=device)
    action = torch.tensor([action], device=device)
    # next_state = torch.tensor([next_state], device=device)

    # Store the transition in memory
    memory.push(state, action, next_state, reward)

    # Move to the next state
    state = next_state

    # Perform one step of the optimization (on the target network)
    optimize_model()
    if done:
      break
    # Update the target network
    if i_episode % TARGET_UPDATE == 0:
      target_net.load_state_dict(policy_net.state_dict())

torch.save(policy_net, "policy_net.pth")
torch.save(target_net, "target_net.pth")
