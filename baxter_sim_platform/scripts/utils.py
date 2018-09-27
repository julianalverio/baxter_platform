#!/usr/bin/env python


import random
import numpy as np
import rospkg


##### Utils for DeRRT* Re-implementation #####

'''
Generates noisy data without actually executing a trajectory.
Uses a hard-coded start position
Choises a random end position, which is calculated by hard-coded end position + random noise
Then samples the linearly interpolated trajectory randomly throughout
Writes the output to csv
samples_per = samples per trajectory execution (not actually executed)
'''
def generateData(num_samples=1000, ending_angles_dict=None, samples_per=8):
  #must use rospkg because os.getcwd() gives nondeterministic output for some reason
  path = rospkg.RosPack().get_path('baxter_sim_platform') + '/scripts/noisydata.csv'
  writer = csv.writer(open(path, 'w+'))
  joints = ['left_w0', 'left_w1', 'left_w2', 'left_e0', 'left_e1', 'left_s0', 'left_s1']
  start = {'left_w0': 0.6699952259595108, 'left_w1': 1.030009435085784, 'left_w2': -0.4999997247485215,'left_e0': -1.189968899785275,'left_e1': 1.9400238130755056,'left_s0': -0.08000397926829805,'left_s1': -0.9999781166910306}
  start_angles = np.array([start[joint] for joint in joints])
  if not ending_angles_dict:
    ending_angles_dict = {'left_w0': 0.5956179611450151, 'left_w1': 0.9844394493019387, 'left_w2': -0.41598924558046574, 'left_e0': -1.2763027864959016, 'left_e1': 1.660696693321956, 'left_s0': -0.1343923972459911, 'left_s1': -0.9408210252355302}
  ending_angles = np.array([ending_angles_dict[joint] for joint in joints])
  for _ in xrange(num_samples):
    noise = np.random.uniform(-0.2, 0.2, 7)
    noisy_angles = ending_angles + noise
    deltas = noisy_angles - start_angles
    data_point = [start_angles]
    for sample_location in sorted(np.random.uniform(0, 1, samples_per)):
      sample = start_angles + deltas * sample_location
      data_point.append(sample.tolist())
    data_point.append(noisy_angles)
    writer.writerow(data_point)

'''
Each cell in the csv is a list of angles indicating one position in a trajectory execution
Each row is a trajectory execution, so [eval(point) for point in row] is one trajectory execution
The collection of trajectory executions constitutes the data set, which is what gets returned
Note: must use rospack for path because os.getcwd() gives nondeterministic output with ROS
'''
def getNoisyData(csv_file=rospkg.RosPack().get_path('baxter_sim_platform') + '/scripts/noisydata.csv'):
  reader = csv.reader(open(csv_file, 'r'))
  trajectories = list()
  for row in reader:
    trajectory = []
    for point in row:
      try:
        trajectory.append(eval(point))
      except SyntaxError:
        trajectory.append(eval('[' + re.sub(r'\s+', ',', row[0])[2:]))
    trajectories.append(trajectory)
  return trajectories
