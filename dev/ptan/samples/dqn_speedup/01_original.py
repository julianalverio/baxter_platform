#!/usr/bin/env python3
import torch
torch.backends.cudnn.deterministic = True
torch.manual_seed(5)
torch.cuda.manual_seed_all(5)
import random
random.seed(5)
import numpy as np
np.random.seed(5)

import gym
import argparse

import torch
import torch.optim as optim

from tensorboardX import SummaryWriter

from lib import dqn_model, common
from other import actions, agent, experience
import other
import csv

import os; os.environ["CUDA_VISIBLE_DEVICES"]="1"


if __name__ == "__main__":
    params = common.HYPERPARAMS['pong']
    parser = argparse.ArgumentParser()
    parser.add_argument("--cuda", default=True, action="store_true", help="Enable cuda")
    args = parser.parse_args()
    device = torch.device("cuda" if args.cuda else "cpu")

    env = gym.make(params['env_name'])
    env = other.common.wrappers.wrap_dqn(env)

    writer = SummaryWriter(comment="-" + params['run_name'] + "-01_original")
    net = dqn_model.DQN(env.observation_space.shape, env.action_space.n).to(device)

    tgt_net = agent.TargetNet(net)
    selector = actions.EpsilonGreedyActionSelector(epsilon=params['epsilon_start'])
    epsilon_tracker = common.EpsilonTracker(selector, params)
    agent = agent.DQNAgent(net, selector, device=device)

    exp_source = experience.ExperienceSourceFirstLast(env, agent, gamma=params['gamma'], steps_count=1)
    buffer = experience.ExperienceReplayBuffer(exp_source, buffer_size=params['replay_size'])
    optimizer = optim.Adam(net.parameters(), lr=params['learning_rate'])
    csv_file = open('losses.csv', 'r')
    csv_reader = csv.reader(csv_file)

    losses = []
    for row in csv_reader:
        losses.append(row[0])

    frame_idx = 0
    counter = 0
    reward_tracker = common.RewardTracker()
    while True:
        frame_idx += 1
        buffer.populate(1)
        epsilon_tracker.frame(frame_idx)

        new_rewards = exp_source.pop_total_rewards()
        if new_rewards:
            done = reward_tracker.add(new_rewards[0])
            if done:
                break

        if len(buffer) < params['replay_initial']:
            continue

        optimizer.zero_grad()
        batch = buffer.sample(params['batch_size'])
        loss_v = common.calc_loss_dqn(batch, net, tgt_net.target_model, gamma=params['gamma'], cuda=args.cuda)
        if loss_v.item() != float(losses[counter]):
            print('FAILURE')
            import pdb; pdb.set_trace()
        counter += 1
        if counter == 3000:
            print('ALL GOOD')
            break
        loss_v.backward()
        optimizer.step()

        if frame_idx % params['target_net_sync'] == 0:
            tgt_net.sync()
