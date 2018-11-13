"""
Agent is something which converts states into actions and has state
"""
import torch
torch.backends.cudnn.deterministic = True
torch.manual_seed(5)
torch.cuda.manual_seed_all(5)
import random
random.seed(5)
import numpy as np
np.random.seed(5)

import copy
import numpy as np
import torch
import torch.nn.functional as F

from . import actions


class BaseAgent:
    def __call__(self, states, agent_states):
        assert isinstance(states, list)
        assert isinstance(agent_states, list)
        assert len(agent_states) == len(states)

        raise NotImplementedError


class DQNAgent(BaseAgent):
    def __init__(self, dqn_model, action_selector, device="cpu"):
        self.dqn_model = dqn_model
        self.action_selector = action_selector
        self.device = device

    def __call__(self, states):
        states = torch.tensor(np.expand_dims(states[0], 0))
        if torch.is_tensor(states):
            states = states.to(self.device)
        q_v = self.dqn_model(states)
        q = q_v.data.cpu().numpy()
        actions = self.action_selector(q)
        return actions


class TargetNet:
    def __init__(self, model):
        self.model = model
        self.target_model = copy.deepcopy(model)

    def sync(self):
        self.target_model.load_state_dict(self.model.state_dict())

    def save(self, name):
        torch.save(self.target_model, name)




