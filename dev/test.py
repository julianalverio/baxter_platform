import torchvision
from PIL import Image
import PIL
import torchvision.transforms as T
import torch.nn as nn
import torch.nn.functional as F



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
        import pdb; pdb.set_trace()
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        return self.head(x.view(x.size(0), -1))





img = Image.open('robot_image.jpg')
converter = T.ToTensor()
resize = T.Resize(40, interpolation=Image.CUBIC)
import pdb; pdb.set_trace()
converted = converter(resize(img)).unsqueeze(0)

nn = DQN()
nn.forward(converted)


