#!/usr/bin/env python3
import torch
import torchvision
import torchvision.transforms as T
import torch.nn.functional as F
from dataset import XYDataset

TASK = "face"
CATEGORIES = ["nose", "left_eye", "right_eye"]
DATASETS = ["A", "B"]
TRANSFORMS = T.Compose([
    T.ColorJitter(0.2, 0.2, 0.2, 0.2),
    T.Resize((224, 224)),
    T.ToTensor(),
    T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])
DATA_DIR = "/nvdli-nano/data/regression/"
MODEL_PATH = DATA_DIR + "my_model.pth"

datasets = {}
for name in DATASETS:
    datasets[name] = XYDataset(DATA_DIR + TASK + "_" + name, CATEGORIES, TRANSFORMS)
dataset = datasets[DATASETS[0]]

device = torch.device("cuda")
output_dim = 2 * len(dataset.categories)
model = torchvision.models.resnet18(pretrained=True)
model.fc = torch.nn.Linear(512, output_dim)
model = model.to(device)

BATCH_SIZE = 8
EPOCHES = 20
optimizer = torch.optim.Adam(model.parameters())

train_loader = torch.utils.data.DataLoader(
    dataset,
    batch_size = BATCH_SIZE,
    shuffle = True
)
model.train()
for epoch in range(1, EPOCHES + 1, 1):
    sum_loss = 0
    for x, y, xy in train_loader:
        x = x.to(device)
        xy = xy.to(device)
        
        h = model(x)
        loss = 0
        for i, c in enumerate(list(y.flatten())):
            loss += torch.mean((h[i][2 * c: 2 * c + 2] - xy[i]) ** 2)
        loss /= len(y)
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        sum_loss += loss.detach().cpu().numpy()
    print("Epoch: %d, loss: %.6f" % (epoch, sum_loss))
torch.save(model.state_dict(), MODEL_PATH)
print("END")
