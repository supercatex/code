#!/usr/bin/env python3
import torch
import torchvision
import torchvision.transforms as T
import torch.nn.functional as F
from dataset import ImageClassificationDataset

TASK = "thumbs"
CATEGORIES = ["thumbs_up", "thumbs_down"]
DATASETS = ["A", "B"]
TRANSFORMS = T.Compose([
    T.ColorJitter(0.2, 0.2, 0.2, 0.2),
    T.Resize((224, 224)),
    T.ToTensor(),
    T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])
DATA_DIR = "/nvdli-nano/data/classification/"
MODEL_PATH = DATA_DIR + "my_model.pth"

datasets = {}
for name in DATASETS:
    datasets[name] = ImageClassificationDataset(DATA_DIR + TASK + "_" + name, CATEGORIES, TRANSFORMS)
dataset = datasets[DATASETS[0]]

device = torch.device("cuda")
model = torchvision.models.resnet18(pretrained=True)
model.fc = torch.nn.Linear(512, len(dataset.categories))
model = model.to(device)

BATCH_SIZE = 8
EPOCHES = 10
optimizer = torch.optim.Adam(model.parameters())

train_loader = torch.utils.data.DataLoader(
    dataset,
    batch_size=BATCH_SIZE,
    shuffle=True
)
model.train()
for epoch in range(1, EPOCHES + 1, 1):
    sum_loss = 0
    for x, y in train_loader:
        x = x.to(device)
        y = y.to(device)
        
        h = model(x)
        loss = F.cross_entropy(h, y)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        sum_loss += loss.detach().cpu().numpy()
    print("Epoch: %d, loss: %.6f" % (epoch, sum_loss))
torch.save(model.state_dict(), MODEL_PATH)
print("END")
        
