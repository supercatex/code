import torchvision.transforms as transforms
from torchvision.models import ResNet18_Weights

from xy_dataset import XYDataset
import torchvision
import torch.utils.data
import os
import matplotlib.pyplot as plt


TASK = 'D:\\datasets\\jetracer\\road_following'
CATEGORIES = ['apex']
MODEL_PATH = "my_model_200_3"

EPOCHS = 200
BATCH_SIZE = 8
LEARNING_RATE = 1e-6


TRANSFORMS = transforms.Compose([
    transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

dataset = XYDataset(TASK, CATEGORIES, TRANSFORMS, random_hflip=True)
train_data, valid_data = torch.utils.data.random_split(dataset, [0.8, 0.2])

device = torch.device('cuda')
output_dim = 2 * len(dataset.categories)  # x, y coordinate for each category
model = torchvision.models.resnet18(weights=ResNet18_Weights.DEFAULT)
model.fc = torch.nn.Linear(512, output_dim)

model = model.to(device)
print(MODEL_PATH + ".pt", len(train_data), len(valid_data))
if os.path.exists(MODEL_PATH + ".pt"):
    model.load_state_dict(torch.load(MODEL_PATH + ".pt"))  # using existed model.

optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)

try:
    train_loader = torch.utils.data.DataLoader(
        train_data,
        batch_size=BATCH_SIZE,
        shuffle=True
    )
    valid_loader = torch.utils.data.DataLoader(
        valid_data,
        batch_size=BATCH_SIZE,
        shuffle=False
    )

    gy1, gy2 = [], []
    plt.ion()
    epoch = 0
    best_loss = 99999.9
    while epoch < EPOCHS:
        train_loss = 0.0
        model = model.train()
        for images, category_idx, xy in iter(train_loader):
            images = images.to(device)
            xy = xy.to(device)
            optimizer.zero_grad()
            outputs = model(images)

            loss = torch.tensor(0.0).to(device)
            for batch_idx, cat_idx in enumerate(list(category_idx.flatten())):
                loss += torch.mean((outputs[batch_idx][2 * cat_idx:2 * cat_idx + 2] - xy[batch_idx]) ** 2)
            loss /= len(category_idx)
            loss.backward()
            train_loss += float(loss)
            optimizer.step()
        train_loss /= len(train_loader)

        valid_loss = 0.0
        model = model.eval()
        for images, category_idx, xy in iter(valid_loader):
            images = images.to(device)
            xy = xy.to(device)
            outputs = model(images)

            loss = torch.tensor(0.0).to(device)
            for batch_idx, cat_idx in enumerate(list(category_idx.flatten())):
                loss += torch.mean((outputs[batch_idx][2 * cat_idx:2 * cat_idx + 2] - xy[batch_idx]) ** 2)
            loss /= len(category_idx)
            valid_loss += float(loss)
        valid_loss /= len(valid_loader)

        gy1.append(train_loss)
        gy2.append(valid_loss)
        plt.clf()
        plt.plot(gy1)
        plt.plot(gy2)
        plt.draw()
        plt.pause(0.2)
        plt.savefig(MODEL_PATH + ".png")
        print("epoch: %3d\ttrain_loss: %.4f\tvalid_loss: %.4f" % (epoch, train_loss, valid_loss))
        epoch += 1
        torch.save(model.state_dict(), MODEL_PATH + ".pt")
        if valid_loss < best_loss:
            torch.save(model.state_dict(), MODEL_PATH + "_best.pt")
            print("Save the best valid loss model. valid_loss: %.4f" % valid_loss)
            best_loss = valid_loss
    plt.ioff()
except Exception as e:
    print(e)
