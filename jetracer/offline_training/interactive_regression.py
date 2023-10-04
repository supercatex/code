import torchvision.transforms as transforms
from xy_dataset import XYDataset
import torch
import torchvision
import torch.utils.data


TASK = 'road_following'
CATEGORIES = ['apex']
DATASETS = ['A', 'B']
MODEL_NAME = "my_model.pt"

EPOCHS = 50
BATCH_SIZE = 8
LEARNING_RATE = 1e-4


TRANSFORMS = transforms.Compose([
    transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

datasets = {}
for name in DATASETS:
    datasets[name] = XYDataset(TASK + '_' + name, CATEGORIES, TRANSFORMS, random_hflip=True)


dataset = datasets[DATASETS[0]]
device = torch.device('cuda')
output_dim = 2 * len(dataset.categories)  # x, y coordinate for each category
model = torchvision.models.resnet18(pretrained=True)
model.fc = torch.nn.Linear(512, output_dim)

model = model.to(device)
# model.load_state_dict(torch.load(MODEL_NAME))  # using existed model.

optimizer = torch.optim.Adam(model.parameters())


def train_eval(is_training):
    global EPOCHS, BATCH_SIZE, LEARNING_RATE, model, dataset, optimizer

    try:
        train_loader = torch.utils.data.DataLoader(
            dataset,
            batch_size=BATCH_SIZE,
            shuffle=True
        )

        if is_training:
            model = model.train()
        else:
            model = model.eval()

        epoch = 0
        while epoch < EPOCHS:
            i = 0
            sum_loss = 0.0
            for images, category_idx, xy in iter(train_loader):
                # send data to device
                images = images.to(device)
                xy = xy.to(device)

                if is_training:
                    # zero gradients of parameters
                    optimizer.zero_grad()

                # execute model to get outputs
                outputs = model(images)

                # compute MSE loss over x, y coordinates for associated categories
                loss = torch.tensor(0.0)
                for batch_idx, cat_idx in enumerate(list(category_idx.flatten())):
                    loss += torch.mean((outputs[batch_idx][2 * cat_idx:2 * cat_idx + 2] - xy[batch_idx]) ** 2)
                loss /= len(category_idx)

                if is_training:
                    # run backpropagation to accumulate gradients
                    loss.backward()

                    # step optimizer to adjust parameters
                    optimizer.step()

                # increment progress
                count = len(category_idx.flatten())
                i += count
                sum_loss += float(loss)

            if is_training:
                epoch += 1
                torch.save(model.state_dict(), MODEL_NAME)
            else:
                break
    except Exception as e:
        print(e)
    model = model.eval()
