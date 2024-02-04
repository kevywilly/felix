#!/usr/bin/python


import torch
import torch.optim as optim
import torch.nn.functional as F
import torchvision
import torchvision.datasets as datasets
import torchvision.models as models
import torchvision.transforms as transforms
import glob
import PIL.Image
import os
import numpy as np
from felix.config.settings import settings

data_path = settings.Training.planning_path
BEST_MODEL_PATH = os.path.join(settings.Training.best_model_folder,"planning.pth")

def get_xy(path):
    """Gets X and Y from path"""
    _,x,y,w,h,_ = path.split('_')
    w_2 = float(w)/2
    h_2 = float(h)/2
    x = float(x)
    y = float(y)

    x = (x-w_2)/w_2
    y = (y-h_2)/h_2
    return x,y
    

class XYDataset(torch.utils.data.Dataset):

    def __init__(self, directory, random_hflips=False):
        self.directory = directory
        self.random_hflips = random_hflips
        self.image_paths = glob.glob(os.path.join(self.directory, '*.jpg'))
        self.color_jitter = transforms.ColorJitter(0.3, 0.3, 0.3, 0.3)

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        image_path = self.image_paths[idx]
        
        image = PIL.Image.open(image_path)
       
        # width, height = image.size
       
        x,y = get_xy(os.path.basename(image_path))
      
        if float(np.random.rand(1)) > 0.5:
            image = transforms.functional.hflip(image)
            x = -x
        
        image = self.color_jitter(image)
        image = transforms.functional.resize(image, (224, 224))
        image = transforms.functional.to_tensor(image)
        image = image.numpy()[::-1].copy()
        image = torch.from_numpy(image)
        image = transforms.functional.normalize(image, [0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        
        return image, torch.tensor([x, y]).float()


def train():
    dataset = XYDataset(data_path, random_hflips=False)

    test_percent = 0.1
    num_test = int(test_percent * len(dataset))
    train_dataset, test_dataset = torch.utils.data.random_split(dataset, [len(dataset) - num_test, num_test])

    train_loader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=8,
        shuffle=True,
        num_workers=0
    )

    test_loader = torch.utils.data.DataLoader(
        test_dataset,
        batch_size=8,
        shuffle=True,
        num_workers=0
    )

    model = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
    model.fc = torch.nn.Linear(512, 2)
    try:
        model.load_state_dict(torch.load(BEST_MODEL_PATH))
        print("loaded model...")
    except Exception as ex:
        pass
    device = torch.device('cuda')
    model = model.to(device)

    NUM_EPOCHS = 70

    best_loss = 1e9

    optimizer = optim.Adam(model.parameters(), lr=.001)

    for epoch in range(NUM_EPOCHS):
        
        model.train()
        train_loss = 0.0
        for images, labels in iter(train_loader):
            images = images.to(device)
            labels = labels.to(device)
            optimizer.zero_grad()
            outputs = model(images)
            loss = F.mse_loss(outputs, labels)
            train_loss += float(loss)
            loss.backward()
            optimizer.step()
        train_loss /= len(train_loader)
        train_loss /= 8
        
        model.eval()
        test_loss = 0.0
        for images, labels in iter(test_loader):
            images = images.to(device)
            labels = labels.to(device)
            outputs = model(images)
            loss = F.mse_loss(outputs, labels)
            test_loss += float(loss)
        test_loss /= len(test_loader)
        test_loss /= 8
        
        print('%f, %f' % (train_loss, test_loss))
        if test_loss < best_loss:
            torch.save(model.state_dict(), BEST_MODEL_PATH)
            best_loss = test_loss

train()