import rclpy
from rclpy.node import Node
import torch
import torch.optim as optim
import torch.nn.functional as F
import torchvision
import torchvision.datasets as datasets
import torchvision.models as models
import torchvision.transforms as transforms
from felix.common.settings import settings
import os

from src.felix.felix.common.settings import TrainingType

torch.hub.set_dir(settings.Training.model_root)
        
class TrainerNode(Node):

    def log(self, txt: str):
        self.get_logger().info(txt)

    def __init__(self):

        super().__init__("train", parameter_overrides=[])
        
        try:
            os.makedirs(settings.Training.best_model_folder)
        except FileExistsError:
            pass
        except Exception as ex:
            print(ex)
            raise ex
            
        self.log(f"Trainer loaded for: {settings.Training.name}")

        if settings.Training.type == TrainingType.OBSTACLE:
            self.train_obstacle()
        elif settings.Training.type == TrainingType.PATH:
            self.train_path()
    

    def train_path(self):
        self.log("Starting path trainer...")
        self.log("Done")


    def train_obstacle(self):
        
        self.log("Starting obstacle trainer...")

        datafolder = settings.Training.training_data_path
        train_pct: float = 0.6
        learning_rate: float = 0.001
        momentum: float = 0.9

        self.log(f"using path {datafolder} for training data.")

        dataset = datasets.ImageFolder(
        datafolder,
        transforms.Compose([
            transforms.ColorJitter(0.1, 0.1, 0.1, 0.1),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ]))

        datapoints = len(dataset)
        train_size = int(train_pct * datapoints)
        test_size = datapoints - train_size

        self.log(f"found {datapoints} datapoints.")

        train_dataset, test_dataset = torch.utils.data.random_split(dataset, [train_size,test_size])

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

        self.log("loading model")
        cat_count = len(settings.Training.categories)
        if settings.Training.classifier.lower() == "alexnet":
            model = torchvision.models.alexnet(pretrained=True)
            model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, cat_count)
        else:
            model = torchvision.models.resnet18(pretrained=True)
            model.fc = torch.nn.Linear(512, cat_count)
        
        device = torch.device('cuda')
        model = model.to(device)

        print("training model...")

        NUM_EPOCHS = 30
        BEST_MODEL_PATH = settings.Training.best_model_file
        best_accuracy = 0.0

        if os.path.isfile(BEST_MODEL_PATH):
            print(f"loading best model from {BEST_MODEL_PATH}")
            model.load_state_dict(torch.load(BEST_MODEL_PATH))

        optimizer = optim.SGD(model.parameters(), lr=learning_rate, momentum=momentum)

        for epoch in range(NUM_EPOCHS):

            for images, labels in iter(train_loader):
                images = images.to(device)
                labels = labels.to(device)
                optimizer.zero_grad()
                outputs = model(images)
                loss = F.cross_entropy(outputs, labels)
                loss.backward()
                optimizer.step()

            test_error_count = 0.0

            for images, labels in iter(test_loader):
                images = images.to(device)
                labels = labels.to(device)
                outputs = model(images)
                #test_error_count += float(torch.sum(torch.abs(labels - outputs.argmax(1))))
                #test_error_count += float(torch.sum(torch.abs(labels - outputs.argmax(1))))
                #err = len(torch.nonzero(outputs.argmax(1) - labels).flatten())
                test_error_count += len(torch.nonzero(outputs.argmax(1) - labels).flatten())

                self.log(f"error_count: {test_error_count}, dataset_len: {len(test_dataset)}")

            test_accuracy = 1.0 - float(test_error_count) / float(len(test_dataset))

            self.log('EPOCH %d: ACCURACY %f' % (epoch+1, test_accuracy))

            if test_accuracy > best_accuracy:
                self.log(f"saving best model... with accuracy: {test_accuracy}")
                torch.save(model.state_dict(), BEST_MODEL_PATH)
                best_accuracy = test_accuracy

def main(args=None):
    rclpy.init(args=args)
    node = TrainerNode()
    rclpy.spin_once(node)
    torch.cuda.empty_cache()
    rclpy.shutdown()