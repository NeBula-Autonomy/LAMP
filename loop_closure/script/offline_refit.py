#!/usr/bin/env python
# This script fits the gnn model used in batching to new training data
from __future__ import division

import pickle
import os
import math
import torch
from torch_geometric.datasets import TUDataset
from torch_geometric.data import InMemoryDataset, Data, DataLoader
import numpy as np
import torch
from torch_geometric.transforms import Compose, LocalDegreeProfile, OneHotDegree, Cartesian, Distance
from tqdm import tqdm
import matplotlib.pyplot as plt
from dataset import LoopClosureDataset, LoopClosureNodeLabelDataset
from gnn_model import LoopClosureGNN
#from torch.utils.tensorboard import SummaryWriter
from torch_geometric.utils import degree
from sklearn.metrics import mean_squared_error

root = "/home/chris/current_training_data/"
save_name = "./model.pkl"
cpu_save_name = "./model-cpu.pkl"

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
# device = "cpu"


dataset = LoopClosureDataset("/media/chris/hdd3/more_training_data")

val_holdout = int(len(dataset) / 4)
train_dataset = dataset
train_dataset = dataset[:-val_holdout]

validation_dataset = dataset[-val_holdout:]
train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
validation_loader = DataLoader(validation_dataset, batch_size=32, shuffle=True)
print("Dataset Size: %d, Train Size %d, Val Size %d" % (len(dataset), len(train_dataset), len(validation_dataset)))


def train(model):
    model.train()

    for data in train_loader:  # Iterate in batches over the training dataset.
        data.to(device)
        out = model(data.x, data.edge_index,data.edge_attr, data.batch)  # Perform a single forward pass.
        loss = criterion(out, data.y)  # Compute the loss.
        loss.backward()  # Derive gradients.
        optimizer.step()  # Update parameters based on gradients.
        optimizer.zero_grad()  # Clear gradients.


def test(loader, model):
    model.eval()
    predictions, actuals = list(), list()
    for i, data in enumerate(loader):
        data.to(device)
        # evaluate the model on the test set
        yhat = model(data.x, data.edge_index,data.edge_attr,data.batch)
        # retrieve numpy array
        yhat = yhat.cpu().detach().numpy()
        actual = data.y.cpu().detach().numpy()
        # actual = actual.reshape((len(actual), 3))
        # store
        predictions.append(yhat)
        actuals.append(actual)
    predictions, actuals = np.vstack(predictions), np.vstack(actuals)
    # calculate mse
    mse = mean_squared_error(actuals, predictions, multioutput='raw_values')
    return np.sqrt(mse)


if __name__ == "__main__":
    model = LoopClosureGNN(64, dataset.num_node_features,dataset.num_edge_features,dataset.num_classes).to(device)

    continue_training = True
    should_train = False
    val_skip = 1
    try:
        if continue_training:
            model.load_state_dict(torch.load(save_name))
    except Exception as e:
        print(e)


    if should_train:
        optimizer = torch.optim.Adam(model.parameters(), lr=0.0005, weight_decay=0.005)
        criterion = torch.nn.MSELoss()
        pbar = tqdm(range(1, 5 * (10 ** 2)), "Training Epochs")
        #writer = SummaryWriter()
        best_test_accuracy = float("inf")
        saving = False
        for epoch in pbar:
            train(model)
            train_acc = test(train_loader, model)
            test_acc = test(validation_loader, model)
            pbar.set_postfix({"Training Accuracy": train_acc, "Testing Accuracy": test_acc, "Saving": saving, "Best Test Accuracy": best_test_accuracy})
            # for x in range(test_acc.shape[0]):
            #     writer.add_scalar("Accuracy/train" + str(x), train_acc[x], epoch)
            #     writer.add_scalar("Accuracy/test" + str(x), test_acc[x], epoch)
            if sum(test_acc) < best_test_accuracy:
                saving = True
                best_test_accuracy = sum(test_acc)
                with open(save_name, "wb") as f:
                    torch.save(model.state_dict(), f)
            else:
                saving = False
        model.load_state_dict(torch.load(save_name))
    model.eval()
    correct = 0
    total = 0
    differences = []

    predictions = []

    for i, graph_1 in tqdm(list(enumerate(validation_dataset)),
                           desc="Compute Ordering"):  # Iterate in batches over the training
        for graph_2 in validation_dataset[i::val_skip]:
            #if graph_1.run_name == graph_2.run_name:
            target_sum_1 = torch.sum(graph_1.y)
            target_sum_2 = torch.sum(graph_2.y)
            #graphs that are super close who cares
            if np.abs((target_sum_1 - target_sum_2).cpu().detach().numpy()) < 0.0001:
                continue

            graph_1.to(device)
            batch = torch.zeros(graph_1.x.shape[0], dtype=int).to(device)
            pred_sum_1 = torch.sum(model(graph_1.x, graph_1.edge_index, graph_1.edge_attr,batch))
            prediction = pred_sum_1.cpu().detach().numpy()


            graph_2.to(device)
            batch = torch.zeros(graph_2.x.shape[0], dtype=int).to(device)
            pred_sum_2 = torch.sum(model(graph_2.x, graph_2.edge_index,graph_2.edge_attr,batch))
            diff = (target_sum_1 - target_sum_2) - (pred_sum_1 - pred_sum_2)
            differences.append(diff.cpu().detach().numpy())
            if (target_sum_1 <= target_sum_2 and pred_sum_1 <= pred_sum_2) or (
                    target_sum_1 > target_sum_2 and pred_sum_1 > pred_sum_2):
                correct += 1
            total += 1

    print("Ordering: Correct %f Total %f Accuracy %f" % (correct, total, correct/total))
    print("MSE: %f" % (test(validation_loader,model)))




    plt.figure()
    plt.hist(np.array(differences),bins=50)
    plt.title("Difference histogram")
    plt.figure()
    plt.hist(np.array(predictions),bins=50)
    plt.title("Predictions")
    plt.show()