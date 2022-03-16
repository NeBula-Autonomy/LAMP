import itertools
import os
import numpy as np

import torch
from torch_geometric.data import InMemoryDataset, Data
from tqdm import tqdm
import random


class LoopClosureDataset(InMemoryDataset):
    def __init__(self, root, transform=None, pre_transform=None, delete_bad_files=True):
        super(LoopClosureDataset, self).__init__(root, transform, pre_transform)
        self.data, self.slices = torch.load(self.processed_paths[0])

    @property
    def raw_file_names(self):
        return []

    @property
    def processed_file_names(self):
        return ['closures.dataset']

    def download(self):
        pass

    def process(self):
        node_types = set()
        bad_files = []
        for run in os.listdir(self.root):
            if not run in ["raw", "processed"]:
                for file in os.listdir(os.path.join(self.root, run, "post")):
                    if "node" in file:
                        with open(os.path.join(self.root, run, "post", file)) as f:
                            for line in f.readlines()[1:]:
                                try:
                                    node_id, node_type, x, y, z, qx, qy, qz, qw, cx, cy, cz, cRX, cRY, cRZ, newline = line.split(
                                        ",")
                                    node_types.add(node_type)
                                except Exception as e:
                                    print(e)
                                    print(os.path.join(self.root, run, file))

        data_list = []
        num_distinct_features = 6
        num_node_features = (len(node_types) + num_distinct_features)
        node_types = sorted(list(node_types))
        g_nodes = []
        g_edges = []
        g_targets = []
        for run in tqdm(os.listdir(self.root), "Loading Graphs"):
            if run in ["raw", "processed"]:
                continue
            nodes_pre = dict()
            edges_pre = dict()
            nodes_post = dict()
            edges_post = dict()
            for file in os.listdir(os.path.join(self.root, run, "pre")):
                epoch = file.split("-")[0]
                if "node" in file:
                    nodes_pre[epoch] = file
                else:
                    edges_pre[epoch] = file
            for file in os.listdir(os.path.join(self.root, run, "post")):
                epoch = file.split("-")[0]
                if "node" in file:
                    nodes_post[epoch] = file
                else:
                    edges_post[epoch] = file

            for graph_epoch, predicted_epoch in tqdm(list(zip(sorted(nodes_pre.keys()), sorted(nodes_post.keys()))),
                                                     desc="Loading graphs from run %s" % (run)):
                cx_after_solve = 0
                cy_after_solve = 0
                cz_after_solve = 0
                with open(os.path.join(self.root, run, "post", nodes_post[predicted_epoch])) as f:
                    for line in f.readlines():
                        if "odom_node" in line:
                            # TODO this should be the sum of all the errors
                            node_id, node_type, x, y, z, qx, qy, qz, qw, cx, cy, cz, cRX, cRY, cRZ, newline = line.split(
                                ",")

                            cx = float(cx)
                            cy = float(cy)
                            cz = float(cz)

                            cx_after_solve += cx
                            cy_after_solve += cy
                            cz_after_solve += cz
                            target_node = node_id
                            # assert all(map(lambda x: not math.isnan(x), target))

                # target = [cx_after_solve, cy_after_solve,
                #                      cz_after_solve]
                target = [cx_after_solve + cy_after_solve + cz_after_solve]
                g_targets.append(target)
                cur_nodes = []
                node_ids_to_seq_id = dict()
                with open(os.path.join(self.root, run, "pre", nodes_pre[graph_epoch])) as f:
                    for i, line in enumerate(f.readlines()[1:]):
                        # try:
                        node_id, node_type, x, y, z, qx, qy, qz, qw, cx, cy, cz, cRX, cRY, cRZ, newline = line.split(
                            ",")
                        if "an" in node_type:
                            continue
                        # Number of node types + covariance
                        # node_features = np.zeros((len(node_types) + 6))
                        node_features = np.zeros((len(node_types) + num_distinct_features))
                        # node_features = np.zeros((3))

                        # node_features[6 + node_types.index(node_type)] = 1
                        node_features[num_distinct_features + node_types.index(node_type)] = 1

                        # cx = (float(cx)-empirical_node_mean[0])/empirical_node_std[0]
                        # cy = (float(cy)-empirical_node_mean[1])/empirical_node_std[1]
                        # cz = (float(cz)-empirical_node_mean[2])/empirical_node_std[2]

                        cx = float(cx)
                        cy = float(cy)
                        cz = float(cz)

                        node_features[0] = float(cx)
                        node_features[1] = float(cy)
                        node_features[2] = float(cz)
                        node_features[3] = float(x)
                        node_features[4] = float(y)
                        node_features[5] = float(z)

                        cur_nodes.append(node_features)
                        node_ids_to_seq_id[node_id] = i
                    # except Exception as e:
                    # print(f"Node Couldn't parse line {line} due to {e}")
                    #    pass
                g_nodes += cur_nodes

                cur_edges = []
                edge_attrs = []
                with open(os.path.join(self.root, run, "post", edges_post[predicted_epoch])) as f:
                    try:
                        for line in f.readlines()[1:]:
                            from_key, to_key, edge_type = line.split(",")
                            edge_1 = node_ids_to_seq_id[from_key]
                            edge_2 = node_ids_to_seq_id[to_key]
                            cur_edges.append((edge_1, edge_2))
                            edge_attr = np.zeros(4)
                            edge_attr[int(edge_type) - 1] = 1
                            edge_attrs.append(edge_attr)
                    except:
                        print(run)

                g_edges.append(cur_edges)

                cur_nodes = torch.from_numpy(np.array(cur_nodes, dtype=np.float32))
                cur_edges = torch.from_numpy(np.array(cur_edges, dtype=np.int64).T)
                target = torch.from_numpy(np.array([target], dtype=np.float32))
                edge_attrs = torch.from_numpy(np.array(edge_attrs, dtype=np.float32))
                try:
                    data = Data(x=cur_nodes, edge_index=cur_edges, y=target, edge_attr=edge_attrs)
                    #data.run_name = run
                    data.debug()
                    data_list.append(data)
                except RuntimeError as e:
                    #print("Skipping dataset {run},{graph_epoch} due to {e}")
                    print("Skipping %s" % (run))

        random.shuffle(data_list)
        data, slices = self.collate(data_list)
        torch.save((data, slices), self.processed_paths[0])



class LoopClosureNodeLabelDataset(InMemoryDataset):
    def __init__(self, root, transform=None, pre_transform=None, delete_bad_files=True):
        super(LoopClosureNodeLabelDataset, self).__init__(root, transform, pre_transform)
        self.data, self.slices = torch.load(self.processed_paths[0])

    @property
    def raw_file_names(self):
        return []

    @property
    def processed_file_names(self):
        return ['closures_nodelevel.dataset']

    def download(self):
        pass

    def process(self):
        node_types = set()
        bad_files = []
        for run in os.listdir(self.root):
            if not run in ["raw", "processed"]:
                for file in os.listdir(os.path.join(self.root, run, "post")):
                    if "node" in file:
                        with open(os.path.join(self.root, run, "post", file)) as f:
                            for line in f.readlines()[1:]:
                                try:
                                    node_id, node_type, x, y, z, qx, qy, qz, qw, cx, cy, cz, cRX, cRY, cRZ, newline = line.split(
                                        ",")
                                    node_types.add(node_type)
                                except Exception as e:
                                    print(e)
                                    print(os.path.join(self.root, run, file))

        data_list = []
        num_distinct_features = 6
        num_node_features = (len(node_types) + num_distinct_features)
        node_types = sorted(list(node_types))
        g_nodes = []
        g_edges = []
        g_targets = []
        for run in tqdm(os.listdir(self.root), "Loading Graphs"):
            if run in ["raw", "processed"]:
                continue
            nodes_pre = dict()
            edges_pre = dict()
            nodes_post = dict()
            edges_post = dict()
            for file in os.listdir(os.path.join(self.root, run, "pre")):
                epoch = file.split("-")[0]
                if "node" in file:
                    nodes_pre[epoch] = file
                else:
                    edges_pre[epoch] = file
            for file in os.listdir(os.path.join(self.root, run, "post")):
                epoch = file.split("-")[0]
                if "node" in file:
                    nodes_post[epoch] = file
                else:
                    edges_post[epoch] = file

            for graph_epoch, predicted_epoch in tqdm(list(zip(sorted(nodes_pre.keys()), sorted(nodes_post.keys()))),
                                                     desc="Loading graphs from run %s" % (run)):
                target = []
                with open(os.path.join(self.root, run, "post", nodes_post[predicted_epoch])) as f:
                    for line in f.readlines():
                        if "odom_node" in line:
                            # TODO this should be the sum of all the errors
                            node_id, node_type, x, y, z, qx, qy, qz, qw, cx, cy, cz, cRX, cRY, cRZ, newline = line.split(
                                ",")

                            cx = float(cx)
                            cy = float(cy)
                            cz = float(cz)

                            target.append([cx,cy,cz])
                #target = [cx_after_solve + cy_after_solve + cz_after_solve]
                g_targets.append(target)
                cur_nodes = []
                node_ids_to_seq_id = dict()
                with open(os.path.join(self.root, run, "pre", nodes_pre[graph_epoch])) as f:
                    for i, line in enumerate(f.readlines()[1:]):
                        # try:
                        node_id, node_type, x, y, z, qx, qy, qz, qw, cx, cy, cz, cRX, cRY, cRZ, newline = line.split(
                            ",")
                        if "an" in node_type:
                            continue
                        # Number of node types + covariance
                        # node_features = np.zeros((len(node_types) + 6))
                        node_features = np.zeros((len(node_types) + num_distinct_features))
                        # node_features = np.zeros((3))

                        # node_features[6 + node_types.index(node_type)] = 1
                        node_features[num_distinct_features + node_types.index(node_type)] = 1

                        # cx = (float(cx)-empirical_node_mean[0])/empirical_node_std[0]
                        # cy = (float(cy)-empirical_node_mean[1])/empirical_node_std[1]
                        # cz = (float(cz)-empirical_node_mean[2])/empirical_node_std[2]

                        cx = float(cx)
                        cy = float(cy)
                        cz = float(cz)

                        node_features[0] = float(cx)
                        node_features[1] = float(cy)
                        node_features[2] = float(cz)
                        node_features[3] = float(x)
                        node_features[4] = float(y)
                        node_features[5] = float(z)

                        cur_nodes.append(node_features)
                        node_ids_to_seq_id[node_id] = i
                    # except Exception as e:
                    # print(f"Node Couldn't parse line {line} due to {e}")
                    #    pass
                g_nodes += cur_nodes

                cur_edges = []
                edge_attrs = []
                with open(os.path.join(self.root, run, "post", edges_post[predicted_epoch])) as f:
                    try:
                        for line in f.readlines()[1:]:
                            from_key, to_key, edge_type = line.split(",")
                            edge_1 = node_ids_to_seq_id[from_key]
                            edge_2 = node_ids_to_seq_id[to_key]
                            cur_edges.append((edge_1, edge_2))
                            edge_attr = np.zeros(4)
                            edge_attr[int(edge_type) - 1] = 1
                            edge_attrs.append(edge_attr)
                    except:
                        print(run)

                g_edges.append(cur_edges)

                cur_nodes = torch.from_numpy(np.array(cur_nodes, dtype=np.float32))
                cur_edges = torch.from_numpy(np.array(cur_edges, dtype=np.int64).T)
                target = torch.from_numpy(np.array(target, dtype=np.float32))
                edge_attrs = torch.from_numpy(np.array(edge_attrs, dtype=np.float32))
                try:
                    data = Data(x=cur_nodes, edge_index=cur_edges, y=target, edge_attr=edge_attrs)
                    data.run_name = run
                    data.debug()
                    data_list.append(data)
                except RuntimeError as e:
                    #print(f"Skipping dataset {run},{graph_epoch} due to {e}")
                    print("Skipping dataset %s" % (run))

        random.shuffle(data_list)
        data, slices = self.collate(data_list)
        torch.save((data, slices), self.processed_paths[0])
