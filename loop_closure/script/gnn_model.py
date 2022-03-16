import torch
from torch.nn import Linear, ModuleList, BatchNorm1d
import torch.nn.functional as F
from torch_geometric.nn import GCNConv, GraphConv, GATConv, SAGEConv, GMMConv
from torch_geometric.nn import global_mean_pool, global_add_pool, GlobalAttention
from torch.nn import ModuleList, Embedding,Sequential, ReLU
from torch.nn import Sequential as Seq, Linear as Lin, ReLU

class LoopClosureGNN(torch.nn.Module):
    def __init__(self, hidden_channels, num_input_features, num_edge_attr, num_output_features, ends_in_sigmoid=False):
        super(LoopClosureGNN, self).__init__()
        #torch.manual_seed(12345)
        self.node_encoder = Linear(num_input_features, hidden_channels)
        self.edge_encoder = Linear(num_edge_attr, hidden_channels)


    #self.input_norm = BatchNorm1d(num_input_features)
        #self.conv1 = SAGEConv(hidden_channels, hidden_channels)
        #self.conv1 = GENConv(hidden_channels, hidden_channels,msg_norm=True,learn_t=True,learn_p=True,learn_msg_scale=True)
        self.conv1 = GMMConv(hidden_channels, hidden_channels, 4,5)
        #self.gcs = ModuleList([SAGEConv(hidden_channels, hidden_channels) for i in range(4)])
        #self.gcs = ModuleList([GENConv(hidden_channels,hidden_channels,msg_norm=True,learn_t=True,learn_p=True,learn_msg_scale=True) for i in range(2)])
        self.gcs = ModuleList([GMMConv(hidden_channels,hidden_channels,4, 5) for i in range(2)])

        #self.conv1 = SAGEConv(num_input_features, hidden_channels)
        #self.gcs = ModuleList([SAGEConv(hidden_channels, hidden_channels) for i in range(4)])

        num_linear = 3
        self.lins = ModuleList([Linear(hidden_channels, hidden_channels) for i in range(num_linear)])
        self.batch_norms = ModuleList([BatchNorm1d(hidden_channels) for i in range(num_linear + 2)])
        self.lin = Linear(hidden_channels, num_output_features)
        self.ends_in_sigmoid = ends_in_sigmoid

    def forward(self, x, edge_index,edge_attr, batch):
        # 1. Obtain node embeddings
        #x = self.input_norm(x)
        x = self.node_encoder(x)
        #edge_attr = self.edge_encoder(edge_attr)
        x = self.conv1(x, edge_index,edge_attr)
        for gc in self.gcs:
            x = x.tanh()
            x = gc(x, edge_index,edge_attr)

        # 2. Readout layer
        x = global_add_pool(x, batch)  # [batch_size, hidden_channels]

        # 3. Apply a final classifier
        for lin,batch_norm in zip(self.lins,self.batch_norms[1:]):
            #x = F.dropout(x, p=0.5, training=self.training)
            x = batch_norm(x)
            x = lin(x)
            x = torch.tanh(x)
        #x = F.dropout(x, p=0.5, training=self.training)
        x = self.batch_norms[-1](x)
        x = self.lin(x)
        if self.ends_in_sigmoid:
            x = torch.sigmoid(x)
        return x

class LoopClosureGNNGlobalAttention(torch.nn.Module):
    def __init__(self, hidden_channels, num_input_features, num_output_features, ends_in_sigmoid=False):
        super(LoopClosureGNNGlobalAttention, self).__init__()
        #torch.manual_seed(12345)

        #self.input_norm = BatchNorm1d(num_input_features)
        self.conv1 = GENConv(num_input_features, hidden_channels,msg_norm=True,learn_t=True,learn_p=True,learn_msg_scale=True)
        #self.gcs = ModuleList([GraphConv(hidden_channels, hidden_channels) for i in range(4)])
        self.gcs = ModuleList([GENConv(hidden_channels,hidden_channels,msg_norm=True,learn_t=True,learn_p=True,learn_msg_scale=True) for i in range(2)])

        #self.conv1 = SAGEConv(num_input_features, hidden_channels)
        #self.gcs = ModuleList([SAGEConv(hidden_channels, hidden_channels) for i in range(4)])

        num_linear = 3
        self.lins = ModuleList([Linear(hidden_channels, hidden_channels) for i in range(num_linear)])
        self.batch_norms = ModuleList([BatchNorm1d(hidden_channels) for i in range(num_linear + 2)])
        self.lin = Linear(hidden_channels, num_output_features)
        self.ends_in_sigmoid = ends_in_sigmoid
        gate_nn = Seq(Lin(hidden_channels, hidden_channels), ReLU(), Lin(hidden_channels, 1))

        self.glob = GlobalAttention(gate_nn)

    def forward(self, x, edge_index, batch):
        # 1. Obtain node embeddings
        #x = self.input_norm(x)
        x = self.conv1(x, edge_index)
        for gc in self.gcs:
            x = x.tanh()
            x = gc(x, edge_index)

        # 2. Readout layer
        #x = global_add_pool(x, batch)  # [batch_size, hidden_channels]
        x = self.glob(x,batch)

        # 3. Apply a final classifier
        for lin,batch_norm in zip(self.lins,self.batch_norms[1:]):
            #x = F.dropout(x, p=0.5, training=self.training)
            x = batch_norm(x)
            x = lin(x)
            x = torch.tanh(x)
        #x = F.dropout(x, p=0.5, training=self.training)
        x = self.batch_norms[-1](x)
        x = self.lin(x)
        if self.ends_in_sigmoid:
            x = torch.sigmoid(x)
        return x

class LoopClosureUNet(torch.nn.Module):
        def __init__(self,hidden_channels,num_features,num_output_features,unet_depth=3):
            super(LoopClosureUNet, self).__init__()
            #pool_ratios = [2000 / data.num_nodes, 0.5]
            self.unet = GraphUNet(num_features, hidden_channels, hidden_channels,
                                  depth=unet_depth)
            num_linear = 3
            self.lins = ModuleList([Linear(hidden_channels, hidden_channels) for i in range(num_linear)])
            self.batch_norms = ModuleList([BatchNorm1d(hidden_channels) for i in range(num_linear + 1)])
            self.lin = Linear(hidden_channels, num_output_features)

        def forward(self,x,edge_index,batch):
            # edge_index, _ = dropout_adj(data.edge_index, p=0.2,
            #                             force_undirected=True,
            #                             num_nodes=data.num_nodes,
            #                             training=self.training)
            x = F.dropout(x, p=0.92, training=self.training)

            x = self.unet(x, edge_index)
            # 2. Readout layer
            x = global_add_pool(x, batch)  # [batch_size, hidden_channels]

            # 3. Apply a final classifier
            for lin,batch_norm in zip(self.lins,self.batch_norms):
                #x = F.dropout(x, p=0.5, training=self.training)
                x = batch_norm(x)
                x = lin(x)
                x = torch.tanh(x)
            #x = F.dropout(x, p=0.5, training=self.training)
            x = self.batch_norms[-1](x)
            x = self.lin(x)

            return x

class LoopClosurePNA(torch.nn.Module):
    def __init__(self,node_features,output_features,deg):
        super(LoopClosurePNA, self).__init__()
        hidden_dim = 32
        self.node_emb = Linear(node_features, hidden_dim)

        aggregators = ['mean', 'min', 'max', 'std']
        scalers = ['identity', 'amplification', 'attenuation']

        self.convs = []
        self.batch_norms = []
        for _ in range(2):
            conv = PNAConv(in_channels=hidden_dim, out_channels=hidden_dim,
                           aggregators=aggregators, scalers=scalers, deg=deg,
                           towers=4, pre_layers=1, post_layers=1,
                           divide_input=False)
            self.convs.append(conv)
            self.batch_norms.append(BatchNorm1d(hidden_dim))
        self.convs = ModuleList(self.convs)
        self.batch_norms = ModuleList(self.convs)

        # self.mlp = Sequential(Linear(hidden_dim, int(hidden_dim/2)), ReLU(),Linear(int(hidden_dim/2),int(hidden_dim/4)), ReLU(),
        #                       Linear(int(hidden_dim/4), output_features))
        self.mlp = Sequential(Linear(hidden_dim, int(hidden_dim/2)), ReLU(), Linear(int(hidden_dim/2), output_features))

    def forward(self, x, edge_index, batch):
        x = self.node_emb(x)
        #edge_attr = self.edge_emb(edge_attr)

        for conv, batch_norm in zip(self.convs, self.batch_norms):
            #x = F.relu(batch_norm(conv(x, edge_index, edge_attr)))
            x = F.relu(batch_norm(conv(x, edge_index),edge_index))

        x = global_add_pool(x, batch)
        return self.mlp(x)

class DeeperGCN(torch.nn.Module):
    def __init__(self, hidden_channels, input_features,output_features,num_layers=2):
        super(DeeperGCN, self).__init__()

        self.node_encoder = Linear(input_features, hidden_channels)

        self.layers = torch.nn.ModuleList()
        for i in range(1, num_layers + 1):
            conv = GENConv(hidden_channels, hidden_channels, aggr='softmax',
                           t=1.0, learn_t=True, num_layers=2, norm='layer')
            norm = LayerNorm(hidden_channels)
            act = ReLU(inplace=True)

            layer = DeepGCNLayer(conv, norm, act, block='res+', dropout=0.1,
                                 ckpt_grad=i % 3)
            self.layers.append(layer)

        self.mlp = Sequential(Linear(hidden_channels, int(hidden_channels/2)), ReLU(),
                              Linear(int(hidden_channels/2), output_features))
    def forward(self, x, edge_index,batch ):
        x = self.node_encoder(x)
        #edge_attr = self.edge_encoder(edge_attr)
        x = self.layers[0].conv(x, edge_index)

        for layer in self.layers[1:]:
            x = layer(x, edge_index)

        x = self.layers[0].act(self.layers[0].norm(x))
        x = F.dropout(x, p=0.1, training=self.training)

        x = global_add_pool(x, batch)
        return self.mlp(x)
