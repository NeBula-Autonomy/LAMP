import random
import torch
import numpy as np
from torch_geometric.data import Data, DataLoader
import rospy
import operator as op
from functools import reduce
from itertools import repeat as _repeat
from bisect import bisect as _bisect
import operator
import json


def constuct_pytorch_geometric_graph(nodes, edges, edge_attrs=None):
    nodes = torch.from_numpy(np.array(nodes, dtype=np.float32))
    edges = torch.from_numpy(np.array(edges, dtype=np.int64).T)
    #if edge_attrs is not None:
    edge_attrs = torch.from_numpy(np.array(edge_attrs, dtype=np.float32))
    return Data(x=nodes, edge_index=edges, edge_attr=edge_attrs)
    # else:
    #     return Data(x=nodes, edge_index=edges)


def run_model_on_list_of_graphs(model, graphs):
    all_batches = []
    for graph in graphs:
        batch = torch.zeros(graph.x.shape[0], dtype=int)
        all_batches.append(torch.sum(model(graph.x, graph.edge_index, graph.edge_attr, batch)).cpu().detach().numpy())
    return np.array(all_batches)



def ncr(n, r):
    r = min(r, n - r)
    numer = reduce(op.mul, range(n, n - r, -1), 1)
    denom = reduce(op.mul, range(1, r + 1), 1)
    return numer // denom  # or / in Python 2

#Backport from python 3.7 itertools.accumulate
def accumulate(iterable, func=operator.add, initial=None):
    'Return running totals'
    # accumulate([1,2,3,4,5]) --> 1 3 6 10 15
    # accumulate([1,2,3,4,5], initial=100) --> 100 101 103 106 110 115
    # accumulate([1,2,3,4,5], operator.mul) --> 1 2 6 24 120
    it = iter(iterable)
    total = initial
    if initial is None:
        try:
            total = next(it)
        except StopIteration:
            return
    yield total
    for element in it:
        total = func(total, element)
        yield total

#Backport from python 3.7 random.choices
def choices(population, weights=None, cum_weights=None, k=1):
    """Return a k sized list of population elements chosen with replacement.
    If the relative weights or cumulative weights are not specified,
    the selections are made with equal probability.
    """
    n = len(population)
    if cum_weights is None:
        if weights is None:
            _int = int
            n += 0.0  # convert to float for a small speed improvement
            return [population[_int(random.random() * n)] for i in _repeat(None, k)]
        cum_weights = list(accumulate(weights))
    elif weights is not None:
        raise TypeError('Cannot specify both weights and cumulative weights')
    if len(cum_weights) != n:
        raise ValueError('The number of weights does not match the population')
    bisect = _bisect
    total = cum_weights[-1] + 0.0  # convert to float
    hi = n - 1
    return [population[bisect(cum_weights, random.random() * total, 0, hi)]
            for i in _repeat(None, k)]


def fast_deepcopy(a):
    return json.loads(json.dumps(a))