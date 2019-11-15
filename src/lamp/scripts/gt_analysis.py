import rospy
import rosbag
from pose_graph_msgs.msg import PoseGraph
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from random import random
import math


class Node(object):

    def __init__(self, string=''):

        self.id = 0
        self.x = 0
        self.y = 0
        self.z = 0

        if string == '':
            return

        data = string.split()

        self.id = int(data[1])
        self.x = float(data[2])
        self.y = float(data[3])
        self.z = float(data[4])

    def InitFromNodeMsg(self, node):
        self.id = node.ID
        self.x = node.pose.position.x
        self.y = node.pose.position.y
        self.z = node.pose.position.z


'''
Read a pose graph in g2o format from a text file.
Extracts nodes only, edges are ignored
'''


def AddNoise(nodes):

    nodes2 = []
    noise_x = 0.02
    noise_y = 0.01
    noise_z = 0.04
    offset_x = 0.03
    offset_y = 0.0
    offset_z = 0.05
    d = 0

    for n in nodes:
        new = Node()
        new.id = n.id
        new.x = n.x + d * (offset_x + noise_x * random())
        new.y = n.y + d * (offset_y + noise_y * random())
        new.z = n.z + d * (offset_z + noise_z * random())

        nodes2.append(new)
        d += 1

    return nodes2


def PlotPoseGraphs(nodes1, nodes2):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    x1 = [g.x for g in nodes1]
    y1 = [g.y for g in nodes1]
    z1 = [g.z for g in nodes1]
    x2 = [g.x for g in nodes2]
    y2 = [g.y for g in nodes2]
    z2 = [g.z for g in nodes2]

    ax.plot(x1, y1, zs=z1, label='ground truth')
    ax.plot(x2, y2, zs=z2, label='pose graph')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    scaling = np.array([getattr(ax, 'get_{}lim'.format(dim))()
                        for dim in 'xyz'])
    ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]] * 3)
    # ax.set_aspect('equal', 'box')


def DistBetween(n1, n2):
    return math.sqrt((n1.x - n2.x)**2 + ((n1.y - n2.y))**2 + ((n1.z - n2.z))**2)


def PlotErrors(nodes1, nodes2):
    fig = plt.figure()
    ax = fig.gca()

    x1 = [g.x for g in nodes1]
    y1 = [g.y for g in nodes1]
    z1 = [g.z for g in nodes1]
    x2 = [g.x for g in nodes2]
    y2 = [g.y for g in nodes2]
    z2 = [g.z for g in nodes2]
    dist = [0] * len(nodes1)
    base = [0] * len(nodes1)
    for i in range(1, len(nodes1)):
        dist[i] = dist[i - 1] + DistBetween(nodes1[i], nodes1[i - 1])

    x_err = [abs(p1 - p2) for p1, p2 in zip(x1, x2)]
    y_err = [abs(p1 - p2) for p1, p2 in zip(y1, y2)]
    z_err = [abs(p1 - p2) for p1, p2 in zip(z1, z2)]
    total_err = [math.sqrt(x**2 + y**2 + z**2)
                 for x, y, z in zip(x_err, y_err, z_err)]

    print('max total error:', max(total_err))

    zscaled = [(z - min(z1)) / (max(z1) - min(z1)) * 0.8 * max(total_err)
               for z in z1]
    ax.fill_between(dist, 0, zscaled, color='gray', alpha=0.3)

    ax.plot(dist, x_err, label='x error')
    ax.plot(dist, y_err, label='y error')
    ax.plot(dist, z_err, label='z error')
    ax.plot(dist, total_err, label='total error')
    plt.xlabel('distance travelled (m)')
    plt.ylabel('error')

    plt.grid()
    plt.legend()

    plt.xlim(0, plt.xlim()[1])
    plt.ylim(0, plt.ylim()[1])


def ReadPoseGraphFromG2O(file):

    nodes = []

    with open(file) as f:

        for line in f:
            if not line.startswith('VERTEX'):
                break

            nodes.append(Node(line))

    return nodes


def ReadPoseGraphFromBagfile(filename):

    bag = rosbag.Bag(filename)

    msgs = []

    for topic, msg, t in bag.read_messages(topics='/base1/lamp/pose_graph'):
        print(topic)
        msgs.append(msg)

    msg = msgs[-1]

    nodes = []

    for n in msg.nodes:
        new_node = Node()
        new_node.InitFromNodeMsg(n)
        nodes.append(new_node)

    return nodes


if __name__ == '__main__':

    # nodes1 = ReadPoseGraphFromG2O('graph.txt')
    nodes1 = ReadPoseGraphFromBagfile('../data/pose_graph.bag')
    nodes2 = AddNoise(nodes1)

    PlotPoseGraphs(nodes1, nodes2)
    PlotErrors(nodes1, nodes2)

    # show all plots
    plt.show()
