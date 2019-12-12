'''
Copyright Notes

Authors: Alex Stephens    (alex.stephens@jpl.nasa.gov)
'''


import rospy
import rosbag
from pose_graph_msgs.msg import PoseGraph
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from random import random
import math
import key_handling


#
# outputfile_GT = 'pose_graph_GT'

class Node(object):
    '''
    Simple class representing a pose graph node.
    Only stores x, y, z and id.
    '''

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
        self.qw = node.pose.orientation.w
        self.qx = node.pose.orientation.x
        self.qy = node.pose.orientation.y
        self.qz = node.pose.orientation.z


def AddNoise(nodes):
    '''
    Add noise to the x,y,z coordinates of a list of nodes.
    Used for debugging purposes only.
    '''

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


def PlotPoseGraphs3D(nodes_data, nodes_gt):
    '''
    Display two sets of pose graph nodes on a single 3D figure.
    '''

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    x1 = [g.x for g in nodes_data]
    y1 = [g.y for g in nodes_data]
    z1 = [g.z for g in nodes_data]
    x2 = [g.x for g in nodes_gt]
    y2 = [g.y for g in nodes_gt]
    z2 = [g.z for g in nodes_gt]

    ax.plot(x1, y1, zs=z1, label='SLAM estimate')
    ax.plot(x2, y2, zs=z2, label='Ground truth')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')

    plt.legend()
    scaling = np.array([getattr(ax, 'get_{}lim'.format(dim))()
                        for dim in 'xyz'])
    ax.auto_scale_xyz(*[[np.min(scaling), np.max(scaling)]] * 3)

def PlotPoseGraphs2D(nodes_data, nodes_gt):
    '''
    Display two sets of pose graph nodes on a single 2D figure.
    '''

    fig = plt.figure()
    ax = fig.gca()

    x1 = [g.x for g in nodes_data]
    y1 = [g.y for g in nodes_data]
    x2 = [g.x for g in nodes_gt]
    y2 = [g.y for g in nodes_gt]

    ax.plot(x1, y1, label='SLAM estimate')
    ax.plot(x2, y2, label='Ground truth')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(b=True, which='both', color='0.65', linestyle='-')
    plt.axis('equal')

    plt.legend()
    

def DistBetween(n1, n2):
    '''
    Compute Euclidean distance between two Nodes.
    '''

    return math.sqrt((n1.x - n2.x)**2 + ((n1.y - n2.y))**2 + ((n1.z - n2.z))**2)


def PlotErrors(nodes_data, nodes_gt):
    '''
    Plot the Euclidean distance errors between corresponding nodes in two pose graphs.
    Shows errors in x, y, z as well as total error.
    '''

    fig = plt.figure()
    ax = fig.gca()

    x1 = [g.x for g in nodes_gt]
    y1 = [g.y for g in nodes_gt]
    z1 = [g.z for g in nodes_gt]
    x2 = [g.x for g in nodes_data]
    y2 = [g.y for g in nodes_data]
    z2 = [g.z for g in nodes_data]
    dist = [0] * len(nodes_gt)
    for i in range(1, len(nodes_gt)):
        dist[i] = dist[i - 1] + DistBetween(nodes_gt[i], nodes_gt[i - 1])

    x_err = [abs(p1 - p2) for p1, p2 in zip(x1, x2)]
    y_err = [abs(p1 - p2) for p1, p2 in zip(y1, y2)]
    z_err = [abs(p1 - p2) for p1, p2 in zip(z1, z2)]
    total_err = [math.sqrt(x**2 + y**2 + z**2)
                 for x, y, z in zip(x_err, y_err, z_err)]

    print('max total error:', max(total_err))

    yaw = [math.atan2(2.0*(n.qy*n.qz + n.qw*n.qx), n.qw**2 - n.qx**2 - n.qy**2 + n.qz**2) for n in nodes_gt]

    # comparison metric to be displayed as a gray area on the error plot
    comp = z1 # elevation

    scaled = [(n - min(comp)) / (max(comp) - min(comp)) * 0.8 * max(total_err)
               for n in comp]
    # ax.fill_between(dist, 0, scaled, color='gray', alpha=0.3, label='elevation (rel.)')

    ax.plot(dist, x_err, label='x error')
    ax.plot(dist, y_err, label='y error')
    ax.plot(dist, z_err, label='z error')
    ax.plot(dist, total_err, label='total error')
    plt.xlabel('Distance travelled (m)')
    plt.ylabel('Localisation error (m)')

    plt.grid()
    plt.legend()

    plt.xlim(0, max(dist))
    plt.ylim(0, plt.ylim()[1])


def ReadPoseGraphFromG2O(file):
    '''
    Read a pose graph in g2o format from a text file.
    Extracts nodes only, edges are ignored
    '''

    nodes = []

    with open(file) as f:
        for line in f:
            if not line.startswith('VERTEX'):
                break

            nodes.append(Node(line))

    return nodes


def IsRobotKey(key):
    return key_handling.split_pg_key(key)[0] in 'abcdef'


def ReadPoseGraphFromBagfile(filename):
    '''
    Read a pose graph in g2o format from a bagfile.
    Extracts nodes only, edges are ignored
    '''

    bag = rosbag.Bag(filename)

    msgs, nodes = [], []

    for topic, msg, t in bag.read_messages(topics='pose_graph'):
        msgs.append(msg)

    # Take most recent message only
    msg = msgs[-1]

    for n in msg.nodes:

        # ignore artifact keys
        if not IsRobotKey(n.key):
            continue

        new_node = Node()
        new_node.InitFromNodeMsg(n)
        nodes.append(new_node)

    return nodes


def PerformAnalysis(abspath, filename_data, filename_gt):

    if not filename_data.endswith('.bag'):
        print("Data must be a .bag file")

    if not filename_gt.endswith('.bag'):
        print("Ground truth must be a .bag file")

    nodes1 = ReadPoseGraphFromBagfile(abspath + filename_data)
    nodes2 = ReadPoseGraphFromBagfile(abspath + filename_gt)

    PlotPoseGraphs2D(nodes1, nodes2)
    PlotPoseGraphs3D(nodes1, nodes2)
    PlotErrors(nodes1, nodes2)

    # show all plots
    plt.show()


if __name__ == '__main__':

    abspath = '/home/costar/data/groundtruth/'
    filename_data = 'saved_pose_graph.bag'
    filename_gt = 'saved_pose_graph_gt.bag'

    PerformAnalysis(abspath, filename_data, filename_gt)
