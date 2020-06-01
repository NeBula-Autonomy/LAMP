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
import tf2_geometry_msgs
import tf
import yaml
import json
import xmltodict

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})


class Node(object):
    '''
    Simple class representing a pose graph node.
    Only stores x, y, z and id.
    '''

    def __init__(self, string=''):

        self.id = 0
        self.stamp = 0
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
        self.stamp = node.header.stamp
        self.x = node.pose.position.x
        self.y = node.pose.position.y
        self.z = node.pose.position.z
        self.qw = node.pose.orientation.w
        self.qx = node.pose.orientation.x
        self.qy = node.pose.orientation.y
        self.qz = node.pose.orientation.z


class Artifact(object):
    '''
    Simple class representing an artifact
    '''

    def __init__(self):

        self.id = 0
        self.stamp = 0
        self.label = ''
        self.x = 0
        self.y = 0
        self.z = 0

    def InitFromMsg(self, msg):
        self.stamp = msg.header.stamp
        self.label = msg.label
        self.x = msg.point.point.x
        self.y = msg.point.point.y
        self.z = msg.point.point.z

    def InitFromLabelXYZ(self, label, x, y, z):
        self.label = label
        self.x = x
        self.y = y
        self.z = z


def GetClosestPoseGraphNodes(nodes, artifact_stamp):
    # https://www.quora.com/How-do-I-iterate-through-a-list-in-python-while-comparing-the-values-at-adjacent-indices
    for prev, curr in zip(nodes[::], nodes[1::]):
        dt_curr = artifact_stamp - RosTimeToSecs(curr.stamp)
        dt_prev = artifact_stamp - RosTimeToSecs(prev.stamp)
        # print dt_prev, dt_curr
        if dt_curr < 0:  # artifact between these two nodes
            fract = dt_prev / (RosTimeToSecs(curr.stamp) - RosTimeToSecs(prev.stamp))
            return prev, curr, fract


def GetArtifactWorld(prev, curr, fract, artifact, world_T_map):
    # spherical interpolation of quaternion
    q = tf.transformations.quaternion_slerp([prev.qx, prev.qy, prev.qz, prev.qw], [curr.qx, curr.qy, curr.qz, curr.qw], fract)
    map_T_base_link_rot = tf.transformations.quaternion_matrix(q)
    # linear interpolation of position
    t = [fract * prev.x + (1 - fract) * curr.x, fract * prev.y + (1 - fract) * curr.y, fract * prev.z + (1 - fract) * curr.z]
    map_T_base_link_tr = tf.transformations.translation_matrix(t)
    map_T_base_link = tf.transformations.concatenate_matrices(map_T_base_link_tr, map_T_base_link_rot)

    base_link_T_artifact = tf.transformations.translation_matrix([artifact.x, artifact.y, artifact.z])

    # world_T_artifact = tf.transformations.concatenate_matrices(world_T_map, map_T_base_link, base_link_T_artifact)
    world_T_artifact = tf.transformations.concatenate_matrices(map_T_base_link, base_link_T_artifact)
    world_T_artifact_tr = tf.transformations.translation_from_matrix(world_T_artifact)

    artifact_node = Artifact()
    artifact_node.InitFromLabelXYZ(artifact.label, world_T_artifact_tr[0], world_T_artifact_tr[1], world_T_artifact_tr[2])
    return artifact_node


def IsArtifact(name):
    ''' Assumes name is in the format rescue_randy_3, phone_2, etc. '''
    artifact_names = ['rescue', 'backpack', 'phone', 'helmet', 'rope']
    if name.split('_')[0] in artifact_names:
        return True
    else:
        return False


def ArtifactLabelFromName(name):
    if name.startswith('rescue'):
        return 'Survivor'
    elif name.startswith('backpack'):
        return 'Backpack'
    elif name.startswith('phone'):
        return 'Cell Phone'
    elif name.startswith('helmet'):
        return 'Helmet'
    elif name.startswith('rope'):
        return 'Rope'                        

def IsRobotKey(key):
    return key_handling.split_pg_key(key)[0] in 'abcdef'


def ReadPoseGraphFromBagfile(filename, robot_namespace):
    '''
    Read a pose graph from a bagfile.
    Extracts nodes only, edges are ignored
    '''

    bag = rosbag.Bag(filename)

    msgs, nodes = [], []

    for topic, msg, t in bag.read_messages():  # topics=[robot_namespace + '/lamp/pose_graph']):
        # if topic == '/' + robot_namespace + '/lamp/pose_graph':
        if topic == '/' + robot_namespace + '/lamp/pose_graph':
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
        # print new_node.x, new_node.y, new_node.z

    return nodes


def ReadArtifactsFromBagfile(filename, robot_namespace):
    '''
    Read a pose graph in g2o format from a bagfile.
    Extracts nodes only, edges are ignored
    '''

    bag = rosbag.Bag(filename)

    msgs, artifacts = [], []

    for topic, msg, t in bag.read_messages(topics=robot_namespace + '/unreconciled_artifact'):
        msgs.append(msg)

        new_artifact = Artifact()
        new_artifact.InitFromMsg(msg)
        artifacts.append(new_artifact)

    return artifacts


def PlotPoseGraphs2D(nodes_data, artifacts_data, artifacts_gt):
    '''
    Display two sets of pose graph nodes on a single 2D figure.
    '''

    fig = plt.figure()
    ax = fig.gca()

    x1 = [g.x for g in nodes_data]
    y1 = [g.y for g in nodes_data]

    ax.plot(x1, y1, label='LAMP Trajectory')
    coords_art = {'Backpack': ['r', [], []], 'Survivor': ['y', [], []], 'Helmet': ['k', [], []], 'Rope': ['b', [], []]}
    coords_gt = {'Backpack': ['r', [], []], 'Survivor': ['y', [], []], 'Helmet': ['k', [], []], 'Rope': ['b', [], []]}
    for a in artifacts_data:
        coords_art[a.label][1].append(a.x)
        coords_art[a.label][2].append(a.y)
    for a in artifacts_gt:
        coords_gt[a.label][1].append(a.x)
        coords_gt[a.label][2].append(a.y)
    for label, coords in coords_art.iteritems():
        ax.scatter(coords[1], coords[2], label=label, color=coords[0], s=30)
    for label, coords in coords_gt.iteritems():
        ax.scatter(coords[1], coords[2], color=coords[0], marker='*', s=250)
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(b=True, which='both', color='0.65', linestyle='-')
    plt.axis('equal')

    plt.legend()


def Analyze(nodes, artifacts, fid_cal_file):
    with open(fid_cal_file, 'r') as f:
        data = yaml.safe_load(f)
        t = data['fiducial_calibration']['position']
        r = data['fiducial_calibration']['orientation']
        world_T_map_rot = tf.transformations.quaternion_matrix([r['x'], r['y'], r['z'], r['w']])
        world_T_map_tr = tf.transformations.translation_matrix([t['x'], t['y'], t['z']])
        world_T_map = tf.transformations.concatenate_matrices(world_T_map_tr, world_T_map_rot)

    artifacts_world = []
    for a in artifacts:
        prev, curr, fract = GetClosestPoseGraphNodes(nodes, RosTimeToSecs(a.stamp))
        if prev is not None and curr is not None:
            artifact_world = GetArtifactWorld(prev, curr, fract, a, world_T_map)
            # print artifact_world.label, artifact_world.x, artifact_world.y, artifact_world.z
            artifacts_world.append(artifact_world)

    return artifacts_world


def RosTimeToSecs(stamp_rospy):
    return stamp_rospy.to_sec()


def PerformAnalysis(abspath, sim_world_path, pg_file, artifact_file, artifacts_gt_file, robot_namespace, fid_cal_data, sim_world_file):

    nodes = ReadPoseGraphFromBagfile(abspath + pg_file, robot_namespace)
    artifacts = ReadArtifactsFromBagfile(abspath + artifact_file, robot_namespace)

    artifacts_world = Analyze(nodes, artifacts, abspath + fid_cal_file)

    # get GT from world file
    with open(sim_world_path + sim_world_file, 'r') as f:
        data = xmltodict.parse(f)
        artifacts_gt = []
        objects = json.loads(json.dumps(data))['sdf']['world']['include']
        for x in objects: 
            if 'name' in x:
                if IsArtifact(x['name']):
                    new_artifact = Artifact()
                    label = ArtifactLabelFromName(name.encode('ascii'))
                    coords = [float(c) for c in x['pose'].encode('ascii').split(' ')[0:3]]
                    new_artifact.InitFromLabelXYZ(label, coords[0], coords[1], coords[2])
                    artifacts_gt.append(new_artifact)                

    PlotPoseGraphs2D(nodes, artifacts_world, artifacts_gt)

    # show all plots
    plt.show()


if __name__ == '__main__':

    abspath = '/data/ros/april-milestone-r3/'
    sim_world_path = '/home/costar/Projects/husky_ws/src/husky_sim/simulation_world/subt_custom_gazebo/worlds/'
    pg_file = 'rosbag/husky1_lamp_2020-04-30-21-25-40_0.bag'
    robot_namespace = 'husky1'
    artifact_file = 'rosbag/artifacts.bag'
    artifact_gt_file = 'artifact_gt.yaml'
    fid_cal_file = robot_namespace + '_rosparam.yaml'
    sim_world_file = 'simple_cave_02.world'

    PerformAnalysis(abspath, sim_world_path, pg_file, artifact_file, artifact_gt_file, robot_namespace, fid_cal_file, sim_world_file)
