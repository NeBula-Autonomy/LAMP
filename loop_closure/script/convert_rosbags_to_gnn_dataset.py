#!/usr/bin/env python

# This script converts bagfiles stored at fname to readable datasets for pytorch geometric in out_location
# Need pose_graph_opt.bag

import csv

import rosbag
import os
import yaml
import tqdm

fname = "/media/chris/hdd3/more_bags"
out_location = "/media/chris/hdd3/more_training_data"



def dump_edges(edges, filename):
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile)
        row = ["key_from", "key_to", "type"]
        writer.writerow(row)
        for edge in edges:
            row = [edge.key_from, edge.key_to, edge.type]
            writer.writerow(row)


def dump_nodes(nodes, filename):
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile)
        row = ['key', 'id',
               'pose_x', 'pose_y', 'pose_z',
               'pose_q_x', 'pose_q_y', 'pose_q_z', 'pose_q_w',
               'cov_x', 'cov_y', 'cov_z',
               'cov_rX', 'cov_rY', 'cov_rZ']
        writer.writerow(row)
        for node in nodes:
            row = [node.key, node.ID,
                   node.pose.position.x, node.pose.position.y, node.pose.position.z,
                   node.pose.orientation.x, node.pose.orientation.y, node.pose.orientation.z, node.pose.orientation.w,
                   node.covariance[0], node.covariance[7], node.covariance[14],
                   node.covariance[21], node.covariance[28], node.covariance[35],"aaaaaaaa"]
            writer.writerow(row)

def makedirs(path):
    cur_path = ""
    directories = path.split("/")
    for directory in directories[1:]:
        cur_path = os.path.join(cur_path,directory)
        print(cur_path)
        try:
            os.makedirs(cur_path)
        except OSError as e:
            print(e)

def closest_time_before(dict,t):
    best_time_diff = None
    best_msg = None
    for time, msg in dict.items():
        if best_time_diff is None or (time < t and t  < best_time_diff):
            best_time_diff = t
            best_msg = msg
    return best_msg


for run in tqdm.tqdm(os.listdir(fname), "Bags"):
    bags = list(os.listdir(os.path.join(fname,run)))
    if len(bags) == 1:
        split_bags = False
    elif len(bags) == 2:
        split_bags = True
    else:
        print(run)
        raise Exception()


    # Make file locations
    pre_files_location = os.path.join(out_location, run,"pre")
    post_files_location = os.path.join(out_location, run,"post")
    if os.path.exists(pre_files_location):
        print("Skipping: " + fname)
        continue
    try:
        os.removedirs(pre_files_location)
    except OSError:
        pass
    os.makedirs(pre_files_location)
    try:
        os.removedirs(post_files_location)
    except:
        pass
    os.makedirs(post_files_location)
    if not split_bags:
        last_unoptimized_message = None
        optimized_topic = "/base1/lamp_pgo/optimized_values"
        unoptimized_topic = "/base1/lamp/pose_graph_to_optimize"
        bag_location = os.path.join(fname,run,bags[0])
        try:
            pgo_info_dict = yaml.load(rosbag.Bag(bag_location, 'r')._get_yaml_info())
        except rosbag.bag.ROSBagUnindexedException:
            continue
        if not pgo_info_dict['indexed']:
            continue

        print pgo_info_dict
        for topic, msg, t in tqdm.tqdm(rosbag.Bag(bag_location).read_messages(topics=[unoptimized_topic, optimized_topic]), desc="Messages",total=pgo_info_dict["messages"]):
            if topic == unoptimized_topic:
                last_unoptimized_message = msg
            if topic == optimized_topic:
                # Dump pre message
                dump_edges(last_unoptimized_message.edges, os.path.join(pre_files_location, str(t) + "-edges.csv"))
                dump_nodes(last_unoptimized_message.nodes, os.path.join(pre_files_location, str(t) + "-nodes.csv"))

                # Dump post messages
                dump_edges(msg.edges, os.path.join(post_files_location, str(t) + "-edges.csv"))
                dump_nodes(msg.nodes, os.path.join(post_files_location, str(t) + "-nodes.csv"))
    else:
        pose_graph_bag = os.path.join(fname,run,"pose_graph.bag")
        pose_graph_opt_bag = os.path.join(fname,run,"pose_graph_opt.bag")
        pg_info_dict = yaml.load(rosbag.Bag(pose_graph_bag, 'r')._get_yaml_info())
        print pg_info_dict
        pgo_info_dict = yaml.load(rosbag.Bag(pose_graph_opt_bag, 'r')._get_yaml_info())
        print pgo_info_dict
        last_time = None        
        for topic,msg,t in tqdm.tqdm(rosbag.Bag(pose_graph_opt_bag).read_messages(),desc="Reading Messages", total=pgo_info_dict["messages"]):
            pose_graph_msg = None
            for topic_pg,msg_pg,t_pg in rosbag.Bag(pose_graph_bag).read_messages(start_time=last_time):
                if t_pg <= t:
                    pose_graph_msg = msg_pg
                    last_time = t_pg
                else:
                    break

            dump_edges(pose_graph_msg.edges, os.path.join(pre_files_location, str(t) + "-edges.csv"))
            dump_nodes(pose_graph_msg.nodes, os.path.join(pre_files_location, str(t) + "-nodes.csv"))

            # Dump post messages
            dump_edges(msg.edges, os.path.join(post_files_location, str(t) + "-edges.csv"))
            dump_nodes(msg.nodes, os.path.join(post_files_location, str(t) + "-nodes.csv"))


