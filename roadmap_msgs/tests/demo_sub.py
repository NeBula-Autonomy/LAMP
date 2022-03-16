#!/usr/bin/env python
"""Demo to show how to subscribe and visualize roadmap."""

import rospy
from roadmap_msgs.msg import Roadmap
from roadmap_msgs import bridge

import networkx as nx
import matplotlib.pyplot as plt


g = nx.Graph()


def callback(msg):
    global g
    g = bridge.to_networkx(msg)
    print('New roadmap received: n={} e={}',
          g.number_of_nodes(),
          g.number_of_edges())


def main():
    rospy.init_node('demo_sub')
    rospy.Subscriber('roadmap', Roadmap, callback)

    plt.figure()

    while not rospy.is_shutdown():
        plt.clf()
        nx.draw_networkx(g, pos={n: (g.nodes[n]['pose'].position.x,
                                     g.nodes[n]['pose'].position.y)
                                 for n in g.nodes})
        plt.axis('equal')
        plt.show(block=False)
        plt.pause(0.05)


if __name__ == '__main__':
    main()
