#!/usr/bin/env python

# SETUP INSTRUCTIONS
# Add an alias to .bashrc:
#
#   alias mlc='python3 ~/Desktop/manual_LC_publish.py'
#
# Create a manual loop closure using e.g.
#
#   mlc base1 a30 a5


import sys
import os
import key_handling


def GetGtsamSymbol(symb):
    chr = symb[0]
    ind = int(symb[1:])

    return key_handling.join_pg_key(chr, ind)


def PublishManualLoopClosure(base, key_from, key_to):

    string = """
    rostopic pub /{}/manual_loop_closure pose_graph_msgs/PoseGraph "
    header:
      seq: 0
      stamp: {{secs: 0, nsecs: 0}}
      frame_id: ''
    incremental: false
    nodes: []
    edges:
    - key_from: {}
      key_to: {}
      type: 0
      pose:
        position: {{x: 0.0, y: 0.0, z: 0.0}}
        orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 0.0}}
      covariance: [100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
      range: 0.0
      range_error: 0.0"
    """.format(base, GetGtsamSymbol(key_from), GetGtsamSymbol(key_to))

    print(string)
    os.system(string)


if __name__ == '__main__':

    base = sys.argv[1]
    key_from = sys.argv[2]
    key_to = sys.argv[3]

    PublishManualLoopClosure(base, key_from, key_to)
