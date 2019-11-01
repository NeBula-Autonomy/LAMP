#!/usr/bin/env python

# SETUP INSTRUCTIONS
# Add an alias to .bashrc:
#
#   alias mlc='python3 ~/Desktop/manual_LC_publish.py'
#
# Create a manual loop closure using e.g.
#
#   mlc a30 a5


import sys
import os

symbol_map = {'a': 6989586621679009792,
              'b': 7061644215716937728,
              'c': 7133701809754865664,
              'd': 7205759403792793600,
              'e': 7277816997830721536,
              'f': 7349874591868649472}


def GetGtsamSymbol(symb):
    chr = symb[0]
    ind = int(symb[1:])

    return symbol_map[chr] + ind


key_from = sys.argv[1]
key_to = sys.argv[2]

string = """
rostopic pub /base_station/manual_loop_closure pose_graph_msgs/PoseGraph "header:
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
  range_error: 0.0
priors: []"
""".format(GetGtsamSymbol(key_from), GetGtsamSymbol(key_to))

print(string)
os.system(string)
