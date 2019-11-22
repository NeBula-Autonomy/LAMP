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
import ctypes
import numpy

keyBits = ctypes.sizeof(ctypes.c_uint64) * 8
chrBits = ctypes.sizeof(ctypes.c_ubyte) * 8
indexBits = keyBits - chrBits
chrMask = numpy.uint64(numpy.int64(~numpy.ubyte(0)) << indexBits)
indexMask = ~chrMask


def split_pg_key(pg_key):
    c_ = chr(numpy.ubyte(numpy.int64(numpy.uint64(pg_key) & chrMask) >> indexBits))
    j_ = numpy.uint64(pg_key) & indexMask
    return c_, j_
# In [79]: split_pg_key(7061644215716937728)
# Out[79]: ('b', 0)


def join_pg_key(c_, j_):
    return numpy.uint64(ord(c_) << indexBits) + numpy.uint64(j_)
# In [8]: join_pg_key('b',0)
# Out[8]: 7061644215716937728


def GetGtsamSymbol(symb):
    chr = symb[0]
    ind = int(symb[1:])

    return join_pg_key(chr, ind)


base = sys.argv[1]
key_from = sys.argv[2]
key_to = sys.argv[3]

string = """
rostopic pub /{}/manual_loop_closure pose_graph_msgs/PoseGraph "header:
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
