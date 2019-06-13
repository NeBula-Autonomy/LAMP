#!/usr/bin/env python

import sys
import rospy
from mesh_msgs.srv import *
from mesh_msgs.msg import *

def fake_drop_command(anchor_id, dropped_time):
    rospy.wait_for_service('/husky/drop_uwb_anchor')
    try:
        dropped_info = rospy.ServiceProxy('/husky/drop_uwb_anchor', ProcessCommNode)
        node = mesh_msgs.msg.CommNode()
        node.AnchorID = anchor_id
        node.DropTime = dropped_time
        response = dropped_info(node)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def main():
    rospy.init_node('fake_uwb_drop_node')
    if len(sys.argv) == 2:
        anchor_id = sys.argv[1]
        dropped_time = rospy.get_rostime()
    else:
        sys.exit(1)
    fake_drop_command(anchor_id, dropped_time)

if __name__ == "__main__":
    main()