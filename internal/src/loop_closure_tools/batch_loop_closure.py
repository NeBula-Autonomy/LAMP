#!/usr/bin/env python
import rospy, sys
from blam_slam.srv import BatchLoopClosure


def connect():
    rospy.init_node('batch_loop_closure_client')
    batch_loop_closure = rospy.ServiceProxy('/husky/blam_slam/batch_loop_closure', BatchLoopClosure)
    if batch_loop_closure().success:
        print('Successfully found loop clousers')
    else:
        print('Did not find any loop colsures')

if __name__ == '__main__':
    try:
        if len(sys.argv) == 1:
            connect()
            


    except rospy.ROSInterruptException: pass
