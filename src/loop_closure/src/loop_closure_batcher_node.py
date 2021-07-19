#!/usr/bin/env python

import os
#comment this if you want CUDA
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import rospy
from loop_closure_batcher import LoopClosureBatcher
import yaml
import rospkg


def initialize():
    rospack = rospkg.RosPack()
    cur_path = rospack.get_path("loop_closure")
    with open(os.path.join(cur_path,"config","loop_closure_batcher_parameters.yaml")) as f:
        try:
            params = yaml.load(f, Loader=yaml.FullLoader)
        except AttributeError:
            params = yaml.load(f)
    return params

if __name__ == "__main__":
    try:
        params = initialize()
        rospy.init_node("loop_closure_batcher")
        estimator = LoopClosureBatcher(params)
        estimator.create_publishers()
        estimator.register_callbacks()
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(str(e))
