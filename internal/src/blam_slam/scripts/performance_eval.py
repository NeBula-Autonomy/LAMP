#!/usr/bin/env python

import sys
import math

import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool

## Compares blam output to wheel odometry output

class PerformanceEvaluation:
    def __init__(self, delta_speed_threshold):
        self.robot_namespace = rospy.get_namespace()
        self.node_name = rospy.get_name()
        rospy.Subscriber('/blam_output', PoseStamped, self.get_blam_pose_clbk, queue_size=1)
        rospy.Subscriber('/odometry', Odometry, self.get_odom_clbk, queue_size=1)

        self.lidar_slip_pub = rospy.Publisher(self.node_name + '/lidar_slip', Float64, queue_size=10)
        self.slip_bool_pub = rospy.Publisher(self.node_name + '/is_slipping', Bool, queue_size=10)
        rospy.loginfo("Performance evaluation node initialized!")

        self.last_blam_pose = None
        self.odom_speed = None
        self.odom_pose = None
        self.slip_start_odom_pose = None
        self.slip_start_blam_pose = None
        self.threshold = delta_speed_threshold

        self.slipping = False
        self.slip = 0

        self.pub_cycle = 0
        self.pub_horizon = 900

    def increment_slip(self):
		odom_traveled = math.sqrt((self.odom_pose.position.x - self.slip_start_odom_pose.position.x)**2 + \
			(self.odom_pose.position.y - self.slip_start_odom_pose.position.y)**2 + \
			(self.odom_pose.position.z - self.slip_start_odom_pose.position.z)**2)
		blam_traveled = math.sqrt((self.last_blam_pose.pose.position.x - self.slip_start_blam_pose.pose.position.x)**2 + \
			(self.last_blam_pose.pose.position.y - self.slip_start_blam_pose.pose.position.y)**2 + \
			(self.last_blam_pose.pose.position.z - self.slip_start_blam_pose.pose.position.z)**2)
		slip = odom_traveled - blam_traveled
		if slip > 0.0:
			self.slip = slip
			self.slipping = True
			return True
		self.slipping = False
		return False

    def get_odom_clbk(self, msg):
		self.odom_speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 + msg.twist.twist.linear.z**2)
		self.odom_pose = msg.pose.pose;

    def get_blam_pose_clbk(self, msg):
    	if self.last_blam_pose is not None and self.odom_speed is not None:
    		# first calculate speed from last pose
	    	trans_dist = math.sqrt((msg.pose.position.x - self.last_blam_pose.pose.position.x)**2 + \
	    		(msg.pose.position.y - self.last_blam_pose.pose.position.y)**2 + \
	    		(msg.pose.position.z - self.last_blam_pose.pose.position.z)**2)
	    	# calculate speed
	    	delta_t = (msg.header.stamp - self.last_blam_pose.header.stamp).to_sec()
	    	speed_from_blam = trans_dist / float(delta_t)
	    	# compare with odom speed
	    	if abs(self.odom_speed - speed_from_blam) > self.threshold:
	    		if self.increment_slip():
	    			self.pub_cycle = 0
	    	else:
	    		self.slipping = False

	    	# if not slipping update last no slip place
    		if not self.slipping:
    			# keep track of when slip starts
    			self.slip_start_odom_pose = self.odom_pose
    			self.slip_start_blam_pose = msg

    	else:
    		self.last_blam_pose = msg
    	# publish the previous slip
    	self.slip_bool_pub.publish(self.slipping)
    	if self.slip > 0 and self.pub_cycle < self.pub_horizon: # don't publish insignificant slip dist
    		self.lidar_slip_pub.publish(self.slip)
    		self.pub_cycle += 1

def main(slip_threshold):
    rospy.init_node('performance_evaluation')

    PerformanceEvaluation(slip_threshold)
    rospy.spin()


if __name__ == '__main__':
    main(float(sys.argv[1]))
