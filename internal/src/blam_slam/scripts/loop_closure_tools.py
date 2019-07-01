#!/usr/bin/env python
import rospy, transforms3d
from blam_slam.srv import SaveGraph, LoadGraph, RemoveFactor, AddFactor, Restart, BatchLoopClosure
from blam_slam.msg import RemoveFactorCmd, AddFactorCmd
from std_msgs.msg import String, Empty, Bool
from pose_graph_visualizer.srv import HighlightEdge


class LoopClosureTools:
    def __init__(self):
        self.robot_namespace = rospy.get_namespace()
        self.node_name = rospy.get_name()
        rospy.Subscriber(self.node_name + '/batch_loop_closure', Empty, self.batch_loop_closure_clbk, queue_size=1)
        rospy.Subscriber(self.node_name + '/save_graph', String, self.save_graph_clbk, queue_size=1)
        rospy.Subscriber(self.node_name + '/load_graph', String, self.load_graph_clbk, queue_size=1)
        rospy.Subscriber(self.node_name + '/add_factor', AddFactorCmd, self.add_factor_clbk, queue_size=1)
        rospy.Subscriber(self.node_name + '/remove_factor', RemoveFactorCmd, self.remove_factor_clbk, queue_size=1)
        rospy.Subscriber(self.node_name + '/restart', Empty, self.restart_clbk, queue_size=1)
        rospy.Subscriber(self.node_name + '/action_confirmation', Bool, self.confirmation_clbk, queue_size=1)
        rospy.loginfo("Loop closure tools initialized!")
        self.confirmation_received = False
        self.confirmation_status = False


    def batch_loop_closure_clbk(self, msg):
        rospy.loginfo("Batch loop closure command received")
        batch_loop_closure_service = rospy.ServiceProxy(self.robot_namespace + '/blam_slam/batch_loop_closure', BatchLoopClosure)
        if batch_loop_closure_service().success:
            print('Successfully found loop closures')
        else:
            print('Did not find any loop closures')


    def save_graph_clbk(self, msg):
        rospy.loginfo("Save graph command received")
        filename = msg.data
        save_graph_service = rospy.ServiceProxy(self.robot_namespace + '/blam_slam/save_graph', SaveGraph)
        if save_graph_service(filename).success:
            print('Successfully saved the pose graph to %s.' % filename)
        else:
            print('An error occurred while trying to save the pose graph to %s.' % filename)

    def add_factor_clbk(self, msg):
        rospy.loginfo("Add factor command received")
        prefix_from = msg.prefix_from
        key_from = msg.key_from
        prefix_to = msg.prefix_to
        key_to = msg.key_to
        quat = transforms3d.euler.euler2quat(msg.roll, msg.pitch, msg.yaw)
        add_factor_service = rospy.ServiceProxy(self.robot_namespace + '/blam_slam/add_factor', AddFactor)
        highlight_edge = rospy.ServiceProxy(self.robot_namespace + '/pose_graph_visualizer/highlight_edge', HighlightEdge)
        response = highlight_edge(prefix_from, key_from, prefix_to, key_to, True)
        if response.success:
            while not self.confirmation_received:
                pass
            self.set_confirmation_received_to_false()
            if self.confirmation_status:
                highlight_edge(prefix_from, key_from, prefix_to, key_to, False)  # remove edge visualization
                response = add_factor_service(prefix_from, key_from, prefix_to, key_to, quat[0], quat[1], quat[2], quat[3], True)
                if response.success:
                    print('Successfully added a factor between %i and %i to the graph.' % (key_from, key_to))
                else:
                    print('An error occurred while trying to add a factor between %i and %i.' % (key_from, key_to))
                    highlight_edge(key_from, key_to, False)
            else:
                print('Aborted manual loop closure.')
                highlight_edge(key_from, key_to, False)  # remove edge visualization  # remove edge visualization
        else:
            print('Error: One or more of the keys %i and %i do not exist.' % (key_from, key_to))
            highlight_edge(key_from, key_to, False)  # remove edge visualization  # remove edge visualization

    def load_graph_clbk(self, msg):
        rospy.loginfo("Load graph command received")
        filename = msg.data
        load_graph_service = rospy.ServiceProxy(self.robot_namespace + '/blam_slam/load_graph', LoadGraph)
        if load_graph_service(filename).success:
            print('Successfully loaded from last saved graph')
        else:
            print('Error: posegraph_backup.zip missing from directory or has been corrupted')

    def remove_factor_clbk(self, msg):
        rospy.loginfo("Remove factor command received")
        prefix_from = msg.prefix_from
        key_from = msg.key_from
        prefix_to = msg.prefix_to
        key_to = msg.key_to
        remove_factor_service = rospy.ServiceProxy(self.robot_namespace + '/blam_slam/remove_factor', RemoveFactor)
        highlight_edge = rospy.ServiceProxy(self.robot_namespace + '/pose_graph_visualizer/highlight_edge', HighlightEdge)
        response = highlight_edge(prefix_from, key_from, prefix_to, key_to, True)
        if response.success:
            while not self.confirmation_received:
                pass
            self.set_confirmation_received_to_false()
            if self.confirmation_status:
                highlight_edge(prefix_from, key_from, prefix_to, key_to, False)  # remove edge visualization
                response = remove_factor_service(prefix_from, key_from, prefix_to, key_to, True)
                if response.success:
                    print('Successfully removed a factor between %i and %i from the graph.' % (key_from, key_to))
                else:
                    print('An error occurred while trying to remove a factor between %i and %i.' % (key_from, key_to))
                    highlight_edge(key_from, key_to, False)
            else:
                print('Aborted manual loop closure.')
                highlight_edge(key_from, key_to, False)  # remove edge visualization # remove edge visualization
        else:
            print('Error: One or more of the keys %i and %i do not exist, or they are adjacent.' % (key_from, key_to))
            highlight_edge(key_from, key_to, False)  # remove edge visualization # remove edge visualization

    def restart_clbk(self, msg):
        rospy.loginfo("Restart command received")
        restart_service = rospy.ServiceProxy(self.robot_namespace + '/blam_slam/restart', Restart)
        if restart_service('posegraph_backup.zip').success:
            print('Successfully restarted from last saved graph')
        else:
            print('Error: posegraph_backup.zip missing from directory or has been corrupted')

    def confirmation_clbk(self, msg):
        self.confirmation_status = msg.data
        self.confirmation_received = True

    def set_confirmation_received_to_false(self):
        self.confirmation_received = False


def main():
    rospy.init_node('loop_closure_tools')

    LoopClosureTools()
    rospy.spin()


if __name__ == '__main__':
    main()
