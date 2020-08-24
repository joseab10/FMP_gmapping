# ROS Libraries
import rospy
import roslib
import tf

# ROS Messages
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from tf2_msgs.msg import TFMessage
from gmapping.msg import doubleMap

# Math Libraries
import numpy as np
import matplotlib.pyplot as plt


class SlamEval:
    def __init__(self):

        rospy.init_node('slam_eval', anonymous=True)

        self._tf_listener = tf.TransformListener()

        self._alpha_map_sequence = 0
        self._beta_map_sequence = 0
        rospy.Subscriber("fmp_alpha", doubleMap, self._map2d_alpha_callback)
        rospy.Subscriber("fmp_beta", doubleMap, self._map2d_beta_callback)

        rospy.spin()

    def _tf_callback(self, data):
        pass

    def _plot_fmp(self):
        if self._fmp_acquired():
            pass

    def _fmp_acquired(self):
        return self._alpha_map_sequence == self._beta_map_sequence

    def _map2d_alpha_callback(self, data):
        self._alpha_map_sequence = data.header.sequence


        self._plot_fmp()

    def _map2d_beta_callback(self, data):
        self._beta_map_sequence =  data.header.sequence

        self._plot_fmp()



if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="Generate a ROSbag file from a simulated robot trajectory.")

    parser.add_argument('-m', '--map_model', action='store', help='Input JSON robot config file', choices=["decay", "reflection"], default=)
    parser.add_argument('-o', '--output', action='store', help='Output ROSbag file', type=str, required=False)

    parser.add_argument('-p', '--preview', action='store_true')
    parser.add_argument('-s', '--search_paths', action='store', help='Search paths for the input and include files separated by colons (:)', type=str, default='.:robots:maps')
    #parser.add_argument('extra_params', nargs='*')


