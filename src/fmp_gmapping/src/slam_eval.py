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
#import matplotlib
#matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from enum import Enum


class FigNum(Enum):
    RAW = 1
    PARAM_A = 2
    PARAM_B = 3
    MEAN = 4
    VAR = 5
    FMP = 6
    MLM = 7


cdict_mlm_beta = {
    "red":   ((0.000, 1.0, 1.0),
              (0.125, 0.5, 0.5),
              (0.250, 1.0, 1.0),
              (0.375, 0.0, 0.0),
              (0.500, 1.0, 1.0),
              (1.000, 0.0, 0.0)),

    "green": ((0.000, 1.0, 1.0),
              (0.125, 0.0, 0.0),
              (0.250, 1.0, 1.0),
              (0.375, 1.0, 1.0),
              (0.500, 1.0, 1.0),
              (1.000, 0.0, 0.0)),

    "blue":  ((0.000, 1.0, 1.0),
              (0.125, 1.0, 1.0),
              (0.250, 1.0, 1.0),
              (0.375, 0.0, 0.0),
              (0.500, 1.0, 1.0),
              (1.000, 0.0, 0.0)),

    "alpha": ((0.000, 1.0, 1.0),
              (0.125, 1.0, 1.0),
              (0.250, 1.0, 1.0),
              (0.375, 1.0, 1.0),
              (0.500, 1.0, 1.0),
              (1.000, 1.0, 1.0))
}

cm_mlm_betadist = LinearSegmentedColormap('BetaDistMLM', cdict_mlm_beta)


class SlamEval:
    def __init__(self):

        rospy.init_node('fmp_plot', anonymous=True)

        self._tf_listener = tf.TransformListener()

        self._alpha_map_sequence = -1
        self._beta_map_sequence = -2
        self._alpha_map = None
        self._beta_map = None

        self._fmp_param1_topic = rospy.get_param("~fmp_param1_topic", "fmp_alpha")
        self._fmp_param2_topic = rospy.get_param("~fmp_param2_topic", "fmp_beta")

        self._decay_model = rospy.get_param("~decay_model", False)

        self._plot_fmp = rospy.get_param("~plot_fmp", True)
        self._plot_mean_var = rospy.get_param("~plot_mean_var", True)
        self._plot_mlm = rospy.get_param("~plot_mlk", True)
        self._plot_params = rospy.get_param("~plot_params", True)

        self._alpha_prior = 1
        self._beta_prior = 0 if self._decay_model else 1

        rospy.Subscriber(self._fmp_param1_topic, doubleMap, self._map2d_alpha_callback)
        rospy.Subscriber(self._fmp_param2_topic, doubleMap, self._map2d_beta_callback)

        rospy.spin()

    def _tf_callback(self, data):
        pass

    def _fmp_acquired(self):
        return self._alpha_map_sequence == self._beta_map_sequence

    def _plot_full_posterior(self):

        if not self._plot_fmp:
            return None

        #fig = plt.figure()


        #plt.plot()

    def _plot_stat(self):
        if not self._plot_mean_var:
            return

        alpha = self._alpha_prior + self._alpha_map
        beta  = self._beta_prior + self._beta_map

        if self._decay_model:
            means = alpha / beta
            var   = means / beta

            max_x = np.max(means) + (3 * np.max(var))

        else:
            a_plus_b = alpha + beta

            means = alpha / a_plus_b
            var = (alpha * beta) / ((a_plus_b**2) * (a_plus_b + 1))

            max_x = 1.0

        # Normalize variance from 0 to max(var) for coloring
        #var = var / np.max(var)

        #rgba =

        plt.figure()
        plt.imshow(means)
        plt.show()

        plt.figure()
        plt.imshow(var)
        plt.show()



    def _plot_most_likely_map(self):

        if not self._plot_mlm:
            return

        alpha = self._alpha_prior + self._alpha_map
        beta  = self._beta_prior  + self._beta_map

        if self._decay_model: # Gamma Distribution
            mlm = np.max(0, (alpha - 1) / beta)
            mlm = mlm / np.max(mlm) # Normalize from 0 to the maximum mode value

            cm = 'hot'

        else: # Beta Distribution
            mlm = np.zeros_like(alpha)

            mlm[alpha > 1  and beta > 1  ] = (alpha - 1) / (alpha + beta - 2)
            mlm[alpha <= 1 and beta > 1] = 0
            mlm[alpha > 1 and beta <= 1] = 1

            mlm = (mlm + 1) / 2 # Normalize likelihood to [0.5, 1.0] range

            mlm[alpha < 1  and beta < 1  ] = 0.125 # Bimodal distribution (Purple)
            mlm[alpha == 1 and betta == 1] = 0.375  # Uniform distribution (Green)

            cm = cm_mlm_betadist

        plt.figure()
        plt.imshow(mlm, cmap=cm)
        plt.show()

    def _plot_param(self, name, fignum, param):
        fig = plt.figure(fignum.value)
        ax = fig.add_subplot(111)
        ax.imshow(param, aspect=1)

        if self._decay_model:
            title = "Exponential Decay"
        else:
            title = "Reflection"
        title += " Model: Parameter " + name

        ax.set_title(title)
        plt.colorbar()
        fig.show()

    def _plot_param_ab(self):
        if not self._plot_params:
            return

        self._plot_param("alpha", FigNum.PARAM_A, self._alpha_map + self._alpha_prior)
        self._plot_param("beta", FigNum.PARAM_B, self._beta_map + self._beta_prior)

    def _plot_raw(self):
        pass

    def plot(self):

        self._plot_raw()

        if not self._fmp_acquired():
            return

        self._plot_param_ab()

        self._plot_full_posterior()
        self._plot_stat()
        self._plot_most_likely_map()

    def _map_reshape(self, data):

        w = data.info.width
        h = data.info.height

        map = np.array(data.data)
        map = map.reshape(w, h)

        return map

    def _map2d_alpha_callback(self, data):
        self._alpha_map_sequence = data.header.seq
        self._alpha_prior = data.param

        self._alpha_map = self._map_reshape(data)

        self.plot()

    def _map2d_beta_callback(self, data):
        self._beta_map_sequence = data.header.seq
        self._beta_prior = data.param

        self._beta_map = self._map_reshape(data)

        self.plot()





if __name__ == '__main__':
    #import argparse

    #parser = argparse.ArgumentParser(description="Generate a ROSbag file from a simulated robot trajectory.")

    #parser.add_argument('-m', '--map_model', action='store', help='Input JSON robot config file', choices=["decay", "reflection"], default=)
    #parser.add_argument('-o', '--output', action='store', help='Output ROSbag file', type=str, required=False)

    #parser.add_argument('-p', '--preview', action='store_true')
    #parser.add_argument('-s', '--search_paths', action='store', help='Search paths for the input and include files separated by colons (:)', type=str, default='.:robots:maps')
    #parser.add_argument('extra_params', nargs='*')

    slam_eval = SlamEval()



