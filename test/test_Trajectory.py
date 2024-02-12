#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
########################################################################################################################
import os
import unittest
import math
import numpy as np
from spatialmath import UnitQuaternion, SE3, SO3, Quaternion
import cnspy_trajectory as tr
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotter import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class Trajectory_Test(unittest.TestCase):
    def load_trajectory_from_CSV(self):
        print('loading...')
        traj = Trajectory()
        traj.load_from_CSV(fn=str(SAMPLE_DATA_DIR + '/ID1-pose-est.csv'))
        return traj

    def test_load_trajectory_from_CSV(self):
        traj = self.load_trajectory_from_CSV()
        self.assertTrue(traj.num_elems() > 0)

    def test_save_to_CSV(self):
        traj = Trajectory()

        saved = traj.save_to_CSV(str(SAMPLE_DATA_DIR + '/results/empty1.csv'))
        self.assertFalse(saved)

        traj.p_vec = np.array([[0, 0, 0],
                               [1, 0, 0],
                               [2, 0, 0],
                               [3, 0, 0]])
        traj.q_vec = np.array([[0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1]])
        traj.t_vec = np.array([[0],
                               [1],
                               [2],
                               [3]])
        saved = traj.save_to_CSV(str(SAMPLE_DATA_DIR + '/results/empty2.csv'))
        self.assertTrue(saved)

    def test_get_distance_from_start(self):
        p_vec = np.array([[0, 0, 0],
                          [1, 0, 0],
                          [2, 0, 0],
                          [3, 0, 0]])

        d = Trajectory.distance(p_vec)
        self.assertTrue(math.floor(d - 3.0) == 0)

        p_vec = np.array([[0, 0, 0],
                          [1, 1, 0],
                          [2, 2, 0],
                          [3, 3, 0]])
        d = Trajectory.distance(p_vec)
        d_ = math.sqrt(9 + 9)
        self.assertTrue(math.floor(d - d_) == 0)

        p_vec = np.array([[0, 0, 0],
                          [1, 1, 1],
                          [2, 2, 2],
                          [3, 3, 3]])

        d = Trajectory.distance(p_vec)
        d_ = math.sqrt(9 + 9 + 9)
        self.assertTrue(math.floor(d - d_) == 0)

    def test_get_rpy_vec(self):
        traj = self.load_trajectory_from_CSV()
        rpy_vec = traj.get_rpy_vec()
        self.assertTrue(rpy_vec.shape[0] == traj.num_elems())
        self.assertTrue(rpy_vec.shape[1] == 3)

    def test_get_angle_axis_vec(self):
        traj = self.load_trajectory_from_CSV()
        phi_vec, axis_vec_ = traj.get_angle_axis_vec(unit='rad')
        phi_vec_, axis_vec = traj.get_angle_axis_vec(unit='deg')
        self.assertTrue(axis_vec.shape[0] == traj.num_elems())
        self.assertTrue(axis_vec.shape[1] == 3)
        self.assertTrue(phi_vec.shape[0] == traj.num_elems())
        self.assertTrue(phi_vec.shape[1] == 1)

    def test_transform(self):
        traj = self.load_trajectory_from_CSV()
        self.assertTrue(traj.num_elems() > 0)
        d = traj.get_distance()
        print('distance: ' + str(d))

        R1 = SO3.RPY([0, 0, 45], unit='deg')
        p = np.array([1, 2, 3])
        traj.transform(1, p, R1.R)

        d2 = traj.get_distance()
        print('distance2:  ' + str(d2))

    def test_transform2(self):
        traj = Trajectory()

        saved = traj.save_to_CSV(str(SAMPLE_DATA_DIR + '/results/empty1.csv'))
        self.assertFalse(saved)

        traj.q_vec = np.array([[0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1]])
        traj.t_vec = np.array([[0],
                               [1],
                               [2],
                               [3], [4], [5], [6]])

        p_GB_in_G_arr = np.array(
            [[0.029, 0.0165, -2.51], [0.0303, 0.3367, -2.511], [0.038, 1.221, -2.516], [0.03115, 1.591, -2.61],
             [0.023, 2.416, -2.819], [-0.03825, 2.79245, -2.82835], [-0.1567, 3.54785, -2.8475]])

        traj.p_vec = p_GB_in_G_arr

        traj2 = Trajectory()
        p_NB_in_N_arr = np.array(
            [[-2.694, -0.049, 0.0205], [-2.696, -0.0475, 0.369], [-2.6935, -0.0435, 1.2225], [-2.623, 0.0175, 1.589],
             [-2.4685, 0.154, 2.403], [-2.4205, 0.123, 2.7725], [-2.317, 0.051, 3.522]])
        traj2.q_vec = np.array([[0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1],
                               [0, 0, 0, 1]])
        traj2.t_vec = np.array([[0],
                               [1],
                               [2],
                               [3], [4], [5], [6]])
        traj2.p_vec = p_NB_in_N_arr


        cfg=TrajectoryPlotConfig(show=True, close_figure=False, save_fn='')
        #fig, ax = traj.plot_3D(cfg=cfg)
        #traj2.plot_3D(cfg=cfg)

        R_GN = np.array([[ 0.92866884,  0.31819331, -0.19059698],       [ 0.218043  , -0.05263923,  0.97451853],       [ 0.3000524 , -0.94656333, -0.11826423]])
        t_GN = np.array([ 2.69581276,  0.61577898, -1.66060676])

        #R_GN = np.array([[ 0.94404031,  0.12013279, -0.30717424], [-0.30010691, -0.07350927, -0.95106899], [-0.13683473,  0.99003257, -0.03334304]])
        #t_GN = np.array([-3.56530399, -2.3538825 , -0.06128361])
        traj3 = traj2.clone()
        traj3.transform(scale=1, R_GN=R_GN, p_GN_in_G=t_GN)
        TrajectoryPlotter.multi_plot_3D([traj, traj2, traj3], cfg=cfg, name_list=['gt', 'est', 'aligned'])





    def test_plot_3D(self):
        traj = self.load_trajectory_from_CSV()
        cfg=TrajectoryPlotConfig(show=True, close_figure=False, save_fn=str(SAMPLE_DATA_DIR + '/../../doc/plot_3D.png'))
        traj.plot_3D(cfg=cfg)

    def test_plot_pose(self):
        traj = self.load_trajectory_from_CSV()
        cfg=TrajectoryPlotConfig(show=False, close_figure=False, save_fn=str(SAMPLE_DATA_DIR + '/../../doc/pose.png'))
        traj.plot_pose(cfg=cfg)
        traj.plot_pose(cfg=cfg, quaternions=False)
        cfg.radians = False
        traj.plot_pose(cfg=cfg, quaternions=False)

        traj.plot_pose(cfg=cfg, plot_angle_distance=True)
        traj.plot_pose(cfg=cfg, plot_angle_distance=True)

        traj.plot_pose(quaternions=True, cfg=TrajectoryPlotConfig(show=False,
                                                                  close_figure=False,
                                                                  radians=False,
                                                                  plot_type=TrajectoryPlotTypes.plot_2D_over_dist))
        traj.plot_pose(quaternions=True, cfg=TrajectoryPlotConfig(show=True,
                                                                  close_figure=False,
                                                                  radians=False,
                                                                  plot_type=TrajectoryPlotTypes.plot_2D_over_t))
if __name__ == "__main__":
    unittest.main()
