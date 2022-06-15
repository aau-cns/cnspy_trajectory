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
# Requirements:
# enum
########################################################################################################################
import os

from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryEstimated import TrajectoryEstimated
import unittest
import time

from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class TrajectoryEstimated_Test(unittest.TestCase):
    start_time = None

    def start(self):
        self.start_time = time.time()

    def stop(self):
        print("Process time: " + str((time.time() - self.start_time)))

    def load_(self):
        print('loading...')
        fn = str(SAMPLE_DATA_DIR + '/ID1-pose-est-posorient-cov.csv')
        obj = TrajectoryEstimated()
        obj.load_from_CSV(fn=fn)
        obj.format.rotation_error_representation = ErrorRepresentationType.theta_R
        return obj

    def test_load_trajectory_from_CSV(self):
        self.start()
        obj = self.load_()
        # self.assertTrue(obj.data_loaded)
        self.stop()

        print('\n' + str(obj.Sigma_p_vec[1000]))
        print('\n' + str(obj.Sigma_p_vec[1000]))
        self.start()
        obj.save_to_CSV(fn=str(SAMPLE_DATA_DIR + '/results/ID1-pose-est-cov-copy.csv'))
        self.stop()

    def test_load_pose_cov_traj(self):
        fn = str(SAMPLE_DATA_DIR + '/ID1-pose-est-pose-cov.csv')
        obj = TrajectoryEstimated()
        obj.load_from_CSV(fn=fn)
        print('\n' + str(obj.Sigma_T_vec[1]))

    def test_plot_pos_orient_cov_theta_R(self):
        traj_est = self.load_()
        traj_est.format.rotation_error_representation = ErrorRepresentationType.theta_R
        cfg = TrajectoryPlotConfig(show=False, close_figure=False, radians=False,
                                   plot_type=TrajectoryPlotTypes.plot_2D_over_dist,
                                   save_fn=str(SAMPLE_DATA_DIR + '/../../doc/traj_est_pos_orient_cov_theta_R.png'))
        traj_est.plot_pos_orient_cov(angles=True, cfg=cfg)


    def test_plot_pos_orient_cov_theta_q(self):
        traj_est = self.load_()
        traj_est.format.rotation_error_representation = ErrorRepresentationType.theta_q
        cfg = TrajectoryPlotConfig(show=False, close_figure=False,
                                   save_fn=str(SAMPLE_DATA_DIR + '/../../doc/traj_est_pos_orient_cov_theta_q.png'))
        traj_est.plot_pos_orient_cov(angles=True, cfg=cfg)

    def test_plot_pos_orient_cov_theta_so3(self):
        traj_est = self.load_()
        traj_est.format.rotation_error_representation = ErrorRepresentationType.theta_so3
        cfg = TrajectoryPlotConfig(show=False, close_figure=False, radians=False,
                                   save_fn=str(SAMPLE_DATA_DIR + '/../../doc/traj_est_pos_orient_cov_theta_so3.png'))
        traj_est.plot_pos_orient_cov(angles=True, cfg=cfg)

if __name__ == "__main__":
    unittest.main()
