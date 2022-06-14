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
# numpy, matplotlib, cnspy_csv2dataframe, cnspy_trajectory
########################################################################################################################
import os
import unittest
import math

from cnspy_spatial_csv_formats.CSVSpatialFormat import CSVSpatialFormat
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.TrajectoryErrorType import TrajectoryErrorType
from cnspy_trajectory.TrajectoryPlotter import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class TrajectoryPlotter_Test(unittest.TestCase):
    def load_trajectory_gt_from_CSV(self):
        traj = Trajectory()
        traj.load_from_CSV(filename=str(SAMPLE_DATA_DIR + '/ID1-pose-gt.csv')) 
        self.assertFalse(traj.is_empty())
        return traj

    def load_trajectory_posorient_from_CSV(self):
        traj = TrajectoryEstimated()
        traj.load_from_CSV(filename=str(SAMPLE_DATA_DIR + '/ID1-pose-est-posorient-cov.csv'))
        self.assertFalse(traj.is_empty())
        return traj

    def load_gt_est_err(self):
        traj_gt = Trajectory()
        traj_gt.load_from_CSV(filename=str(SAMPLE_DATA_DIR + '/ID1-pose-gt.csv'))
        traj_est = TrajectoryEstimated()
        traj_est.load_from_CSV(filename=str(SAMPLE_DATA_DIR + '/ID1-pose-est-posorient-cov.csv'))
        traj_est.format = CSVSpatialFormat(fmt_type=CSVSpatialFormatType.PosOrientWithCov,
                                         est_err_type=EstimationErrorType.type5,
                                           err_rep_type=ErrorRepresentationType.R_small_theta)
        traj_err = TrajectoryError()
        traj_err.load_from_CSV(filename=str(SAMPLE_DATA_DIR + '/ID1-pose-err.csv'))
        traj_err.traj_err_type = TrajectoryErrorType(err_type=EstimationErrorType.type5)

        return traj_gt, traj_est, traj_err

    def test_plot_multi(self):

        traj_gt = self.load_trajectory_gt_from_CSV()
        traj_est = self.load_trajectory_posorient_from_CSV()
        cfg = TrajectoryPlotConfig(show=True,
                                   save_fn=str(SAMPLE_DATA_DIR + '/../../doc/multi.png'),
                                   view_angle=(40, 20))
        TrajectoryPlotter.multi_plot_3D([traj_gt, traj_est], cfg=cfg, name_list=['gt', 'est'])

    def test_full_error_plot(self):
        traj_gt, traj_est, traj_err = self.load_gt_est_err()
        cfg = TrajectoryPlotConfig(show=True,
                                   radians=False,
                                   close_figure=False,
                                   save_fn=str(SAMPLE_DATA_DIR + '/../../doc/traj_plotter_full_err.png'),
                                   view_angle=(40, 20))

        TrajectoryPlotter.plot_pose_err_cov(traj_est=traj_est, traj_err=traj_err, traj_gt=traj_gt, cfg=cfg)




if __name__ == "__main__":
    unittest.main()
