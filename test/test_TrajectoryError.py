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
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryError import TrajectoryError
from cnspy_trajectory.TrajectoryErrorType import TrajectoryErrorType
from cnspy_trajectory.TrajectoryEstimated import TrajectoryEstimated
import unittest
import time

from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class TrajectoryError_Test(unittest.TestCase):
    start_time = None

    def start(self):
        self.start_time = time.time()

    def stop(self):
        print("Process time: " + str((time.time() - self.start_time)))

    def load_(self):
        print('loading...')
        fn = str(SAMPLE_DATA_DIR + '/ID1-pose-err.csv')
        obj = TrajectoryError(fn=fn)
        obj.traj_err_type = TrajectoryErrorType(err_type=EstimationErrorType.type1)
        return obj


    def test_plot(self):
        traj_err = self.load_()
        traj_err.save_to_CSV(str(SAMPLE_DATA_DIR + '/results/ID1-pose-err.COPY.csv'))
        cfg = TrajectoryPlotConfig(show=True, close_figure=False,
                                   save_fn=str(SAMPLE_DATA_DIR + '/../../doc/traj_err_pos.png'))
        traj_err.plot_p_err(cfg=cfg)
        cfg = TrajectoryPlotConfig(show=True, close_figure=False,
                                   save_fn=str(SAMPLE_DATA_DIR + '/../../doc/traj_err_rpy.png'))

        traj_err.plot_rpy_err(cfg=cfg)

