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
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryEstimated import TrajectoryEstimated
import unittest
import time

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
        obj.load_from_CSV(filename=fn)
        return obj

    def test_load_trajectory_from_CSV(self):
        self.start()
        obj = self.load_()
        # self.assertTrue(obj.data_loaded)
        self.stop()

        print(obj.Sigma_p_vec[1000])
        print(obj.Sigma_p_vec[1000])
        self.start()
        obj.save_to_CSV(filename=str(SAMPLE_DATA_DIR + '/results/ID1-pose-est-cov-copy.csv'))
        self.stop()

    def test_load_pose_cov_traj(self):
        # TODO: does not work yet
        fn = str(SAMPLE_DATA_DIR + '/ID1-pose-est-pose-cov.csv')
        obj = TrajectoryEstimated()
        obj.load_from_CSV(filename=fn)
        print(obj.Sigma_T_vec[1])



if __name__ == "__main__":
    unittest.main()
