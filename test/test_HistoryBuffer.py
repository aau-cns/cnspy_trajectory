#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2024, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
import unittest
import os
from cnspy_trajectory.Trajectory  import *
from cnspy_trajectory.HistoryBuffer import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class HistoryBuffer_Test(unittest.TestCase):
    @staticmethod
    def get_dict() -> HistoryBuffer:
        t_vec = [0, 1, 2, 3, 4, 5]
        dict_t = dict()
        for t in t_vec:
            dict_t[t] = t
        return HistoryBuffer(dict_t=dict_t)


    def test_CTOR(self):
        t_vec = [0, 1, 2, 3, 4, 5]
        dict_t = dict()
        for t in t_vec:
            dict_t[t] = t
        hist = HistoryBuffer(dict_t=dict_t)

        self.assertEqual(len(t_vec), len(hist.t_vec))  # add assertion here

    def test_indexing(self):
        hist = HistoryBuffer_Test.get_dict()

        t_vec = [0, 1, 2, 3, 4, 5]
        for i in range(0, len(t_vec)):
            self.assertTrue(hist.get_idx_at_t(i) == i)
        for i in range(1, len(t_vec)):
            self.assertTrue(hist.get_idx_before_t(i) == i-1)
        for i in range(1, len(t_vec)):
            self.assertTrue(hist.get_idx_before_t(i-0.01) == i-1)
        for i in range(0, len(t_vec)-1):
            self.assertTrue(hist.get_idx_before_t(i+0.01) == i)
        for i in range(0, len(t_vec)-1):
            self.assertTrue(hist.get_idx_after_t(i) == i+1)
        for i in range(0, len(t_vec)-1):
            self.assertTrue(hist.get_idx_after_t(i+0.01) == i+1)
        for i in range(1, len(t_vec)):
            self.assertTrue(hist.get_idx_after_t(i-0.01) == i)

        self.assertTrue(hist.get_idx_at_t(-1) == -1)
        self.assertTrue(hist.get_idx_at_t(0.5) == -1)
        self.assertTrue(hist.get_idx_at_t(6) == -1)
        self.assertTrue(hist.get_idx_before_t(0) == -1)
        self.assertTrue(hist.get_idx_after_t(5) == -1)



    def test_traj(self):
        traj = Trajectory(fn=SAMPLE_DATA_DIR+'/pose-raw.csv')
        hist = HistoryBuffer()
        hist.set(t_vec_=traj.t_vec[:,0], val_vec_=traj.p_vec)


        self.assertEqual(traj.num_elems(), len(hist.t_vec))  # add assertion here


if __name__ == '__main__':
    unittest.main()
