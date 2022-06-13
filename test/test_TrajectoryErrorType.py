#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2022, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.TrajectoryErrorType import TrajectoryErrorType
SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class TrajectoryErrorType_Test(unittest.TestCase):
    def test_list(self):
        err_type = TrajectoryErrorType(EstimationErrorType.type1)

        print('\n* type: ' + str(err_type))

        print("\n" + str(err_type.is_local_rotation_error()))
        print("\n" + str(err_type.error_def()))

        print("\n" + str(TrajectoryErrorType.list()))



if __name__ == '__main__':
    unittest.main()
