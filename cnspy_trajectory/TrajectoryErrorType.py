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
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType


# TODO: Unftly, TrajectoryErrorType cannot inherit from EstimationErrorType due to restrictions in Enum!
class TrajectoryErrorType:
    err_type = None # EstimationErrorType

    def __init__(self, err_type=EstimationErrorType.type1):
        self.err_type = err_type

    def __str__(self):
        return self.err_type.__str__()

    def is_local_pose(self):
        return self.err_type == EstimationErrorType.type1

    def is_global_pose(self):
        return self.err_type == EstimationErrorType.type2

    def is_local_pose_inv(self):
        return self.err_type == EstimationErrorType.type3

    def is_global_pose_inv(self):
        return self.err_type == EstimationErrorType.type4

    def is_global_p_local_q(self):
        return self.err_type == EstimationErrorType.type5

    def is_local_p_global_q(self):
        return self.err_type == EstimationErrorType.type6

    def is_local_rotation_error(self):
        if self.err_type == EstimationErrorType.type1 or \
           self.err_type == EstimationErrorType.type3 or \
           self.err_type == EstimationErrorType.type5:
            return True

        return False

    def is_local_position_error(self):
        if self.err_type == EstimationErrorType.type1 or \
                self.err_type == EstimationErrorType.type3 or \
                self.err_type == EstimationErrorType.type5:
            return True
        return False

    def error_def(self):
        p_err_text = ''
        R_err_text = ''
        if self.err_type == EstimationErrorType.type5:
            p_err_text = '(p_EST - p_GT)'
            R_err_text = '(inv(R_GT) * R_EST)'
        elif self.err_type == EstimationErrorType.type1:
            p_err_text = '(R_EST^(T)(p_EST - p_GT))'
            R_err_text = '(inv(R_EST) * R_GT)'
        elif self.err_type == EstimationErrorType.type2:
            p_err_text = '(p_GT - R_ERR*p_EST)'
            R_err_text = '(R_GT * inv(R_EST))'
        return [p_err_text, R_err_text]

    @staticmethod
    def local_pose():
        return TrajectoryErrorType(err_type=EstimationErrorType.type1)

    @staticmethod
    def global_pose():
        return TrajectoryErrorType(err_type=EstimationErrorType.type2)

    @staticmethod
    def global_p_local_q():
        return TrajectoryErrorType(err_type=EstimationErrorType.type5)

    @staticmethod
    def local_p_global_q():
        return TrajectoryErrorType(err_type=EstimationErrorType.type6)

    @staticmethod
    def list():
        return EstimationErrorType.list()