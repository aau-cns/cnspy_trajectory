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
from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.Trajectory import Trajectory


# - TODO: save and load to CSV file and DataFrame: introduce new format!
class TrajectoryEstimationError(Trajectory):
    # p_vec: position error vector defined via "error_type" [m]
    # q_vec: rotation error defined via "error_type"  [quaternion]

    est_err_type = None # EstimationErrorType
    err_rep_type = None # ErrorRepresentationType
    theta_q_vec = None  # rotation error converted into space and unit of the rotation covariance matrix

    def __init__(self, t_vec=None, p_vec=None, q_vec=None, theta_q_vec=None,
                 est_err_type=EstimationErrorType.type1,
                 err_rep_type=ErrorRepresentationType.R_small_theta):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)

        p_rows, p_cols = p_vec.shape
        theta_rows, theta_cols = theta_q_vec.shape
        assert (theta_rows == p_rows)
        assert (theta_cols == 3)

        self.est_err_type = est_err_type
        self.err_rep_type = err_rep_type
        self.theta_q_vec = theta_q_vec


