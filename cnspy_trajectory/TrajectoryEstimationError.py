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
from sys import version_info

import numpy as np
import pandas

from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_csv2dataframe.PoseErrorStamped2DataFrame import PoseErrorStamped2DataFrame
from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.TrajectoryBase import TrajectoryBase


# - TODO: save and load to CSV file and DataFrame: introduce new format!
class TrajectoryEstimationError(TrajectoryBase):
    # p_vec: position error vector defined via "error_type" [m]
    # q_vec: rotation error defined via "error_type"  [quaternion]
    nu_vec = None     # position error converted into space and unit of the rotation covariance matrix
    theta_vec = None  # rotation error converted into space and unit of the rotation covariance matrix
    est_err_type = EstimationErrorType.none # EstimationErrorType
    err_rep_type = ErrorRepresentationType.none # ErrorRepresentationType

    def __init__(self, t_vec=None, nu_vec=None, theta_vec=None,
                 est_err_type=EstimationErrorType.none, err_rep_type=ErrorRepresentationType.none,
                 df=None, fn=None):
        TrajectoryBase.__init__(self)
        if df is not None:
            self.load_from_DataFrame(df)
        elif fn is not None:
            self.load_from_CSV(fn)
        elif t_vec is not None and nu_vec is not None and theta_vec is not None:
            if t_vec.ndim == 1:
                t_vec = np.array([t_vec])

            t_rows, t_cols = t_vec.shape
            nu_rows, nu_cols = nu_vec.shape
            theta_rows, theta_cols = theta_vec.shape
            assert (t_rows == nu_rows)
            assert (t_rows == theta_rows)
            assert (theta_cols == 3)
            assert (nu_cols == 3)

            self.nu_vec = nu_vec
            self.theta_vec = theta_vec
            self.t_vec = t_vec
            self.est_err_type = est_err_type
            self.err_rep_type = err_rep_type
        pass

    # overriding abstract method
    def subsample(self, step=None, num_max_points=None, verbose=False):
        sparse_indices = TrajectoryBase.subsample(self, step=step, num_max_points=num_max_points, verbose=verbose)
        self.nu_vec = self.nu_vec[sparse_indices]
        self.theta_vec = self.theta_vec[sparse_indices]

        return sparse_indices

    # overriding abstract method
    def sample(self, indices_arr, verbose=False):
        TrajectoryBase.sample(self, indices_arr=indices_arr)
        self.nu_vec = self.nu_vec[indices_arr]
        self.theta_vec = self.theta_vec[indices_arr]

    # overriding abstract method
    def clone(self):
        obj = TrajectoryEstimationError(t_vec=self.t_vec.copy(),
                                        nu_vec=self.nu_vec.copy(),
                                        theta_vec=self.theta_vec.copy(),
                                        est_err_type=self.est_err_type,
                                        err_rep_type=self.err_rep_type)
        return obj

    # overriding abstract method
    def load_from_DataFrame(self, df, fmt_type=None):
        assert (isinstance(df, pandas.DataFrame))
        if fmt_type is None:
            fmt_type = CSV2DataFrame.identify_format(dataframe=df)
        else:
            assert (isinstance(fmt_type, CSVSpatialFormatType))

        est_err_type = EstimationErrorType.none
        err_rep_type = ErrorRepresentationType.none
        if fmt_type == CSVSpatialFormatType.PoseErrorStamped:
            self.t_vec, self.nu_vec, self.theta_vec, est_err_type_vec, err_rep_vec = \
                PoseErrorStamped2DataFrame.from_DataFrame(df)
            est_err_type = EstimationErrorType(est_err_type_vec[0])
            err_rep_type = ErrorRepresentationType(err_rep_vec[0])

        self.est_err_type = est_err_type
        self.err_rep_type = err_rep_type

    # overriding abstract method
    def to_DataFrame(self):
        est_err_type_vec = np.tile(self.est_err_type.str(), (self.num_elems(), 1))
        err_rep_vec = np.tile(self.err_rep_type.str(), (self.num_elems(), 1))
        return PoseErrorStamped2DataFrame.to_DataFrame(self.t_vec, self.nu_vec, self.theta_vec,
                                                       est_err_type_vec=est_err_type_vec, err_rep_vec=err_rep_vec)
