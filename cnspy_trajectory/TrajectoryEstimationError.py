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
from cnspy_trajectory.SpatialConverter import SpatialConverter
from cnspy_trajectory.TrajectoryBase import TrajectoryBase


# - TODO: save and load to CSV file and DataFrame: introduce new format!
class TrajectoryEstimationError(TrajectoryBase):
    # p_vec: position error vector defined via "error_type" [m]
    # q_vec: rotation error defined via "error_type"  [quaternion]
    nu_vec = None  # position error converted into space and unit of the rotation covariance matrix
    theta_vec = None  # rotation error converted into space and unit of the rotation covariance matrix
    est_err_type = EstimationErrorType.none  # EstimationErrorType
    err_rep_type = ErrorRepresentationType.none  # ErrorRepresentationType

    def __init__(self, t_vec=None,
                 nu_vec=None,
                 theta_vec=None,
                 p_vec=None,
                 q_vec=None,
                 est_err_type=EstimationErrorType.none,  # global or local
                 err_rep_type=ErrorRepresentationType.none,  # first order approx. or on tangent space
                 df=None, fn=None):
        TrajectoryBase.__init__(self)
        if df is not None:
            self.load_from_DataFrame(df)
        elif fn is not None:
            self.load_from_CSV(fn)
        elif t_vec is not None and nu_vec is not None and theta_vec is not None:
            self.set(t_vec, nu_vec, theta_vec)
            self.est_err_type = est_err_type
            self.err_rep_type = err_rep_type
        elif t_vec is not None and p_vec is not None and q_vec is not None:
            nu_vec, theta_vec = TrajectoryEstimationError.convert_error(p_vec, q_vec, err_rep_type)
            self.set(t_vec, nu_vec, theta_vec)
            self.est_err_type = est_err_type
            self.err_rep_type = err_rep_type
        pass

    @staticmethod
    def convert_error(p_vec=None,  q_vec=None,  err_rep_type=ErrorRepresentationType.none) -> (np.ndarray, np.ndarray):
        """
        Converts the trajectory in R(3) x SO(3) into the different error formats
        Parameters
        ----------
        p_vec
        q_vec expect HTMQ quaternion format [x,y,z,w]
        err_rep_type

        Returns nu_vec and theta_vec
        -------
        """
        assert (isinstance(err_rep_type, ErrorRepresentationType))

        if err_rep_type == ErrorRepresentationType.none:
            return p_vec, q_vec

        len, q_cols = q_vec.shape
        assert(q_cols == 4)
        theta_vec = np.zeros((len, 3))

        # nu_vec default: position remains unchanged
        nu_vec = p_vec

        if err_rep_type == ErrorRepresentationType.theta_R:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2theta_R(q_vec[i])
        elif err_rep_type == ErrorRepresentationType.theta_q:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2theta_q(q_vec[i])
        elif err_rep_type == ErrorRepresentationType.theta_so3:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2theta_so3(q_vec[i])
        elif err_rep_type == ErrorRepresentationType.rpy_rad:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2rpy(q_vec[i])
        elif err_rep_type == ErrorRepresentationType.tau_se3:
            nu_vec = np.zeros((len, 3))
            for i in range(len):
                tau_se3 = SpatialConverter.p_q_to_tau_se3(p_vec[i], q_vec[i])
                nu_vec[i, :] = tau_se3[0:3]
                theta_vec[i, :] = tau_se3[3:6]
        else:
            assert False, 'format is not supported: ' + str(err_rep_type)

        return nu_vec, theta_vec

    def set(self, t_vec: (list, np.ndarray), nu_vec: np.ndarray, theta_vec: np.ndarray):
        self.set_t_vec(t_vec)

        t_rows, t_cols = self.t_vec.shape
        nu_rows, nu_cols = nu_vec.shape
        theta_rows, theta_cols = theta_vec.shape
        assert (t_rows == nu_rows)
        assert (t_rows == theta_rows)
        assert (theta_cols == 3)
        assert (nu_cols == 3)

        self.nu_vec = nu_vec
        self.theta_vec = theta_vec

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
