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
########################################################################################################################
import os

import pandas
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_csv2dataframe.PosOrientWithCov2DataFrame import PosOrientWithCov2DataFrame
from cnspy_csv2dataframe.PoseWithCov2DataFrame import PoseWithCov2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.CSVSpatialFormat import CSVSpatialFormat
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType


class TrajectoryEstimated(Trajectory):
    # position uncertainty: 3x3 covariance matrix.
    # The with upper triangular elements are vectorized: 'pxx', 'pxy', 'pxz', 'pyy', 'pyz', 'pzz'
    Sigma_p_vec = None

    # small angle `theta` uncertainty: R = ( eye(3) + skew(theta_R)), a 3x3 covariance matrix.
    # The with upper triangular elements are vectorized in radiant: 'qrr', 'qrp', 'qry', 'qpp', 'qpy', 'qyy'
    Sigma_R_vec = None

    # correlation between position (p) and orientation (R): 3x3 covariance matrix.
    # The with upper triangular elements are vectorized: 'pxr', 'pxp', 'pxy', 'pyr', 'pyp', 'pyy', 'pzr', 'pzp', 'pzy'
    Sigma_pR_vec = None

    Sigma_T_vec = None

    # EstimationErrorType: specifies global or local definitions of the uncertainty (Sigma_p/q_vec)
    # ErrorRepresentationType: specifies the uncertainty of the rotation (Sigma_R_vec)
    format = CSVSpatialFormat(est_err_type=EstimationErrorType.type5,err_rep_type=ErrorRepresentationType.R_small_theta)

    def __init__(self, t_vec=None, p_vec=None, q_vec=None,
                 Sigma_p_vec=None, Sigma_R_vec=None,
                 Sigma_pR_vec=None, Sigma_T_vec=None,
                 df=None, fmt=None):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)
        self.Sigma_p_vec = Sigma_p_vec
        self.Sigma_R_vec = Sigma_R_vec
        self.Sigma_pR_vec = Sigma_pR_vec
        self.Sigma_T_vec = Sigma_T_vec

        if df is not None:
            self.load_from_DataFrame(df)

        if fmt is not None:
            self.set_format(fmt)

    def set_format(self, fmt):
        if fmt is not None and isinstance(fmt, CSVSpatialFormat):
            self.format = fmt

    def get_format(self):
        return self.format

    def load_from_CSV(self, filename):
        if not os.path.isfile(filename):
            print("Trajectory: could not find file %s" % os.path.abspath(filename))
            return False

        loader = CSV2DataFrame(filename=filename)
        if loader.data_loaded:
            self.load_from_DataFrame(df=loader.data_frame, fmt=loader.format)
            return True

        return False

    def load_from_DataFrame(self, df, fmt=None):
        assert (isinstance(df, pandas.DataFrame))
        if fmt is None:
            fmt = self.format
        else:
            self.set_format(fmt)

        # TODO: different uncertainties can be loaded! either full pose covariance or position and orientation separated
        if fmt.type == CSVSpatialFormatType.PosOrientWithCov:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec, \
                self.Sigma_R_vec = PosOrientWithCov2DataFrame.DataFrame_to_TPQCov(data_frame=df)
        elif fmt.type == CSVSpatialFormatType.PoseWithCov:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec = \
                PoseWithCov2DataFrame.DataFrame_to_TPQCov(data_frame=df)
        else:
            print('Error: format not supported yet')
            assert(False)

    def to_DataFrame(self):
        if self.format.type == CSVSpatialFormatType.PosOrientWithCov:
            return PosOrientWithCov2DataFrame.TPQCov_to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec,
                                                         self.Sigma_R_vec)
        elif self.format.type == CSVSpatialFormatType.PoseWithCov:
            return PoseWithCov2DataFrame.TPQCov_to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec)
        elif self.format.type == CSVSpatialFormatType.TUM:
            return TUMCSV2DataFrame.TPQ_to_DataFrame(self.t_vec, self.p_vec, self.q_vec)
        else:
            print('Error: format [' + str(self.format.type) + '] not supported yet')
            assert (False)

    def save_to_CSV(self, filename):
        if self.is_empty():
            return False
        df = self.to_DataFrame()
        CSV2DataFrame.save_CSV(df, filename=filename, fmt=self.get_format())
        return True


