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
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_csv2dataframe.PoseWithCov2DataFrame import PoseWithCov2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_spatial_csv_formats.CSVFormatPose import CSVFormatPose


class TrajectoryEstimated(Trajectory):
    # position uncertainty: 3x3 covariance matrix.
    # The with upper triangular elements are vectorized: 'pxx', 'pxy', 'pxz', 'pyy', 'pyz', 'pzz'
    Sigma_p_vec = None

    # small angle `theta` uncertainty: R = ( eye(3) + slew(theta)), a 3x3 covariance matrix.
    # The with upper triangular elements are vectorized: 'qrr', 'qrp', 'qry', 'qpp', 'qpy', 'qyy'
    Sigma_q_vec = None

    def __init__(self, t_vec=None, p_vec=None, q_vec=None, Sigma_p_vec=None, Sigma_q_vec=None, df=None):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)
        self.Sigma_p_vec = Sigma_p_vec
        self.Sigma_q_vec = Sigma_q_vec

        if df is not None:
            self.load_from_DataFrame(df)

    def load_from_CSV(self, filename):
        if not os.path.isfile(filename):
            print("Trajectory: could not find file %s" % os.path.abspath(filename))
            return False

        loader = CSV2DataFrame(filename=filename)
        self.load_from_DataFrame(loader.data_frame)
        return loader.data_loaded

    def load_from_DataFrame(self, df):
        self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec, self.Sigma_q_vec = PoseWithCov2DataFrame.DataFrame_to_TPQCov(
            data_frame=df)

    def to_DataFrame(self):
        return PoseWithCov2DataFrame.TPQCov_to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec,
                                                         self.Sigma_q_vec)

    def save_to_CSV(self, filename):
        if self.is_empty():
            return False
        df = self.to_DataFrame()
        CSV2DataFrame.save_CSV(df, filename=filename, fmt=CSVFormatPose.PoseWithCov)
        return True


