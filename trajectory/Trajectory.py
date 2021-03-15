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
import numpy as np
from spatialmath import UnitQuaternion, SE3, SO3, Quaternion
from csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from csv2dataframe.CSV2DataFrame import CSV2DataFrame
from spatial_csv_formats.CSVFormatPose import CSVFormatPose
from trajectory.SpatialConverter import SpatialConverter


class Trajectory:
    p_vec = None
    q_vec = None
    t_vec = None

    def __init__(self, t_vec=None, p_vec=None, q_vec=None, df=None):
        """
            CTOR expects either a pandas.DataFrame or a (timestamp + position + quaternion) matrix

            INPUT:
            t_vec -- [Nx1] matrix, containing N timestamps
            p_vec -- [Nx3] matrix, containing x,y,z positions
            q_vec -- [Nx4] matrix, containing quaternions [x,y,z,w]
            df -- pandas.DataFrame holding a table with ['t', 'tx', 'ty', 'tz','qx', 'qy', 'qz', 'qw']
        """
        if df is not None:
            self.load_from_DataFrame(df)
        elif t_vec is not None and p_vec is not None and q_vec is not None:
            if t_vec.ndim == 1:
                t_vec = np.array([t_vec])

            t_rows, t_cols = t_vec.shape
            p_rows, p_cols = p_vec.shape
            q_rows, q_cols = q_vec.shape
            assert (t_rows == p_rows)
            assert (t_rows == q_rows)
            assert (t_cols == 1)
            assert (p_cols == 3)
            assert (q_cols == 4)

            self.t_vec = t_vec
            self.p_vec = p_vec
            self.q_vec = q_vec

    def load_from_CSV(self, filename):
        if not os.path.isfile(filename):
            print("Trajectory: could not find file %s" % os.path.abspath(filename))
            return False

        loader = CSV2DataFrame(filename=filename)
        self.load_from_DataFrame(loader.data_frame)
        return loader.data_loaded

    def load_from_DataFrame(self, df):
        self.t_vec, self.p_vec, self.q_vec = TUMCSV2DataFrame.DataFrame_to_TPQ(data_frame=df)

    def to_DataFrame(self):
        return TUMCSV2DataFrame.TPQ_to_DataFrame(self.t_vec, self.p_vec, self.q_vec)

    def save_to_CSV(self, filename):
        if self.is_empty():
            return False
        df = TUMCSV2DataFrame.TPQ_to_DataFrame(self.t_vec, self.p_vec, self.q_vec)
        TUMCSV2DataFrame.save_CSV(df, filename=filename, fmt=CSVFormatPose.TUM)
        return True

    def is_empty(self):
        return self.t_vec is None

    def num_elems(self):
        if self.is_empty():
            return 0
        else:
            return len(self.t_vec)

    def get_distance(self):
        if self.p_vec is not None:
            return Trajectory.distance(self.p_vec)
        else:
            return 0

    def get_accumulated_distances(self):
        return Trajectory.distances_from_start(self.p_vec)

    def get_rpy_vec(self):
        rpy_vec = np.zeros(np.shape(self.p_vec))
        for i in range(np.shape(self.p_vec)[0]):
            q = SpatialConverter.HTMQ_quaternion_to_Quaternion(self.q_vec[i, :])
            rpy_vec[i, :] = q.unit().rpy(order='xyz')

        return rpy_vec

    def transform(self, scale=1.0, t=np.zeros((3,)), R=np.identity(3)):
        p_es_aligned = np.zeros(np.shape(self.p_vec))
        q_es_aligned = np.zeros(np.shape(self.q_vec))

        T_AB = SpatialConverter.p_R_to_SE3(t, R)
        for i in range(np.shape(self.p_vec)[0]):
            T_BC = SpatialConverter.p_q_HTMQ_to_SE3(scale * self.p_vec[i, :], self.q_vec[i, :])
            # T_AC = T_AB * T_BC
            p_AC, q_AC = SpatialConverter.SE3_to_p_q_HTMQ(T_AB * T_BC)
            q_es_aligned[i, :] = q_AC
            p_es_aligned[i, :] = p_AC

        # self.p_vec = R * (scale * self.p_vec) + t

        self.p_vec = p_es_aligned
        self.q_vec = q_es_aligned

    @staticmethod
    def distances_from_start(p_vec):
        distances = np.diff(p_vec[:, 0:3], axis=0)
        distances = np.sqrt(np.sum(np.multiply(distances, distances), axis=1))
        distances = np.cumsum(distances)
        distances = np.concatenate(([0], distances))
        return distances

    @staticmethod
    def distance(p_vec):
        accum_distances = Trajectory.distances_from_start(p_vec)
        return accum_distances[-1]

