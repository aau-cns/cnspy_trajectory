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
import math
import os
from sys import version_info
from abc import ABC, abstractmethod
import numpy as np
import pandas

from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_timestamp_association.TimestampAssociation import TimestampAssociation


# abstract methods:
# - to_DataFrame(self):
# - load_from_DataFrame(self, df, fmt_type=None)
class TrajectoryBase(ABC):
    t_vec = None

    @abstractmethod
    def to_DataFrame(self):
        pass

    @abstractmethod
    def load_from_DataFrame(self, df, fmt_type=None):
        pass

    @abstractmethod
    def subsample(self, step=None, num_max_points=None, verbose=False):
        num_elems = self.num_elems()

        if num_max_points:
            step = 1
            if (int(num_max_points) > 0) and (int(num_max_points) < num_elems):
                step = int(math.ceil(num_elems / float(num_max_points)))

        sparse_indices = np.arange(start=0, stop=num_elems, step=step)
        if (num_max_points or step):
            if verbose:
                print("TrajectoryBase.subsample():")
                print("* len: " + str(num_elems) + ", max_num_points: " + str(
                    num_max_points) + ", subsample by: " + str(step))

            self.t_vec = self.t_vec[sparse_indices]

        return sparse_indices

    @abstractmethod
    def sample(self, indices_arr, verbose=False):
        num_elems = self.num_elems()
        assert (len(indices_arr) <= num_elems), "TrajectoryBase.sample():\n\t index array must be smaller " \
                                                "equal the dataframe."
        assert (max(indices_arr) <= num_elems), "TrajectoryBase.sample():\n\t elemts in the index array " \
                                                "must be smaller equal the dataframe."

        assert (min(indices_arr) >= 0), "TrajectoryBase.sample():\n\t elemts in the index array " \
                                        "must be greater equal zero."
        if verbose:
            print("TrajectoryBase.sample():")
            print("* current len: " + str(num_elems) + ", take N-samples: " + len(indices_arr))

        self.t_vec = self.t_vec[indices_arr]

    @abstractmethod
    def clone(self):
        pass


    def is_empty(self):
        return self.num_elems() == 0

    def num_elems(self):
        if self.t_vec is None:
            return 0
        else:
            return len(self.t_vec)

    def sample_at_t_arr(self, t_arr, max_difference=0, round_decimals=9, unique_timestamps=False):
        indices_arr, idx_t, t_vec_matched, t_arr_matched = \
            TimestampAssociation.associate_timestamps(self.t_vec, t_arr,
                                                      max_difference=max_difference,
                                                      round_decimals=round_decimals,
                                                      unique_timestamps=unique_timestamps)
        # call abstract method
        self.sample(indices_arr)
        return idx_t, t_arr_matched

    def load_from_CSV(self, fn):
        if not os.path.isfile(fn):
            print("TrajectoryBase: could not find file %s" % os.path.abspath(fn))
            return False
        try:
            loader = CSV2DataFrame(fn=fn)
            self.load_from_DataFrame(loader.data_frame, fmt_type=loader.format)
            return loader.data_loaded
        except:
            print("TrajectoryBase: could not load file %s" % os.path.abspath(fn))
            return False

    def save_to_CSV(self, fn):
        if self.is_empty():
            return False
        CSV2DataFrame.save_CSV(self.to_DataFrame(), filename=fn)
        return True
