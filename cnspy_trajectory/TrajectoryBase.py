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
from abc import ABC, abstractmethod
import numpy as np
import pandas

from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame


class TrajectoryBase(ABC):
    t_vec = None

    def is_empty(self):
        return self.num_elems() == 0

    def num_elems(self):
        if self.t_vec is None:
            return 0
        else:
            return len(self.t_vec)

    @abstractmethod
    def to_DataFrame(self):
        pass

    @abstractmethod
    def load_from_DataFrame(self, df, fmt_type=None):
        pass

    def load_from_CSV(self, fn):
        if not os.path.isfile(fn):
            print("TrajectoryBase: could not find file %s" % os.path.abspath(fn))
            return False

        loader = CSV2DataFrame(fn=fn)
        self.load_from_DataFrame(loader.data_frame)
        return loader.data_loaded

    def save_to_CSV(self, fn):
        if self.is_empty():
            return False
        CSV2DataFrame.save_CSV(self.to_DataFrame(), filename=fn)
        return True
