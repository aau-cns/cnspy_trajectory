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
from enum import Enum

# Please refer to "Error definitions and filter credibility evaluation", Jung and Weiss, 2022
class RotationErrorRepresentationType(Enum):
    R_small_theta = 'R_small_theta'  # R ~ eye(3) + skew(theta_R)
    q_small_theta = 'q_small_theta'  # q ~ [1; 0.5 theta_q]
    so3_theta = 'so3_theta'          # R = exp(skew(theta))
    rpy_degree = 'rpy_degree'        # R = Rz(y)*Ry(p)*Rx(r); roll(r),pitch(p),yaw(y) in [deg]
    rpy_rad = 'rpy_rad'              # R = Rz(y)*Ry(p)*Rx(r); roll(r),pitch(p),yaw(y) in [rad]
    none = 'none'
    # HINT: if you add an entry here, please also add it to the .list() method!

    def __str__(self):
        return self.value

    @staticmethod
    def list():
        return list([str(RotationErrorRepresentationType.R_small_theta),
                     str(RotationErrorRepresentationType.q_small_theta),
                     str(RotationErrorRepresentationType.rpy_degree),
                     str(RotationErrorRepresentationType.rpy_rad),
                     str(RotationErrorRepresentationType.none)])
