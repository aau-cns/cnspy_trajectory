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
class EstimationErrorType(Enum):
    type1 = 'type1'  #  TRUE = EST \oplus ERR; e.g. local pose error from estimated body to true body
    # e.g., p_WB_in_W_true = p_WB_in_W_est + R(q_WB_est)*p_WB_in_B_err; and R(q_WB_true) = R(q_WB_est)*R(q_WB_err)
    type2 = 'type2'  #  TRUE = ERR \oplus EST; e.g. global pose error from true world to estimated world
    type3 = 'type3'  #  TRUE = EST \ominus ERR; a \ominus b == a \oplus \inv(b); e.g. local pose error from true body to estimated body
    type4 = 'type4'  #  TRUE = ERR \ominus EST; e.g. global pose error from true global to estimated global
    type5 = 'type5'  #  TRUE = EST + ERR for positions R(3); TRUE = EST \oplus ERR for rotations SO(3);
    # e.g. p_WB_in_W_true = p_WB_in_W_est + p_WB_in_W_err; and  R(q_WB_true) = R(q_WB_est)*R(q_WB_err)
    type6 = 'type6'  #  TRUE = ERR + EST for positions R(3); TRUE = ERR \oplus EST for rotations SO(3);
    # e.g. p_WB_in_W_true = p_WB_in_W_est + p_WB_in_W_err; and R(q_WB_true) = R(q_WB_err)*R(q_WB_est)
    none = 'none'
    # HINT: if you add an entry here, please also add it to the .list() method!

    def __str__(self):
        return self.value

    @staticmethod
    def list():
        return list([str(EstimationErrorType.type1),
                     str(EstimationErrorType.type2),
                     str(EstimationErrorType.type3),
                     str(EstimationErrorType.type4),
                     str(EstimationErrorType.type5),
                     str(EstimationErrorType.type6),
                     str(EstimationErrorType.none)])
