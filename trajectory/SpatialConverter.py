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
# adapted from:
# * https://github.com/uzh-rpg/rpg_trajectory_evaluation/blob/master/src/rpg_trajectory_evaluation/align_trajectory.py
#
# Requirements:
# numpy, numpy_utils
########################################################################################################################
import numpy as np
from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base


class SpatialConverter:
    """
    SpatialConverter for:
    Homogeneous Transformation Matrices and Quaternions (HTMQ) by `Christoph Gohlke <http://www.lfd.uci.edu/~gohlke/>`__,
      Laboratory for Fluorescence Dynamics, University of California, Irvine (https://pypi.org/project/transformations/)

    Problem: the quaternion format is non-alphabetic: [x,y,z,w], with w being the scalar part and [x,y,z] the imaginary
    The spatialmath toolbox (https://github.com/petercorke/spatialmath-python) uses [s, v], s = scalar/real part and v = vector/imaginary part

    In general this convert is required from a historical point, as the spatialmath toolbox is pretty new and would
    require to refactor the whole code base (the [x,y,z,w] format is widely used!)
    """

    def __init__(self):
        pass

    @staticmethod
    def UnitQuaternion_to_HTMQ_quaternion(q_AB):
        assert (isinstance(q_AB, UnitQuaternion))

        # Convert SO(3) rotation matrix to unit-quaternion
        q_AB_ = base.r2q(q_AB.R)
        indices_inv = [1, 2, 3, 0]
        return q_AB_[indices_inv]

    @staticmethod
    def HTMQ_quaternion_to_Quaternion(q_AB):
        # numpy is inconsistent with shape! as q_AB.shape == (4, 1) fails!
        if not (isinstance(q_AB, np.ndarray) and len(q_AB) == 4):
            raise ValueError('Invalid input')
        indices = [3, 0, 1, 2]
        return Quaternion(q_AB[indices])

    @staticmethod
    def SO3_to_HTMQ_quaternion(R_AB):
        # Convert SO(3) rotation matrix to unit-quaternion
        if isinstance(R_AB, SO3):
            q_AB_ = base.r2q(R_AB.R)
        elif base.isrot(R_AB, check=False):
            q_AB_ = base.r2q(R_AB)
        else:
            raise ValueError('Invalid type')

        indices_inv = [1, 2, 3, 0]
        return q_AB_[indices_inv]


    @staticmethod
    def HTMQ_quaternion_to_SO3(q_AB):
        # numpy is inconsistent with shape! as q_AB.shape == (4, 1) fails!
        if not (isinstance(q_AB, np.ndarray) and len(q_AB) == 4):
            raise ValueError('Invalid input')
        indices = [3, 0, 1, 2]
        SO3_ = SO3(Quaternion(q_AB[indices]).unit().R)
        return SO3_

    @staticmethod
    def p_q_HTMQ_to_SE3(p_AB_in_A, q_AB):
        if not (isinstance(q_AB, np.ndarray) and len(q_AB) == 4):
            raise ValueError('Invalid input')
        if not (isinstance(p_AB_in_A, np.ndarray) and len(p_AB_in_A) == 3):
            raise ValueError('Invalid input')

        T = np.identity(4)
        R_AB = SpatialConverter.HTMQ_quaternion_to_SO3(q_AB)
        T[0:3, 0:3] = R_AB.R
        T[:3, 3] = p_AB_in_A
        T_AB = SE3(T)
        return T_AB

    @staticmethod
    def p_R_to_SE3(p_AB_in_A, R_AB):
        if not (isinstance(R_AB, np.ndarray) and R_AB.shape == (3, 3)):
            raise ValueError('Invalid input')
        if not (isinstance(p_AB_in_A, np.ndarray) and len(p_AB_in_A) == 3):
            raise ValueError('Invalid input')

        T = np.identity(4)
        T[0:3, 0:3] = R_AB
        T[:3, 3] = p_AB_in_A
        T_AB = SE3(T)
        return T_AB

    @staticmethod
    def SE3_to_p_q_HTMQ(T_AB):
        assert(isinstance(T_AB, SE3))
        R_AB = T_AB.R
        t_AB = T_AB.t

        return t_AB, SpatialConverter.SO3_to_HTMQ_quaternion(R_AB)


########################################################################################################################
#################################################### T E S T ###########################################################
########################################################################################################################
import unittest

class SpatialConverter_Test(unittest.TestCase):
    def test_SO3_to_HTMQ_quaternion(self):
        R1 = SO3.RPY([0, 0, 45], unit='deg')

        print(R1)
        q1 = SpatialConverter.SO3_to_HTMQ_quaternion(R1)
        print(q1)
        q2 = SpatialConverter.SO3_to_HTMQ_quaternion(R1.R)
        print(q2)
        self.assertTrue(((q1 == q2).all()))

        q3 = SpatialConverter.SO3_to_HTMQ_quaternion(np.eye(3))
        print(q3)
        self.assertTrue(((q3 == np.array([0, 0, 0, 1])).all()))

        pass

    def test_HTMQ_quaternion_to_SO3(self):
        q1_TUM = np.array([0, 0,  0.38268343, 0.92387953])
        print(q1_TUM)
        R1 = SpatialConverter.HTMQ_quaternion_to_SO3(q1_TUM)
        print(R1)
        pass

    def test_p_q_HTMQ_to_SE3(self):
        p = np.array([1,2,3])
        q = np.array([0, 0,  0.38268343, 0.92387953])

        SE3_ = SpatialConverter.p_q_HTMQ_to_SE3(p, q)
        print(SE3_)
        self.assertTrue((SE3_.t == np.array([1,2,3]) ).all())
        R1 = SO3.RPY([0, 0, 45], unit='deg')

        diff = SE3_.R - R1.R
        print(diff)
        self.assertTrue( np.allclose(SE3_.R, R1.R))

    def test_SE3_to_HTMQ_p_q(self):
        SE3_ = SE3(1,2,3)
        # SE3 is missing a CTOR for rotation matrices
        SE3_R = SE3.RPY([0, 0, 45], unit='deg')
        SE3_ = SE3_ * SE3_R

        p, q = SpatialConverter.SE3_to_p_q_HTMQ(SE3_)

        print(p)
        print(q)
        self.assertTrue((p == np.array([1, 2, 3])).all())
        self.assertTrue(np.allclose(q, np.array([0, 0,  0.38268343, 0.92387953])))

if __name__ == "__main__":
    unittest.main()
