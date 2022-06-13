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
# numpy, cnspy_numpy_utils
########################################################################################################################
import numpy as np
from spatialmath import UnitQuaternion, SO3, SE3, Quaternion, base
from cnspy_trajectory.SpatialConverter import SpatialConverter
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


    def test_rpy2rot(self):
        R_ref = SO3.RPY([0, 0, 45], unit='deg')
        rpy = np.array([0, 0, 45])
        R_ = SpatialConverter.rpy2rot(rpy, unit='deg')
        rpy_ = SpatialConverter.rot2rpy(R_, unit='deg')
        print(R_ref - R_)
        print(rpy - rpy_)
        self.assertTrue(np.allclose(R_ref, R_))
        self.assertTrue(np.allclose(rpy, rpy_))

    def test_theta_q(self):
        q = np.array([0, 0, 0.38268343, 0.92387953])

        theta_q = SpatialConverter.quat2theta_q(q)
        print(theta_q)
        q_ = SpatialConverter.theta_q2quat(theta_q)
        R_ = SpatialConverter.theta_q2rot(theta_q)
        self.assertTrue(np.allclose(q, q_))

    def test_theta_so3(self):
        q = np.array([0, 0, 0.38268343, 0.92387953])

        theta_so3 = SpatialConverter.quat2theta_so3(q)
        print(theta_so3)
        q_ = SpatialConverter.theta_so3_2quat(theta_so3)
        self.assertTrue(np.allclose(q, q_))

        R_ref = SO3.RPY([0, 0, 45], unit='deg').R
        theta_so3 = SpatialConverter.rot2theta_so3(R_ref)
        print(theta_so3)
        R_ = SpatialConverter.theta_so3_2rot(theta_so3)
        self.assertTrue(np.allclose(R_ref, R_))

    def test_theta_R(self):
        R_ref = SO3.RPY([4, 2, 4], unit='deg').R
        theta_R = SpatialConverter.rot2theta_R(R_ref)
        print(theta_R)
        R_ = SpatialConverter.theta_R2rot(theta_R)
        print(R_ref - R_)


if __name__ == "__main__":
    unittest.main()
