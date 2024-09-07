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

from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType


class SpatialConverter:
    """
    SpatialConverter for:
    Homogeneous Transformation Matrices and Quaternions (HTMQ) by `Christoph Gohlke <http://www.lfd.uci.edu/~gohlke/>`__,
      Laboratory for Fluorescence Dynamics, University of California, Irvine (https://pypi.org/project/transformations/)

    Problem: the HTMQ quaternion format is non-alphabetic: [x,y,z,w], with w being the scalar part and [x,y,z] the imaginary
    The spatialmath toolbox (https://github.com/petercorke/spatialmath-python) uses [s, v] ([w, x, y, z]),
    s = scalar/real part and v = vector/imaginary part

    In general this convert is required from a historical point, as the spatialmath toolbox is pretty new and would
    require to refactor the whole code base (the [x,y,z,w] format is widely used!)
    """

    def __init__(self):
        pass
    @staticmethod
    def HTMQ_quaternion_identity():
        # [x, y, z, w]
        return np.array([0, 0, 0, 1])

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
        return UnitQuaternion(q_AB[indices])

    @staticmethod
    def HTMQ_quaternion_to_rot(q_AB):
        # numpy is inconsistent with shape! as q_AB.shape == (4, 1) fails!
        if not (isinstance(q_AB, np.ndarray) and len(q_AB) == 4):
            raise ValueError('Invalid input')
        indices = [3, 0, 1, 2]
        return UnitQuaternion(q_AB[indices]).R

    @staticmethod
    def SO3_to_HTMQ_quaternion(R_AB):
        # returns a np.array representing a unit-quaternion [x,y,z, w], [[v], s];
        # Convert SO(3) rotation matrix to unit-quaternion
        if isinstance(R_AB, SO3):
            return base.r2q(R_AB.R, order="xyzs")
        elif base.isrot(R_AB, check=False):
            return base.r2q(R_AB, order="xyzs")
        else:
            print(str(R_AB))
            raise ValueError('Invalid type')

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
            print(str(R_AB))
            raise ValueError('Invalid input')
        if not (isinstance(p_AB_in_A, np.ndarray) and len(p_AB_in_A) == 3):
            print(str(p_AB_in_A))
            raise ValueError('Invalid input')

        T = np.identity(4)
        T[0:3, 0:3] = R_AB
        T[:3, 3] = p_AB_in_A
        T_AB = SE3(T, check=False)
        return T_AB

    @staticmethod
    def SE3_to_p_q_HTMQ(T_AB):
        assert(isinstance(T_AB, SE3))
        R_AB = T_AB.R
        t_AB = T_AB.t

        return t_AB, SpatialConverter.SO3_to_HTMQ_quaternion(R_AB)

    @staticmethod
    def rpy2rot(rpy, unit='rad'):
        return base.rpy2r(roll=rpy[0], pitch=rpy[1], yaw=rpy[2], unit=unit, order='zyx')

    @staticmethod
    def rot2rpy(R, unit='rad'):
        # R = rotz(angles[2]) @ roty(angles[1]) @ rotx(angles[0])
        return base.tr2rpy(T=R, unit=unit, order='zyx')

    @staticmethod
    def quat2rpy(q, unit='rad'):
        # R = rotz(angles[2]) @ roty(angles[1]) @ rotx(angles[0])
        quad = SpatialConverter.HTMQ_quaternion_to_Quaternion(q)
        return base.tr2rpy(T=quad.R, unit=unit, order='zyx')

    @staticmethod
    def quat2theta_q(q_err):
        """
        converts a rotation represented by a quaternion in its small angle approximation
        > S = R * Exp(theta)
        > R(q_err) = R^T * S
        > theta = Log(R(q_err))
        refer to:
        * "Quaternion kinematics for the error-state Kalman filter" by Joan Sola,
            https://arxiv.org/pdf/1711.02508.pdf, chapter 4: Perturbations, derivatives and integrals
        For really small perturbations the following should be a sufficient approximation:
        > R(q) = I + skew(theta)   // Sola EQ. 193
        > theta = unskew(R(q) - I)
        """
        return  2 * (q_err[:3] / q_err[3])

    @staticmethod
    def theta_q2quat(theta_q):
        q_ = Quaternion(s=1, v=theta_q*0.5)
        return SpatialConverter.UnitQuaternion_to_HTMQ_quaternion(q_.unit())

    @staticmethod
    def theta_q2rot(theta_q):
        q_ = Quaternion(s=1, v=theta_q*0.5)
        return q_.unit().R

    @staticmethod
    def p_q_to_tau_se3(p, q):
        T_ = SpatialConverter.p_q_HTMQ_to_SE3(p, q)
        return base.trlog(T_.A, twist=True)

    @staticmethod
    def quat2theta_so3(q_err):
        # R = expm(skew(theta_so3))
        R_ = SpatialConverter.HTMQ_quaternion_to_SO3(q_err)
        return base.trlog(R_.R, twist=True)

    @staticmethod
    def theta_so3_2quat(theta_so3):
        R_ = base.exp2tr(theta_so3)
        if R_.shape == (4,4):
            R_ = R_[0:3,0:3]
        return SpatialConverter.SO3_to_HTMQ_quaternion(R_)

    @staticmethod
    def rot2theta_so3(R):
        # R = expm(skew(theta_so3))
        return base.trlog(R, twist=True)

    @staticmethod
    def theta_so3_2rot(theta_so3):
        R_ =  base.exp2tr(theta_so3)
        if R_.shape == (4,4):
            R_ = R_[0:3,0:3]
        return R_

    @staticmethod
    def quat2theta_R(q_err):
        # R = I + skew(theta_R)
        R_ = SpatialConverter.HTMQ_quaternion_to_SO3(q_err)
        return base.vex(R_.R - np.eye(3))

    @staticmethod
    def theta_R2rot(theta_R):
        R_ = np.eye(3) + base.skew(theta_R)
        # TODO: normalization of rotation matrix via UnitQuaternion
        q_ = UnitQuaternion(SO3(R_, check=False)).unit()
        return q_.R

    @staticmethod
    def rot2theta_R(R):
        # R = I + skew(theta_R)
        return base.vex(R - np.eye(3))

    @staticmethod
    def convert_q_vec_to_theta_vec(q_vec, rot_err_rep):
        # q_vec: expect HTMQ quaternion format [x,y,z,w]
        assert (isinstance(rot_err_rep, ErrorRepresentationType))
        # q_vec as vector of  HTMQ_quaternions
        len, q_cols = q_vec.shape
        assert(q_cols == 4)
        theta_vec = np.zeros((len, 3))

        # Converts quaternion to small angle approximations
        if rot_err_rep == ErrorRepresentationType.theta_R:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2theta_R(q_vec[i])
        elif rot_err_rep == ErrorRepresentationType.theta_q:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2theta_q(q_vec[i])
        elif rot_err_rep == ErrorRepresentationType.theta_so3 or rot_err_rep == ErrorRepresentationType.tau_se3:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2theta_so3(q_vec[i])
        elif rot_err_rep == ErrorRepresentationType.rpy_rad:
            for i in range(len):
                theta_vec[i] = SpatialConverter.quat2rpy(q_vec[i])
        else:
            assert False, 'format is not supported: ' + str(rot_err_rep)

        return theta_vec
