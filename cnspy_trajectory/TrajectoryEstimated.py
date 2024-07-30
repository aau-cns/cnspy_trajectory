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
import math
import os

import numpy as np
import pandas
import matplotlib.pyplot as plt
from spatialmath import base

from cnspy_csv2dataframe.PosOrientWithCovTyped2DataFrame import PosOrientWithCovTyped2DataFrame
from cnspy_csv2dataframe.PoseWithCovTyped2DataFrame import PoseWithCovTyped2DataFrame
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_trajectory.SpatialConverter import SpatialConverter
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_csv2dataframe.PosOrientWithCov2DataFrame import PosOrientWithCov2DataFrame
from cnspy_csv2dataframe.PoseWithCov2DataFrame import PoseWithCov2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.CSVSpatialFormat import CSVSpatialFormat
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_spatial_csv_formats.ErrorRepresentationType import ErrorRepresentationType
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotUtils import TrajectoryPlotUtils

# TODO: make TrajectoryEstimatedBase and derive TrajEstPose and TrajEstPosOrient
# TODO: convert locally estimated covariance into global covariance using the
#  adjoint: https://gtsam.org/2021/02/23/uncertainties-part3.html Σ_W = Ad_{T_{WB}} Σ_B Ad_{T_{WB}}^{T}
# TODO: overwrite transform to rotate global covariances!
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
    format = CSVSpatialFormat()

    def __init__(self, t_vec=None, p_vec=None, q_vec=None,
                 Sigma_p_vec=None, Sigma_R_vec=None,
                 Sigma_pR_vec=None, Sigma_T_vec=None,
                 df=None, fn=None):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)
        self.Sigma_p_vec = Sigma_p_vec
        self.Sigma_R_vec = Sigma_R_vec
        self.Sigma_pR_vec = Sigma_pR_vec
        self.Sigma_T_vec = Sigma_T_vec

        if df is not None:
            self.load_from_DataFrame(df)
        elif fn is not None:
            self.load_from_CSV(fn)
        pass

    def set_format(self, fmt):
        if fmt is not None and isinstance(fmt, CSVSpatialFormat):
            self.format = fmt
        elif fmt is not None and isinstance(fmt, CSVSpatialFormatType):
            self.format = CSVSpatialFormat(fmt_type=fmt)

    def get_format(self):
        return self.format

    def convert_to_global_covariance(self):
        # Covariance of pose is expressed in the local frame
        if self.format.estimation_error_type == EstimationErrorType.type1:
            if self.Sigma_T_vec is not None:
                for i in range(self.num_elems()):
                    T_AB = SpatialConverter.p_q_HTMQ_to_SE3(self.p_vec[i,:], self.q_vec[i,:])
                    Adj_AB = base.tr2adjoint(T_AB)

                    # Sigma expressed in Navigation frame
                    Sigma_NB = self.Sigma_T_vec[i, :, :]
                    Sigma_GB = np.dot(Adj_AB, np.dot(Sigma_NB, Adj_AB.T))
                    self.Sigma_T_vec[i, :, :] = Sigma_GB

            self.format.estimation_error_type = EstimationErrorType.type2
        # Covariance of orientation is defined locally
        elif self.format.estimation_error_type == EstimationErrorType.type5:
            if self.Sigma_R_vec is not None:
                for i in range(self.num_elems()):
                    # Sigma expressed in Navigation frame
                    Sigma_q_NB = self.Sigma_R_vec[i, :, :]
                    R_GN = SpatialConverter.HTMQ_quaternion_to_rot(self.q_vec[i,:])
                    Sigma_q_GB = np.dot(R_GN, np.dot(Sigma_q_NB, R_GN.T))
                    self.Sigma_R_vec[i, :, :] = Sigma_q_NB
        # Covariance of position and orientation is defined globally
            self.format.estimation_error_type = EstimationErrorType.type6

        pass

    # overriding abstract method
    def transform(self, scale=1.0, p_GN_in_G=np.zeros((3,)), R_GN=np.identity(3)):
        # Calling the parent's class
        Trajectory.transform(self=self, scale=scale, p_GN_in_G=p_GN_in_G, R_GN=R_GN)

        # Covariance of pose is expressed in the global frame
        if self.format.estimation_error_type == EstimationErrorType.type2:
            if self.Sigma_T_vec is not None:
                T_AB = SpatialConverter.p_R_to_SE3(p_GN_in_G, R_GN)
                Adj_AB = base.tr2adjoint(T_AB)
                for i in range(self.num_elems()):
                    # Sigma expressed in Navigation frame
                    Sigma_NB = self.Sigma_T_vec[i, :, :]
                    Sigma_GB = np.dot(Adj_AB, np.dot(Sigma_NB, Adj_AB.T))
                    self.Sigma_T_vec[i, :, :] = Sigma_GB
        # Covariance of position is defined globally
        elif self.format.estimation_error_type == EstimationErrorType.type5:
            if self.Sigma_p_vec is not None:
                for i in range(self.num_elems()):
                    # Sigma expressed in Navigation frame
                    Sigma_p_NB = self.Sigma_p_vec[i, :, :]
                    Sigma_p_GB = np.dot(R_GN, np.dot(Sigma_p_NB, R_GN.T))
                    self.Sigma_p_vec[i, :, :] = Sigma_p_GB
        # Covariance of position and orientation is defined globally
        elif self.format.estimation_error_type == EstimationErrorType.type6:
            if self.Sigma_p_vec is not None and self.Sigma_R_vec is not None:
                for i in range(self.num_elems()):
                    # Sigma expressed in Navigation frame
                    Sigma_p_NB = self.Sigma_p_vec[i, :, :]
                    Sigma_p_GB = np.dot(R_GN, np.dot(Sigma_p_NB, R_GN.T))
                    self.Sigma_p_vec[i, :, :] = Sigma_p_GB

                    Sigma_R_NB = self.Sigma_R_vec[i, :, :]
                    Sigma_R_GB = np.dot(R_GN, np.dot(Sigma_R_NB, R_GN.T))
                    self.Sigma_p_vec[i, :, :] = Sigma_R_GB
        # else:
        #    local covariances are invariant to global reference changes!

    ########################################################

    # overriding abstract method
    def subsample(self, step=None, num_max_points=None, verbose=False):
        sparse_indices = Trajectory.subsample(self, step=step, num_max_points=num_max_points, verbose=verbose)

        if self.Sigma_p_vec is not None:
            self.Sigma_p_vec = self.Sigma_p_vec[sparse_indices, :, :]
        if self.Sigma_R_vec is not None:
            self.Sigma_R_vec = self.Sigma_R_vec[sparse_indices, :, :]
        if self.Sigma_pR_vec is not None:
            self.Sigma_pR_vec = self.Sigma_pR_vec[sparse_indices, :, :]
        if self.Sigma_T_vec is not None:
            self.Sigma_T_vec = self.Sigma_T_vec[sparse_indices, :, :]

        return sparse_indices

    # overriding abstract method
    def sample(self, indices_arr, verbose=False):
        Trajectory.sample(self, indices_arr=indices_arr)
        if self.Sigma_p_vec is not None:
            self.Sigma_p_vec = self.Sigma_p_vec[indices_arr, :, :]
        if self.Sigma_R_vec is not None:
            self.Sigma_R_vec = self.Sigma_R_vec[indices_arr, :, :]
        if self.Sigma_pR_vec is not None:
            self.Sigma_pR_vec = self.Sigma_pR_vec[indices_arr, :, :]
        if self.Sigma_T_vec is not None:
            self.Sigma_T_vec = self.Sigma_T_vec[indices_arr, :, :]

    # overriding abstract method
    def clone(self):
        obj =  TrajectoryEstimated(t_vec=self.t_vec.copy(), p_vec=self.p_vec.copy(), q_vec=self.q_vec.copy())
        obj.set_format(self.get_format())
        if self.Sigma_p_vec is not None:
            obj.Sigma_p_vec = self.Sigma_p_vec.copy()
        if self.Sigma_R_vec is not None:
            obj.Sigma_R_vec = self.Sigma_R_vec.copy()
        if self.Sigma_pR_vec is not None:
            obj.Sigma_pR_vec = self.Sigma_pR_vec.copy()
        if self.Sigma_T_vec is not None:
            obj.Sigma_T_vec = self.Sigma_T_vec.copy()
        return obj

    # overriding abstract method
    def load_from_DataFrame(self, df, fmt_type=None):
        assert (isinstance(df, pandas.DataFrame))
        if fmt_type is None:
            fmt_type = CSV2DataFrame.identify_format(dataframe=df)
        else:
            assert (isinstance(fmt_type, CSVSpatialFormatType))

        err_rep_type = ErrorRepresentationType.none
        est_err_type = EstimationErrorType.none
        # TODO: different uncertainties can be loaded! either full pose covariance or position and orientation separated
        if fmt_type == CSVSpatialFormatType.PosOrientWithCov:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec, self.Sigma_R_vec = \
                PosOrientWithCov2DataFrame.from_DataFrame(data_frame=df)
        elif fmt_type == CSVSpatialFormatType.PoseWithCov:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec = \
                PoseWithCov2DataFrame.from_DataFrame(data_frame=df)
        elif fmt_type == CSVSpatialFormatType.PoseStamped or fmt_type == CSVSpatialFormatType.TUM:
            self.t_vec, self.p_vec, self.q_vec = TUMCSV2DataFrame.from_DataFrame(data_frame=df)
        elif fmt_type == CSVSpatialFormatType.PosOrientWithCovTyped:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec, self.Sigma_R_vec, \
            est_err_type_vec, err_rep_vec = PosOrientWithCovTyped2DataFrame.from_DataFrame(data_frame=df)
            est_err_type = EstimationErrorType(est_err_type_vec[0])
            err_rep_type = ErrorRepresentationType(err_rep_vec[0])
        elif fmt_type == CSVSpatialFormatType.PoseWithCovTyped:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec, \
            est_err_type_vec, err_rep_vec = PoseWithCovTyped2DataFrame.from_DataFrame(data_frame=df)
            est_err_type = EstimationErrorType(est_err_type_vec[0])
            err_rep_type = ErrorRepresentationType(err_rep_vec[0])
        else:
            print('Error: format not supported yet')
            return False

        self.set_format(CSVSpatialFormat(fmt_type, est_err_type=est_err_type, err_rep_type=err_rep_type))
        return True

    # overriding abstract method
    def to_DataFrame(self):
        if self.format.type == CSVSpatialFormatType.PosOrientWithCov:
            return PosOrientWithCov2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec,
                                                           self.Sigma_R_vec)
        elif self.format.type == CSVSpatialFormatType.PoseWithCov:
            return PoseWithCov2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec)
        elif self.format.type == CSVSpatialFormatType.TUM:
            return TUMCSV2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec)
        elif self.format.type == CSVSpatialFormatType.PosOrientWithCovTyped:
            est_err_type_vec = np.repeat(self.format.estimation_error_type.str(), self.num_elems(), axis=0)
            err_rep_vec = np.repeat(self.format.rotation_error_representation.str(), self.num_elems(), axis=0)

            return PosOrientWithCovTyped2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec,
                                                                self.Sigma_R_vec,
                                                                est_err_type_vec=est_err_type_vec,
                                                                err_rep_vec=err_rep_vec)
        elif self.format.type == CSVSpatialFormatType.PoseWithCovTyped:
            est_err_type_vec = np.repeat(self.format.estimation_error_type.str(), self.num_elems(), axis=0)
            err_rep_vec = np.repeat(self.format.rotation_error_representation.str(), self.num_elems(), axis=0)

            return PoseWithCovTyped2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec,
                                                           est_err_type_vec=est_err_type_vec,
                                                           err_rep_vec=err_rep_vec)
        else:
            print('Error: format [' + str(self.format.type) + '] not supported yet')
            return pandas.DataFrame()

    ####################################################################################################################
    ######### PLOTTING #################################################################################################
    ####################################################################################################################
    def get_p_N_sigma_data(self, cfg, sigma_N=3.0):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        negative_variance_detected = False

        sigma_p_diag_vec = np.zeros(np.shape(self.p_vec))
        if self.format.type == CSVSpatialFormatType.PosOrientWithCov or self.format.type == CSVSpatialFormatType.PosOrientWithCovTyped:
            for i in range(self.num_elems()):
                P = self.Sigma_p_vec[i]
                d_variance = np.diag(P)
                if min(d_variance) < 0:
                    negative_variance_detected = True
                    d_variance = abs(d_variance)

                sigma_p_diag_vec[i] = np.sqrt(d_variance)

        elif self.format.type == CSVSpatialFormatType.PoseWithCov or self.format.type == CSVSpatialFormatType.PoseWithCovTyped :
            for i in range(self.num_elems()):
                P = self.Sigma_T_vec[i]

                d_variance = np.diag(P[[0,1,2], [0,1,2]])
                if min(d_variance) < 0:
                    negative_variance_detected = True
                    d_variance = abs(d_variance)

                sigma_p_diag_vec[i] = np.sqrt(d_variance)
        else:
            print('format ' + str(self.format.type) + ' not supported!')
            assert False
        sigma_N_diag_vec = sigma_p_diag_vec * sigma_N

        if negative_variance_detected:
            print('WARNING: TrajectoryEstimated.get_p_N_sigma_data(): negative variance detected!')

        # sigma3_diag_vec = np.zeros(np.shape(self.p_vec))
        # if self.format.rotation_error_representation == ErrorRepresentationType.theta_R:
        #     for i in range(self.num_elems()):
        #         sigma3_diag_vec = sigma_p_diag_vec[i]
        # if not cfg.radians:
        #     sigma3_diag_vec = np.rad2deg(sigma3_diag_vec)

        ts = self.t_vec
        xs = sigma_N_diag_vec[:, 0]
        ys = sigma_N_diag_vec[:, 1]
        zs = sigma_N_diag_vec[:, 2]
        dist_vec = self.get_accumulated_distances()


        if cfg.white_list:
            print("white list args: " + str(cfg.white_list))
        if any([flag == 'x' for flag in cfg.white_list]):
            xs = []
            print("clear xs")
        if any([flag == 'y' for flag in cfg.white_list]):
            ys = []
            print("clear ys")
        if any([flag == 'z' for flag in cfg.white_list]):
            zs = []
            print("clear zs")

        if not (len(xs) and isinstance(xs[0], np.float64) and not math.isnan(xs[0])):
            xs = []
        if not (len(ys) and isinstance(ys[0], np.float64) and not math.isnan(ys[0])):
            ys = []
        if not (len(zs) and isinstance(zs[0], np.float64) and not math.isnan(zs[0])):
            zs = []

        return ts, xs, ys, zs, dist_vec

    def get_rpy_N_sigma_data(self, cfg, sigma_N=3.0):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        negative_variance_detected = False

        sigma_theta_diag_vec = np.zeros(np.shape(self.p_vec))
        if self.format.type == CSVSpatialFormatType.PosOrientWithCov or self.format.type == CSVSpatialFormatType.PosOrientWithCovTyped:
            for i in range(self.num_elems()):
                d_variance = np.diag(self.Sigma_R_vec[i])
                if min(d_variance) < 0:
                    negative_variance_detected = True
                    d_variance = abs(d_variance)
                sigma_theta_diag_vec[i] = np.sqrt(d_variance)
        elif self.format.type == CSVSpatialFormatType.PoseWithCov or self.format.type == CSVSpatialFormatType.PoseWithCovTyped:
            for i in range(self.num_elems()):
                P = self.Sigma_T_vec[i]
                d_variance = np.diag(P[[3,4,5], [3,4,5]])
                if min(d_variance) < 0:
                    negative_variance_detected = True
                    d_variance = abs(d_variance)
                sigma_theta_diag_vec[i] = np.sqrt(d_variance)

        else:
            print('format ' + str(self.format.type) + ' not supported!')
            assert False

        if negative_variance_detected:
            print('WARNING: TrajectoryEstimated.get_rpy_N_sigma_data(): negative variance detected!')

        sigma_rpy_diag_vec = np.zeros(np.shape(self.p_vec))

        unit='rad'
        if not cfg.radians:
            unit='deg'

        # Converts small angle approximations of covariance to rpy angles: R =  Rz(y)Ry(p)Rx(r)
        if self.format.rotation_error_representation == ErrorRepresentationType.theta_R:
            for i in range(self.num_elems()):
                R = SpatialConverter.theta_R2rot(sigma_theta_diag_vec[i])
                sigma_rpy_diag_vec[i] = SpatialConverter.rot2rpy(R, unit=unit)
        elif self.format.rotation_error_representation == ErrorRepresentationType.theta_q:
            for i in range(self.num_elems()):
                R = SpatialConverter.theta_q2rot(sigma_theta_diag_vec[i])
                sigma_rpy_diag_vec[i] = SpatialConverter.rot2rpy(R, unit=unit)
        elif self.format.rotation_error_representation == ErrorRepresentationType.theta_so3:
            for i in range(self.num_elems()):
                R = SpatialConverter.theta_so3_2rot(sigma_theta_diag_vec[i])
                sigma_rpy_diag_vec[i] = SpatialConverter.rot2rpy(R, unit=unit)
        elif self.format.rotation_error_representation == ErrorRepresentationType.rpy_rad:
            if cfg.radians:
                sigma_rpy_diag_vec = sigma_theta_diag_vec
            else:
                for i in range(self.num_elems()):
                    sigma_rpy_diag_vec[i] = np.rad2deg(sigma_theta_diag_vec[i])
        elif self.format.rotation_error_representation == ErrorRepresentationType.rpy_degree:
            if not cfg.radians:
                sigma_rpy_diag_vec = sigma_theta_diag_vec
            else:
                for i in range(self.num_elems()):
                    sigma_rpy_diag_vec[i] = np.deg2rad(sigma_theta_diag_vec[i])
        else:
            print('format is not supported: ' + str(self.format.rotation_error_representation))
            assert False

        sigma_N_diag_vec = sigma_rpy_diag_vec * sigma_N

        ts = self.t_vec
        xs = sigma_N_diag_vec[:, 0]
        ys = sigma_N_diag_vec[:, 1]
        zs = sigma_N_diag_vec[:, 2]
        dist_vec = self.get_accumulated_distances()

        if cfg.white_list:
            print("white list args: " + str(cfg.white_list))
        if any([flag == 'x' for flag in cfg.white_list]):
            xs = []
            print("clear xs")
        if any([flag == 'y' for flag in cfg.white_list]):
            ys = []
            print("clear ys")
        if any([flag == 'z' for flag in cfg.white_list]):
            zs = []
            print("clear zs")

        if not (len(xs) and isinstance(xs[0], np.float64) and not math.isnan(xs[0])):
            xs = []
        if not (len(ys) and isinstance(ys[0], np.float64) and not math.isnan(ys[0])):
            ys = []
        if not (len(zs) and isinstance(zs[0], np.float64) and not math.isnan(zs[0])):
            zs = []

        return ts, xs, ys, zs, dist_vec

    def ax_plot_rpy_sigma(self, ax, cfg,
                          sigma_N=3.0,
                          x_label_prefix='',
                          y_label_prefix='sigma rpy-zyx',
                          colors=['r', 'g', 'b'],
                          labels=['x', 'y', 'z'],
                          ls=PlotLineStyle()):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts, xs, ys, zs, dist_vec = self.get_rpy_N_sigma_data(cfg, sigma_N=sigma_N)

        TrajectoryPlotUtils.ax_plot_rpy(ax=ax,ts=ts, xs=xs, ys=ys, zs=zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+" "+y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        radians=cfg.radians,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)
        TrajectoryPlotUtils.ax_plot_rpy(ax=ax,ts=ts, xs=-xs, ys=-ys, zs=-zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+" "+y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        radians=cfg.radians,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)

    def ax_plot_p_sigma(self, ax, cfg,
                          sigma_N=3.0,
                          x_label_prefix='',
                          y_label_prefix='sigma ',
                          colors=['r', 'g', 'b'],
                          labels=['x', 'y', 'z'],
                          ls=PlotLineStyle()):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts, xs, ys, zs, dist_vec = self.get_p_N_sigma_data(cfg, sigma_N=sigma_N)

        TrajectoryPlotUtils.ax_plot_pos(ax=ax, ts=ts, xs=xs, ys=ys, zs=zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+" "+y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)
        TrajectoryPlotUtils.ax_plot_pos(ax=ax, ts=ts, xs=-xs, ys=-ys, zs=-zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+" "+y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)

    def plot_pos_orient_cov(self, cfg=TrajectoryPlotConfig(), fig=None, angles=False, plot_angle=False):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        # create 2x2 grid
        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)

        self.ax_plot_pos(ax=ax1, cfg=cfg)

        if plot_angle:
            self.ax_plot_angle(ax=ax2, cfg=cfg)
        else:
            if angles:
                self.ax_plot_rpy(ax=ax2, cfg=cfg)
            else:
                self.ax_plot_q(ax=ax2, cfg=cfg)

        self.ax_plot_p_sigma(ax3, cfg=cfg, colors=['darkred', 'darkgreen', 'darkblue'])
        self.ax_plot_rpy_sigma(ax4, cfg=cfg, colors=['darkred', 'darkgreen', 'darkblue'])

        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax1, ax2, ax3, ax4