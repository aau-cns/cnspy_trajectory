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
    format = CSVSpatialFormat(est_err_type=EstimationErrorType.type5,
                              err_rep_type=ErrorRepresentationType.R_small_theta)

    def __init__(self, t_vec=None, p_vec=None, q_vec=None,
                 Sigma_p_vec=None, Sigma_R_vec=None,
                 Sigma_pR_vec=None, Sigma_T_vec=None,
                 df=None, fmt=None):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)
        self.Sigma_p_vec = Sigma_p_vec
        self.Sigma_R_vec = Sigma_R_vec
        self.Sigma_pR_vec = Sigma_pR_vec
        self.Sigma_T_vec = Sigma_T_vec

        if df is not None:
            self.load_from_DataFrame(df)

        if fmt is not None:
            self.set_format(fmt)

    def set_format(self, fmt):
        if fmt is not None and isinstance(fmt, CSVSpatialFormat):
            self.format = fmt

    def get_format(self):
        return self.format

    def load_from_CSV(self, filename):
        if not os.path.isfile(filename):
            print("Trajectory: could not find file %s" % os.path.abspath(filename))
            return False

        loader = CSV2DataFrame(filename=filename)
        if loader.data_loaded:
            self.load_from_DataFrame(df=loader.data_frame, fmt=loader.format)
            return True

        return False

    def load_from_DataFrame(self, df, fmt=None):
        assert (isinstance(df, pandas.DataFrame))
        if fmt is None:
            fmt = self.format
        else:
            self.set_format(fmt)

        # TODO: different uncertainties can be loaded! either full pose covariance or position and orientation separated
        if fmt.type == CSVSpatialFormatType.PosOrientWithCov:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec, \
                self.Sigma_R_vec = PosOrientWithCov2DataFrame.DataFrame_to_TPQCov(data_frame=df)
        elif fmt.type == CSVSpatialFormatType.PoseWithCov:
            self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec = \
                PoseWithCov2DataFrame.DataFrame_to_TPQCov(data_frame=df)
        else:
            print('Error: format not supported yet')
            assert(False)

    def to_DataFrame(self):
        if self.format.type == CSVSpatialFormatType.PosOrientWithCov:
            return PosOrientWithCov2DataFrame.TPQCov_to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec,
                                                         self.Sigma_R_vec)
        elif self.format.type == CSVSpatialFormatType.PoseWithCov:
            return PoseWithCov2DataFrame.TPQCov_to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_T_vec)
        elif self.format.type == CSVSpatialFormatType.TUM:
            return TUMCSV2DataFrame.TPQ_to_DataFrame(self.t_vec, self.p_vec, self.q_vec)
        else:
            print('Error: format [' + str(self.format.type) + '] not supported yet')
            assert (False)

    def save_to_CSV(self, filename):
        if self.is_empty():
            return False
        df = self.to_DataFrame()
        CSV2DataFrame.save_CSV(df, filename=filename, fmt=self.get_format())
        return True

    ####################################################################################################################
    ######### PLOTTING #################################################################################################
    ####################################################################################################################
    def get_p_N_sigma_data(self, cfg, sigma_N=3.0):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        sigma_p_diag_vec = np.zeros(np.shape(self.p_vec))
        if self.format.type == CSVSpatialFormatType.PosOrientWithCov:
            for i in range(self.num_elems()):
                P = self.Sigma_p_vec[i]
                sigma_p_diag_vec[i] = np.sqrt(np.diag(P))

        elif self.format.type == CSVSpatialFormatType.PoseWithCov:
            for i in range(self.num_elems()):
                P = self.Sigma_T_vec[i]
                sigma_p_diag_vec[i] = np.sqrt(np.diag(P[[0,1,2], [0,1,2]]))
        else:
            print('format ' + str(self.format.type) + ' not supported!')
            assert False
        sigma_N_diag_vec = sigma_p_diag_vec * sigma_N


        # sigma3_diag_vec = np.zeros(np.shape(self.p_vec))
        # if self.format.rotation_error_representation == ErrorRepresentationType.R_small_theta:
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

        sigma_theta_diag_vec = np.zeros(np.shape(self.p_vec))
        if self.format.type == CSVSpatialFormatType.PosOrientWithCov:
            for i in range(self.num_elems()):
                sigma_theta_diag_vec[i] = np.sqrt(np.diag(self.Sigma_R_vec[i]))
        elif self.format.type == CSVSpatialFormatType.PoseWithCov:
            for i in range(self.num_elems()):
                P = self.Sigma_T_vec[i]
                sigma_theta_diag_vec[i] = np.sqrt(np.diag(P[[0, 1, 2], [0, 1, 2]]))
        else:
            print('format ' + str(self.format.type) + ' not supported!')
            assert False

        sigma_rpy_diag_vec = np.zeros(np.shape(self.p_vec))

        unit='rad'
        if not cfg.radians:
            unit='deg'

        # Converts small angle approximations of covariance to rpy angles: R =  Rz(y)Ry(p)Rx(r)
        if self.format.rotation_error_representation == ErrorRepresentationType.R_small_theta:
            for i in range(self.num_elems()):
                R = SpatialConverter.theta_R2rot(sigma_theta_diag_vec[i])
                sigma_rpy_diag_vec[i] = SpatialConverter.rot2rpy(R, unit=unit)
        elif self.format.rotation_error_representation == ErrorRepresentationType.q_small_theta:
            for i in range(self.num_elems()):
                R = SpatialConverter.theta_q2rot(sigma_theta_diag_vec[i])
                sigma_rpy_diag_vec[i] = SpatialConverter.rot2rpy(R, unit=unit)
        elif self.format.rotation_error_representation == ErrorRepresentationType.so3_theta:
            for i in range(self.num_elems()):
                R = SpatialConverter.theta_so3_2rot(sigma_theta_diag_vec[i])
                sigma_rpy_diag_vec[i] = SpatialConverter.rot2rpy(R, unit=unit)
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
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        radians=cfg.radians,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)
        TrajectoryPlotUtils.ax_plot_rpy(ax=ax,ts=ts, xs=-xs, ys=-ys, zs=-zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+y_label_prefix,
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
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)
        TrajectoryPlotUtils.ax_plot_pos(ax=ax, ts=ts, xs=-xs, ys=-ys, zs=-zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=str(sigma_N)+y_label_prefix,
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

        self.ax_plot_p_sigma(ax3, cfg=cfg)
        self.ax_plot_rpy_sigma(ax4, cfg=cfg)

        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax1, ax2, ax3, ax4