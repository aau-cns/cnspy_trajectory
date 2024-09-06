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

import numpy as np
import pandas
import matplotlib.pyplot as plt

from cnspy_csv2dataframe.PoseTypedStamped2DataFrame import PoseTypedStamped2DataFrame
from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.SpatialConverter import SpatialConverter
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryErrorType import TrajectoryErrorType
from cnspy_trajectory.TrajectoryPlotUtils import TrajectoryPlotUtils, TrajectoryPlotConfig


class TrajectoryError(Trajectory):
    # p_vec: position error vector defined via "error_type" [m]
    # q_vec: rotation error defined via "error_type"  [quaternion]
    format_type = CSVSpatialFormatType.PoseTypedStamped
    traj_err_type = None # TrajectoryErrorType
    scale = 1.0          # position scale factor

    # private metrics:
    # force access to ARMSE via method get_ARMSE()!
    __p_RMSE_vec = None       # norm of position error vector [m]
    __q_RMSE_deg_vec = None   # rotation error angle (phi of axang) [deg]
    __ARMSE_p = None          # [m]
    __ARMSE_q_deg = None      # [deg]

    def __init__(self, t_vec=None, p_vec=None, q_vec=None, scale=1.0,
                 traj_err_type=TrajectoryErrorType(err_type=EstimationErrorType.type1),
                 df=None, fn=None):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)
        self.traj_err_type = traj_err_type
        self.scale = scale

        if df is not None:
            self.load_from_DataFrame(df)
        elif fn is not None:
            self.load_from_CSV(fn)
        pass

    # overriding abstract method
    def get_rpy_vec(self):
        rpy_vec = np.zeros(np.shape(self.p_vec))
        for i in range(np.shape(self.p_vec)[0]):
            q = SpatialConverter.HTMQ_quaternion_to_Quaternion(self.q_vec[i, :])
            rpy = q.unit().rpy(order='zyx')
            rpy2= (rpy + np.pi) % (2 * np.pi) - np.pi
            rpy_vec[i, :] = rpy2

        #rpy_vec = np.unwrap(rpy_vec, axis=0)
        return rpy_vec


    def get_ARMSE(self):
        if self.__ARMSE_p is None or self.__ARMSE_q_deg is None:
            self.compute_ARMSE()
        return self.__ARMSE_p, self.__ARMSE_q_deg

    def compute_ARMSE(self):
        if self.__p_RMSE_vec is None or self.__q_RMSE_deg_vec is None:
            self.__p_RMSE_vec = TrajectoryError.compute_RMSE_p_vec(self.p_vec)
            a, self.__q_RMSE_deg_vec = TrajectoryError.compute_RMSE_q_vec(self.q_vec)

        self.__ARMSE_p = np.mean(self.__p_RMSE_vec)
        self.__ARMSE_q_deg = np.mean(self.__q_RMSE_deg_vec)

    @staticmethod
    def compute_RMSE_p_vec(p_err_arr):
        return np.sqrt(np.sum(p_err_arr ** 2, 1))

    @staticmethod
    def compute_RMSE_q_vec(q_err_arr):
        length = np.shape(q_err_arr)[0]
        rmse_rad_vec = np.zeros((length, 1))
        for i in range(length):
            R_SO3 = SpatialConverter.HTMQ_quaternion_to_SO3(q_err_arr[i, :])
            # Eq. 24 in Zhang and Scaramuzza [1]
            # Bi-invariate metric for rotations (length of geodesic on unit-sphere from identity element) [3]
            [theta, v] = R_SO3.angvec()
            rmse_rad_vec[i, :] = abs(theta)

        rmse_deg_vec = np.rad2deg(rmse_rad_vec)
        return rmse_rad_vec, rmse_deg_vec

    # overriding abstract method
    def load_from_DataFrame(self, df, fmt_type=None):
        assert (isinstance(df, pandas.DataFrame))
        if fmt_type is None:
            fmt_type = CSV2DataFrame.identify_format(dataframe=df)
        else:
            assert (isinstance(fmt_type, CSVSpatialFormatType))

        est_err_type = EstimationErrorType.none
        if fmt_type == CSVSpatialFormatType.PoseStamped:
            self.t_vec, self.p_vec, self.q_vec = \
                TUMCSV2DataFrame.from_DataFrame(df)
            self.scale = 1.0
        elif fmt_type == CSVSpatialFormatType.PoseTypedStamped:
            self.t_vec, self.p_vec, self.q_vec, scale_vec, est_err_type_vec = \
                PoseTypedStamped2DataFrame.from_DataFrame(df)
            self.scale = scale_vec[0]
            est_err_type = EstimationErrorType(est_err_type_vec[0])

        self.traj_err_type = TrajectoryErrorType(est_err_type)
        self.format_type = fmt_type

    # overriding abstract method
    def to_DataFrame(self):
        est_err_type_vec = np.repeat(str(self.traj_err_type.err_type), self.num_elems(), axis=0)
        scale_vec = np.repeat(self.scale, self.num_elems(), axis=0)
        return PoseTypedStamped2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec, scale_vec, est_err_type_vec)

    # overriding abstract method
    def clone(self):
        return TrajectoryError(t_vec=self.t_vec.copy(), p_vec=self.p_vec.copy(), q_vec=self.q_vec.copy(),
                              scale=self.scale, traj_err_type=self.traj_err_type)

    ########### PLOTTING #################
    def plot_p_err(self, cfg=TrajectoryPlotConfig(), fig=None, ax=None):
        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))
        if ax is None:
            ax = fig.add_subplot(111)


        self.ax_plot_pos(ax=ax, cfg=cfg)

        [p_err_text, R_err_text] = self.traj_err_type.error_def()
        ARMSE_p, ARMSE_q_deg = self.get_ARMSE()
        ax.set_ylabel(p_err_text + ' ARMSE ={:.2f} [m]'.format(ARMSE_p))

        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax

    def plot_rpy_err(self, cfg=TrajectoryPlotConfig(), fig=None, ax=None):

        if ax is None:
            if fig is None:
                fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))
            ax = fig.add_subplot(111)
        self.ax_plot_rpy(ax=ax, cfg=cfg)

        [p_err_text, R_err_text] = self.traj_err_type.error_def()
        ARMSE_p, ARMSE_q_deg = self.get_ARMSE()
        if cfg.radians:
            ax.set_ylabel(R_err_text + ' ARMSE ={:.2f} [rad]'.format(np.deg2rad(ARMSE_q_deg)))
        else:
            ax.set_ylabel(R_err_text + ' ARMSE ={:.2f} [deg]'.format(ARMSE_q_deg))

        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax



    @staticmethod
    def plot_pose_err(traj_est, traj_err, fig=None, cfg=None, angles=False, plot_rpy=False, traj_gt=None, axes=None):
        assert(isinstance(traj_err, TrajectoryError))
        assert(isinstance(traj_est, Trajectory))
        assert (isinstance(cfg, TrajectoryPlotConfig))

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        if axes is None or len(axes) < 4:
            # create 2x2 grid
            ax1 = fig.add_subplot(221)
            ax2 = fig.add_subplot(222)
            ax3 = fig.add_subplot(223)
            ax4 = fig.add_subplot(224)
        else:
            ax1 = axes[0]
            ax2 = axes[1]
            ax3 = axes[2]
            ax4 = axes[3]

        # Error type text:
        p_err_text, R_err_text = traj_err.traj_err_type.error_def()

        traj_est.ax_plot_pos(ax=ax1, cfg=cfg)
        if traj_gt:
            assert (isinstance(traj_gt, Trajectory))
            traj_gt.ax_plot_pos(ax=ax1, cfg=cfg, y_label_prefix='true ',
                                colors=['darkred', 'darkgreen', 'darkblue'], labels=['gt_x', 'gt_y', 'gt_z'],
                                ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
        traj_err.ax_plot_pos(ax=ax2, cfg=cfg)

        ax1.set_ylabel('position est [m]')
        ax2.set_ylabel(p_err_text + 'position err [m]')

        if angles:
            traj_est.ax_plot_rpy(ax=ax3, cfg=cfg)
            if traj_gt:
                traj_gt.ax_plot_rpy(ax=ax3, cfg=cfg, colors=['darkred', 'darkgreen', 'darkblue'],
                                    ls=PlotLineStyle(linestyle='-.', linewidth=0.5))

            if plot_rpy:
                traj_err.ax_plot_rpy(ax=ax4, cfg=cfg)
            else:
                traj_err.ax_plot_angle(ax=ax4, cfg=cfg)

            if cfg.radians:
                ax3.set_ylabel('rotation est [rad]')
                ax4.set_ylabel(R_err_text + 'rotation err [rad]')
            else:
                ax3.set_ylabel('rotation est [deg]')
                ax4.set_ylabel(R_err_text + 'rotation err [deg]')
        else:
            traj_est.ax_plot_q(ax=ax3, cfg=cfg)
            if traj_gt:
                traj_gt.ax_plot_q(ax=ax3, cfg=cfg, colors=['k', 'k', 'k', 'k'],
                                  ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
            traj_err.ax_plot_q(ax=ax4, cfg=cfg)
            ax4.set_ylabel(R_err_text + ' quaternion')

        ax1.grid(visible=True)
        ax2.grid(visible=True)
        ax3.grid(visible=True)
        ax4.grid(visible=True)

        TrajectoryPlotConfig.show_save_figure(cfg, fig)

        return fig, ax1, ax2, ax3, ax4


    @staticmethod
    def plot_pose(traj_est, traj_gt, fig=None, cfg=None, angles=False, plot_rpy=False):
        assert(isinstance(traj_gt, Trajectory))
        assert(isinstance(traj_est, Trajectory))
        assert (isinstance(cfg, TrajectoryPlotConfig))

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        # create 2x2 grid
        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)

        # Error type text:
        traj_est.ax_plot_pos(ax=ax1, cfg=cfg)
        traj_gt.ax_plot_pos(ax=ax2, cfg=cfg, y_label_prefix='true ',
                            colors=['darkred', 'darkgreen', 'darkblue'],
                            ls=PlotLineStyle(linestyle='-.'))

        ax1.set_ylabel('position est [m]')
        ax2.set_ylabel('position gt [m]')

        if angles:
            traj_est.ax_plot_rpy(ax=ax3, cfg=cfg)
            traj_gt.ax_plot_rpy(ax=ax4, cfg=cfg, colors=['darkred', 'darkgreen', 'darkblue'],
                                ls=PlotLineStyle(linestyle='-.'))

            if cfg.radians:
                ax3.set_ylabel('rotation est [rad]')
                ax4.set_ylabel('rotation gt [rad]')
            else:
                ax3.set_ylabel('rotation est [deg]')
                ax4.set_ylabel('rotation gt [deg]')
        else:
            traj_est.ax_plot_q(ax=ax3, cfg=cfg)
            traj_gt.ax_plot_q(ax=ax4, cfg=cfg,
                                  ls=PlotLineStyle(linestyle='-.'))
            ax4.set_ylabel('quaternion')

        ax1.grid(visible=True)
        ax2.grid(visible=True)
        ax3.grid(visible=True)
        ax4.grid(visible=True)

        TrajectoryPlotConfig.show_save_figure(cfg, fig)

        return fig, ax1, ax2, ax3, ax4
