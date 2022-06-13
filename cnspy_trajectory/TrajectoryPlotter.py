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
# Requirements:
# numpy, matplotlib, cnspy_csv2dataframe, cnspy_trajectory
########################################################################################################################
import os
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting
# from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting (copy, if accidentally auto-removed)
from spatialmath import base, SE3

from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryEstimated import TrajectoryEstimated
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes
from cnspy_trajectory.pyplot_utils import set_axes_equal
from cnspy_trajectory.SpatialConverter import SpatialConverter

# TODO: extract some features into dedicated plotting class:
# TODOs PosePlotter:
#  - TODO: ax_plot_n_dim, ax_plot_pos/rpq/q, ax_plot_pos_3D,
#  - TODO: plot_pose
#  - TODO: show_save_figure
#  - TODO: plot 3 sigma bounds
#  - TODO: legend show ARMSE


class TrajectoryPlotter:
    traj_obj = None
    traj_df = None
    config = None

    def __init__(self, traj_obj, config=TrajectoryPlotConfig()):
        if config.num_points > 0:
            # subsample cnspy_trajectory first:
            df = traj_obj.to_DataFrame()
            self.traj_df = CSV2DataFrame.subsample_DataFrame(df, num_max_points=config.num_points)
            self.traj_obj = Trajectory(df=self.traj_df)
        else:
            self.traj_obj = traj_obj
            self.traj_df = traj_obj.to_DataFrame()

        self.config = config




    @staticmethod
    def multi_plot_3D(traj_plotter_list, cfg, name_list=[]):
        assert (isinstance(cfg, TrajectoryPlotConfig))
        num_plots = len(traj_plotter_list)

        fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))
        ax = fig.add_subplot(111, projection='3d')

        # https://stackoverflow.com/questions/51452112/how-to-fix-cm-spectral-module-matplotlib-cm-has-no-attribute-spectral
        cmap = plt.cm.get_cmap("Spectral")
        colors = cmap(np.linspace(0.1, 0.9, num_plots))
        ax.set_prop_cycle('color', colors)

        idx = 0
        for traj in traj_plotter_list:
            traj.ax_plot_pos_3D(ax=ax, label=name_list[idx])
            idx += 1

        ax.legend(shadow=True, fontsize='x-small')
        ax.grid(b=True)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        TrajectoryPlotConfig.set_view_angle(cfg=cfg, ax=ax)
        plt.draw()
        plt.pause(0.001)

        TrajectoryPlotConfig.show_save_figure(cfg, fig)

        return fig, ax

    @staticmethod
    def plot_pose_err(plotter_est, plotter_err, fig=None, cfg=None, angles=False, plotter_gt=None, local_p_err=True,
                      local_R_err=True):
        assert(isinstance(plotter_err, TrajectoryPlotter))
        assert(isinstance(plotter_est, TrajectoryPlotter))

        if cfg is None:
            cfg = plotter_est.config
        else:
            assert (isinstance(cfg, TrajectoryPlotConfig))

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        # create 2x2 grid
        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)

        # Error type text:
        text_p_err = 'local '
        if not local_p_err:
            text_p_err = 'global '
        # Error type text:
        text_R_err = 'local '
        if not local_R_err:
            text_R_err = 'global '

        plotter_est.ax_plot_pos(ax=ax1, cfg=cfg)
        if plotter_gt:
            plotter_gt.ax_plot_pos(ax=ax1, cfg=cfg, colors=['k', 'k', 'k'], labels=['gt_x', 'gt_y', 'gt_z'],
                                   ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
        plotter_err.ax_plot_pos(ax=ax2, cfg=cfg)

        ax1.set_ylabel('position est [m]')
        ax2.set_ylabel(text_p_err + 'position err [m]')

        if angles:
            plotter_est.ax_plot_rpy(ax=ax3, cfg=cfg)
            if plotter_gt:
                plotter_gt.ax_plot_rpy(ax=ax3, cfg=cfg, colors=['k', 'k', 'k'],
                                       ls=PlotLineStyle(linestyle='-.', linewidth=0.5))

            plotter_err.ax_plot_angle(ax=ax4, cfg=cfg)

            if cfg.radians:
                ax3.set_ylabel('rotation est [rad]')
                ax4.set_ylabel(text_R_err + 'rotation err [rad]')
            else:
                ax3.set_ylabel('rotation est [deg]')
                ax4.set_ylabel(text_R_err + 'rotation err [deg]')
        else:
            plotter_est.ax_plot_q(ax=ax3, cfg=cfg)
            if plotter_gt:
                plotter_gt.ax_plot_q(ax=ax3, cfg=cfg, colors=['k', 'k', 'k', 'k'],
                                     ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
            plotter_err.ax_plot_q(ax=ax4, cfg=cfg)
            ax4.set_ylabel(text_R_err + 'quaternion err')

        ax1.grid(visible=True)
        ax2.grid(visible=True)
        ax3.grid(visible=True)
        ax4.grid(visible=True)
        TrajectoryPlotConfig.show_save_figure(cfg, fig)

        return fig, ax1, ax2, ax3, ax4

