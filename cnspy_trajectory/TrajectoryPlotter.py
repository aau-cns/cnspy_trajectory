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
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting
# from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting (copy, if accidentally auto-removed)
from spatialmath import base, SE3

from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryEstimated import TrajectoryEstimated
from cnspy_trajectory.TrajectoryError import TrajectoryError
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

    @staticmethod
    def plot_pose_err(traj_est, traj_err, cfg, fig=None, traj_gt=None):
        assert (isinstance(traj_est, Trajectory))
        assert (isinstance(traj_err, TrajectoryError))
        assert (isinstance(cfg, TrajectoryPlotConfig))

        # disable functionality
        cfg_ = copy.deepcopy(cfg)
        cfg_.show = False
        cfg_.close = False
        cfg_.save_fn = None
        fig, ax1, ax2, ax3, ax4 = TrajectoryError.plot_pose_err(traj_est=traj_est, traj_err=traj_err,
                                                                cfg=cfg_, fig=fig,
                                                                angles=True, plot_rpy=True,
                                                                traj_gt=traj_gt)
        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax1, ax2, ax3, ax4
    @staticmethod
    def plot_pose_err_cov(traj_est, traj_err, cfg, fig=None, traj_gt=None):
        assert (isinstance(traj_est, TrajectoryEstimated))
        assert (isinstance(traj_err, TrajectoryError))
        assert (isinstance(cfg, TrajectoryPlotConfig))

        # disable functionality
        cfg_ = copy.deepcopy(cfg)
        cfg_.show = False
        cfg_.close = False
        cfg_.save_fn = None
        fig, ax1, ax2, ax3, ax4 = TrajectoryError.plot_pose_err(traj_est=traj_est, traj_err=traj_err,
                                                                cfg=cfg_, fig=fig,
                                                                angles=True, plot_rpy=True,
                                                                traj_gt=traj_gt)
        traj_est.ax_plot_p_sigma(ax2, cfg=cfg_, colors=['darkred', 'darkgreen', 'darkblue'], ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
        traj_est.ax_plot_rpy_sigma(ax4, cfg=cfg_, colors=['darkred', 'darkgreen', 'darkblue'], ls=PlotLineStyle(linestyle='-.', linewidth=0.5))

        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax1, ax2, ax3, ax4

    @staticmethod
    def multi_plot_3D(traj_list, cfg, name_list=None, color_map="gist_rainbow"):
        assert (isinstance(cfg, TrajectoryPlotConfig))
        num_plots = len(traj_list)

        fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))
        ax = fig.add_subplot(111, projection='3d')

        # https://stackoverflow.com/questions/51452112/how-to-fix-cm-spectral-module-matplotlib-cm-has-no-attribute-spectral
        # choose among: https://matplotlib.org/stable/users/explain/colors/colormaps.html#miscellaneous
        cmap = plt.cm.get_cmap(color_map)
        colors = cmap(np.linspace(0.01, 0.99, num_plots))
        ax.set_prop_cycle('color', colors)

        idx = 0
        for traj in traj_list:
            label = "traj" + str(idx)
            if name_list:
                label = name_list[idx]
            traj.ax_plot_pos_3D(ax=ax, cfg=cfg, label=label)
            idx += 1

        ax.legend(shadow=True, fontsize='x-small')
        ax.grid(b=True)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        set_axes_equal(ax)
        TrajectoryPlotConfig.set_view_angle(cfg=cfg, ax=ax)
        plt.draw()
        plt.pause(0.001)

        TrajectoryPlotConfig.show_save_figure(cfg, fig)

        return fig, ax


