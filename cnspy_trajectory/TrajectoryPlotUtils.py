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
# Requirements:
# numpy, matplotlib, cnspy_csv2dataframe, cnspy_trajectory
########################################################################################################################
import os
import numpy as np
import math
import matplotlib.pyplot as plt
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes

class TrajectoryPlotUtils:

    @staticmethod
    def ax_plot_n_dim(ax, x_linespace, values,
                      colors=['r', 'g', 'b'],
                      labels=['x', 'y', 'z'], ls=PlotLineStyle()):
        assert len(colors) == len(labels)
        if len(colors) > 1:
            assert len(colors) == values.shape[1]
            for i in range(len(colors)):
                ax.plot(x_linespace, values[:, i],
                        color=colors[i], label=labels[i], linestyle=ls.linestyle, linewidth=ls.linewidth,
                        marker=ls.marker)
        else:
            ax.plot(x_linespace, values, color=colors[0], label=labels[0], linestyle=ls.linestyle,
                    linewidth=ls.linewidth)
    @staticmethod
    def ax_x_linespace(ax, ts=None, dist_vec=None, relative_time=True, plot_type=None, x_label_prefix=''):
        if plot_type == TrajectoryPlotTypes.plot_2D_over_dist:
            x_arr = dist_vec
            ax.set_xlabel(x_label_prefix + 'distance [m]')
        else:
            x_arr = ts
            if relative_time:
                x_arr = x_arr - x_arr[0]
                ax.set_xlabel(x_label_prefix + 'rel. time [sec]')
            else:
                ax.set_xlabel(x_label_prefix + 'time [sec]')
        return x_arr

    @staticmethod
    def ax_plot_pos(ax, ts=None, xs=None, ys=None, zs=None, dist_vec=None,
                    x_label_prefix='',
                    y_label_prefix='',
                    plot_type=TrajectoryPlotTypes.plot_2D_over_dist,
                    relative_time=True,
                    colors=['r', 'g', 'b'], labels=['x', 'y', 'z'],
                    ls=PlotLineStyle()):

        x_arr = TrajectoryPlotUtils.ax_x_linespace(ax=ax, ts=ts, dist_vec=dist_vec,
                                                   relative_time=relative_time, plot_type=plot_type,
                                                   x_label_prefix=x_label_prefix)
        ax.set_ylabel(y_label_prefix + 'position [m]')

        if xs is not None and len(xs):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, xs, colors=[colors[0]], labels=[labels[0]], ls=ls)
        if ys is not None and len(ys):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, ys, colors=[colors[1]], labels=[labels[1]], ls=ls)
        if zs is not None and len(zs):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, zs, colors=[colors[2]], labels=[labels[2]], ls=ls)

    @staticmethod
    def ax_plot_rpy(ax, ts=None, xs=None, ys=None, zs=None, dist_vec=None,
                    x_label_prefix='',
                    y_label_prefix='Rz(y)Ry(p)Rx(r)',
                    plot_type=TrajectoryPlotTypes.plot_2D_over_dist,
                    radians=True,
                    relative_time=True,
                    colors=['r', 'g', 'b'], labels=['x', 'y', 'z'],
                    ls=PlotLineStyle()):

        x_arr = TrajectoryPlotUtils.ax_x_linespace(ax=ax, ts=ts, dist_vec=dist_vec,
                                                   relative_time=relative_time, plot_type=plot_type,
                                                   x_label_prefix=x_label_prefix)
        if radians:
            ax.set_ylabel(y_label_prefix + ' [rad]')
        else:
            ax.set_ylabel(y_label_prefix + ' [deg]')

        if len(xs):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, xs, colors=[colors[0]], labels=[labels[0]], ls=ls)
        if len(ys):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, ys, colors=[colors[1]], labels=[labels[1]], ls=ls)
        if len(zs):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, zs, colors=[colors[2]], labels=[labels[2]], ls=ls)



    @staticmethod
    def ax_plot_q(ax, ts=None, qx=None, qy=None, qz=None, qw=None, dist_vec=None,
                  x_label_prefix='',
                  y_label_prefix='',
                  plot_type=TrajectoryPlotTypes.plot_2D_over_dist,
                  relative_time=True,
                  colors=['r', 'g', 'b', 'k'],
                  labels=['qx', 'qy', 'qz', 'qw'], ls=PlotLineStyle()):

        x_arr = TrajectoryPlotUtils.ax_x_linespace(ax=ax, ts=ts, dist_vec=dist_vec,
                                                   relative_time=relative_time, plot_type=plot_type,
                                                   x_label_prefix=x_label_prefix)
        ax.set_ylabel(y_label_prefix + 'quaternion')

        if len(qx):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, qx, colors=[colors[0]], labels=[labels[0]], ls=ls)
        if len(qy):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, qy, colors=[colors[1]], labels=[labels[1]], ls=ls)
        if len(qz):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, qz, colors=[colors[2]], labels=[labels[2]], ls=ls)
        if len(qw):
            TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, qw, colors=[colors[3]], labels=[labels[3]], ls=ls)

    @staticmethod
    def ax_plot_angle(ax, phi_vec, ts=None, dist_vec=None, radians=True,
                      x_label_prefix='',
                      y_label_prefix='angle',
                      plot_type=TrajectoryPlotTypes.plot_2D_over_dist,
                      relative_time=True,
                      colors=['r'],
                      labels=['phi'],
                      ls=PlotLineStyle()):
        x_arr = TrajectoryPlotUtils.ax_x_linespace(ax=ax, ts=ts, dist_vec=dist_vec,
                                                   relative_time=relative_time, plot_type=plot_type,
                                                   x_label_prefix=x_label_prefix)

        if radians:
            ax.set_ylabel(y_label_prefix + '[rad]')
        else:
            ax.set_ylabel(y_label_prefix + '[deg]')

        TrajectoryPlotUtils.ax_plot_n_dim(ax, x_arr, phi_vec, colors=colors,
                                        labels=labels, ls=ls)