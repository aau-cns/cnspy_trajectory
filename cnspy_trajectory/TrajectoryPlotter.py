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
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryEstimated import TrajectoryEstimated
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes
from cnspy_trajectory.pyplot_utils import set_axes_equal
from cnspy_trajectory.SpatialConverter import SpatialConverter

# TODO: extract some features into dedicated plotting class:
# PosePlotter
#  - ax_plot_n_dim, ax_plot_pos/rpq/q, ax_plot_pos_3D,
#  - plot_pose
#  - show_save_figure
#  - plot 3 sigma bounds
#  - legend show ARMSE

class TrajectoryPlotter:
    traj_obj = None
    traj_df = None
    config = None

    def __init__(self, traj_obj, config=TrajectoryPlotConfig()):
        if config.num_points > 0:
            # subsample cnspy_trajectory first:
            df = traj_obj.to_DataFrame()
            self.traj_df = TUMCSV2DataFrame.subsample_DataFrame(df, num_max_points=config.num_points)
            self.traj_obj = Trajectory(df=self.traj_df)
        else:
            self.traj_obj = traj_obj
            self.traj_df = traj_obj.to_DataFrame()

        self.config = config

    def get_pos_data(self, cfg):
        data_dict = TUMCSV2DataFrame.DataFrame_to_numpy_dict(self.traj_df)
        ts = data_dict['t']
        xs = data_dict['tx']
        ys = data_dict['ty']
        zs = data_dict['tz']
        dist_vec = self.traj_obj.get_accumulated_distances()

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

        if cfg.scale and cfg.scale != 1.0:
            scale = float(cfg.scale)
            xs *= scale
            ys *= scale
            zs *= scale

        return ts, xs, ys, zs, dist_vec

    def get_rpy_data(self, cfg):
        rpy_vec = self.traj_obj.get_rpy_vec()

        rpy_vec = np.unwrap(rpy_vec, axis=0)
        if not cfg.radians:
            rpy_vec = np.rad2deg(rpy_vec)

        ts = self.traj_obj.t_vec
        rs = rpy_vec[:, 0]
        ps = rpy_vec[:, 1]
        ys = rpy_vec[:, 2]
        dist_vec = self.traj_obj.get_accumulated_distances()

        if cfg.white_list:
            print("white list args: " + str(cfg.white_list))
        if any([flag == 'roll' for flag in cfg.white_list]):
            rs = []
            print("clear rs")
        if any([flag == 'pitch' for flag in cfg.white_list]):
            ps = []
            print("clear ps")
        if any([flag == 'yaw' for flag in cfg.white_list]):
            ys = []
            print("clear ys")

        if not (len(rs) and isinstance(rs[0], np.float64) and not math.isnan(rs[0])):
            xs = []
        if not (len(ps) and isinstance(ps[0], np.float64) and not math.isnan(ps[0])):
            ps = []
        if not (len(ys) and isinstance(ys[0], np.float64) and not math.isnan(ys[0])):
            ys = []

        return ts, rs, ps, ys, dist_vec

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

    def ax_plot_pos(self, ax, cfg, colors=['r', 'g', 'b'], labels=['x', 'y', 'z'], ls=PlotLineStyle()):
        ts, xs, ys, zs, dist_vec = self.get_pos_data(cfg)

        if cfg.plot_type == TrajectoryPlotTypes.plot_2D_over_dist:
            linespace = dist_vec
            ax.set_xlabel('distance [m]')
        else:
            ts = ts - ts[0]
            linespace = ts
            ax.set_xlabel('rel. time [sec]')
        ax.set_ylabel('position [m]')

        if len(xs):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, xs, colors=[colors[0]], labels=[labels[0]], ls=ls)
        if len(ys):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, ys, colors=[colors[1]], labels=[labels[1]], ls=ls)
        if len(zs):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, zs, colors=[colors[2]], labels=[labels[2]], ls=ls)

    def ax_plot_rpy(self, ax, cfg, colors=['r', 'g', 'b'], labels=['x', 'y', 'z'], ls=PlotLineStyle()):
        ts, xs, ys, zs, dist_vec = self.get_rpy_data(cfg)

        if cfg.plot_type == TrajectoryPlotTypes.plot_2D_over_dist:
            linespace = dist_vec
            ax.set_xlabel('distance [m]')
        else:
            ts = ts - ts[0]
            linespace = ts
            ax.set_xlabel('rel. time [sec]')

        if cfg.radians:
            ax.set_ylabel('rotation [rad]')
        else:
            ax.set_ylabel('rotation [deg]')

        if len(xs):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, xs, colors=[colors[0]], labels=[labels[0]], ls=ls)
        if len(ys):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, ys, colors=[colors[1]], labels=[labels[1]], ls=ls)
        if len(zs):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, zs, colors=[colors[2]], labels=[labels[2]], ls=ls)

    def ax_plot_q(self, ax, cfg, colors=['r', 'g', 'b', 'k'],
                  labels=['qx', 'qy', 'qz', 'qw'], ls=PlotLineStyle()):
        q_vec = self.traj_obj.q_vec
        if cfg.plot_type == TrajectoryPlotTypes.plot_2D_over_dist:
            x_linespace = self.traj_obj.get_accumulated_distances()
            ax.set_xlabel('distance [m]')
        else:
            x_linespace = self.traj_obj.t_vec
            x_linespace = x_linespace - x_linespace[0]
            ax.set_xlabel('rel. time [sec]')
        ax.set_ylabel('quaternion')

        TrajectoryPlotter.ax_plot_n_dim(ax, x_linespace, q_vec, colors=colors,
                                        labels=labels)

    def ax_plot_pos_3D(self, ax, cfg=None, label="cnspy_trajectory"):
        if cfg is None:
            cfg = self.config

        ts, xs, ys, zs, dist_vec = self.get_pos_data(cfg)

        if cfg.plot_type == TrajectoryPlotTypes.scatter_3D:
            ax.scatter(xs, ys, zs, zdir='z', label=str(label))
        elif cfg.plot_type == TrajectoryPlotTypes.plot_3D:
            ax.plot3D(xs, ys, zs, label=str(label))

    def ax_plot_frames_3D(self, ax, cfg=None, plot_origin=True, origin_name="0", num_markers=0):
        if cfg is None:
            cfg = self.config

        if plot_origin and not self.traj_obj.is_empty():
            T = SE3(self.traj_obj.p_vec[0,0], self.traj_obj.p_vec[0,1], self.traj_obj.p_vec[0,2])
            base.trplot(T.A, axes=ax, frame=origin_name, rviz=True, length=1, width=0.2, block=False)

        if num_markers > 0 and not self.traj_obj.is_empty():
            q_vec = self.traj_obj.q_vec
            p_vec = self.traj_obj.p_vec
            for i in range(1, self.traj_obj.num_elems(), int(self.traj_obj.num_elems()/num_markers)):
                T_i = SpatialConverter.p_q_HTMQ_to_SE3(self.traj_obj.p_vec[i, :], self.traj_obj.q_vec[i, :])
                base.trplot(T_i.A, axes=ax, rviz=True,
                            length=1, width=0.1, block=False)



    def plot_pose(self, fig=None, cfg=None, angles=False):
        if cfg is None:
            cfg = self.config

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        ax1 = fig.add_subplot(211)
        self.ax_plot_pos(ax=ax1, cfg=cfg)
        ax2 = fig.add_subplot(212)

        if angles:
            self.ax_plot_rpy(ax=ax2, cfg=cfg)
        else:
            self.ax_plot_q(ax=ax2, cfg=cfg)

        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax1, ax2

    def plot_3D(self, fig=None, ax=None, cfg=None):
        if cfg is None:
            cfg = self.config

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        if ax is None:
            ax = fig.add_subplot(111, projection='3d')

        if cfg.title:
            ax.set_title(cfg.title)
        else:
            if cfg.plot_type == TrajectoryPlotTypes.scatter_3D:
                ax.set_title("Scatter Plot")
            else:
                ax.set_title("Plot3D")

        self.ax_plot_pos_3D(ax=ax, cfg=cfg)
        self.ax_plot_frames_3D(ax=ax, cfg=cfg, plot_origin=True, num_markers=10)

        set_axes_equal(ax)
        ax.legend(shadow=True, fontsize='x-small')
        ax.grid(b=True)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        set_axes_equal(ax)
        TrajectoryPlotConfig.set_view_angle(cfg=cfg, ax=ax)
        TrajectoryPlotConfig.show_save_figure(cfg, fig=fig)

        return fig, ax

    @staticmethod
    def multi_plot_3D(traj_plotter_list, cfg, name_list=[]):

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
    def plot_pose_err(plotter_est, plotter_err, fig=None, cfg=None, angles=False, plotter_gt=None):
        assert(isinstance(plotter_err, TrajectoryPlotter))
        assert(isinstance(plotter_est, TrajectoryPlotter))

        if cfg is None:
            cfg = plotter_est.config

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)

        plotter_est.ax_plot_pos(ax=ax1, cfg=cfg)
        if plotter_gt:
            plotter_gt.ax_plot_pos(ax=ax1, cfg=cfg, colors=['k', 'k', 'k'], labels=['gt_x', 'gt_y', 'gt_z'],
                                   ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
        plotter_err.ax_plot_pos(ax=ax2, cfg=cfg)
        ax1.set_ylabel('position est [m]')
        ax2.set_ylabel('position err [m]')

        if angles:
            plotter_est.ax_plot_rpy(ax=ax3, cfg=cfg)
            if plotter_gt:
                plotter_gt.ax_plot_rpy(ax=ax3, cfg=cfg, colors=['k', 'k', 'k'],
                                       ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
            plotter_err.ax_plot_rpy(ax=ax4, cfg=cfg)
            if cfg.radians:
                ax3.set_ylabel('rotation est [rad]')
                ax4.set_ylabel('rotation err [rad]')
            else:
                ax3.set_ylabel('rotation est [deg]')
                ax4.set_ylabel('rotation err [deg]')
        else:
            plotter_est.ax_plot_q(ax=ax3, cfg=cfg)
            if plotter_gt:
                plotter_gt.ax_plot_q(ax=ax3, cfg=cfg, colors=['k', 'k', 'k', 'k'],
                                     ls=PlotLineStyle(linestyle='-.', linewidth=0.5))
            plotter_err.ax_plot_q(ax=ax4, cfg=cfg)
            ax4.set_ylabel('quaternion err')

        ax1.grid(b=True)
        ax2.grid(b=True)
        ax3.grid(b=True)
        ax4.grid(b=True)
        TrajectoryPlotConfig.show_save_figure(cfg, fig)

        return fig, ax1, ax2, ax3, ax4

