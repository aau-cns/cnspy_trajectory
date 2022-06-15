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
# enum
########################################################################################################################
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting
# from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting (copy, if accidentally auto-removed)
from spatialmath import base, SE3

from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_trajectory.PlotLineStyle import PlotLineStyle
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes
from cnspy_trajectory.TrajectoryPlotUtils import TrajectoryPlotUtils

from cnspy_trajectory.SpatialConverter import SpatialConverter
from cnspy_trajectory.pyplot_utils import set_axes_equal


class Trajectory:
    p_vec = None
    q_vec = None
    t_vec = None
    dist = None  # cache

    def __init__(self, t_vec=None, p_vec=None, q_vec=None, df=None):
        """
            CTOR expects either a pandas.DataFrame or a (timestamp + position + quaternion) matrix

            INPUT:
            t_vec -- [Nx1] matrix, containing N timestamps
            p_vec -- [Nx3] matrix, containing x,y,z positions
            q_vec -- [Nx4] matrix, containing quaternions [x,y,z,w]
            df -- pandas.DataFrame holding a table with ['t', 'tx', 'ty', 'tz','qx', 'qy', 'qz', 'qw']
        """
        if df is not None:
            self.load_from_DataFrame(df)
        elif t_vec is not None and p_vec is not None and q_vec is not None:
            if t_vec.ndim == 1:
                t_vec = np.array([t_vec])

            t_rows, t_cols = t_vec.shape
            p_rows, p_cols = p_vec.shape
            q_rows, q_cols = q_vec.shape
            assert (t_rows == p_rows)
            assert (t_rows == q_rows)
            assert (t_cols == 1)
            assert (p_cols == 3)
            assert (q_cols == 4)

            self.t_vec = t_vec
            self.p_vec = p_vec
            self.q_vec = q_vec

    def load_from_CSV(self, filename):
        if not os.path.isfile(filename):
            print("Trajectory: could not find file %s" % os.path.abspath(filename))
            return False

        loader = CSV2DataFrame(fn=filename)
        self.load_from_DataFrame(loader.data_frame)
        return loader.data_loaded

    def load_from_DataFrame(self, df):
        self.t_vec, self.p_vec, self.q_vec = TUMCSV2DataFrame.from_DataFrame(data_frame=df)

    def to_DataFrame(self):
        return TUMCSV2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec)

    def save_to_CSV(self, filename):
        if self.is_empty():
            return False
        df = TUMCSV2DataFrame.to_DataFrame(self.t_vec, self.p_vec, self.q_vec)
        TUMCSV2DataFrame.save_CSV(df, filename=filename, fmt=CSVSpatialFormatType.TUM)
        return True

    def is_empty(self):
        return self.t_vec is None

    def num_elems(self):
        if self.is_empty():
            return 0
        else:
            return len(self.t_vec)

    def get_distance(self):
        if self.p_vec is not None:
            if self.dist is not None:
                return self.dist
            else:
                self.dist = Trajectory.distance(self.p_vec)
                return self.dist
        else:
            return 0

    def get_accumulated_distances(self):
        return Trajectory.distances_from_start(self.p_vec)

    def get_rpy_vec(self):
        rpy_vec = np.zeros(np.shape(self.p_vec))
        for i in range(np.shape(self.p_vec)[0]):
            q = SpatialConverter.HTMQ_quaternion_to_Quaternion(self.q_vec[i, :])
            rpy_vec[i, :] = q.unit().rpy(order='zyx')

        return rpy_vec

    def get_angle_axis_vec(self, unit='rad'):
        phi_vec = np.zeros([np.shape(self.p_vec)[0], 1])
        axis_vec = np.zeros([np.shape(self.p_vec)[0], 3])
        for i in range(np.shape(self.p_vec)[0]):
            q = SpatialConverter.HTMQ_quaternion_to_Quaternion(self.q_vec[i, :])
            phi, u_vec = q.angvec(unit=unit)
            phi_vec[i, :] = phi
            axis_vec[i, :] = u_vec

        return phi_vec, axis_vec

    def transform(self, scale=1.0, t=np.zeros((3,)), R=np.identity(3)):
        p_es_aligned = np.zeros(np.shape(self.p_vec))
        q_es_aligned = np.zeros(np.shape(self.q_vec))

        T_AB = SpatialConverter.p_R_to_SE3(t, R)
        for i in range(np.shape(self.p_vec)[0]):
            T_BC = SpatialConverter.p_q_HTMQ_to_SE3(scale * self.p_vec[i, :], self.q_vec[i, :])
            # T_AC = T_AB * T_BC
            p_AC, q_AC = SpatialConverter.SE3_to_p_q_HTMQ(T_AB * T_BC)
            q_es_aligned[i, :] = q_AC
            p_es_aligned[i, :] = p_AC

        # self.p_vec = R * (scale * self.p_vec) + t

        self.p_vec = p_es_aligned
        self.q_vec = q_es_aligned

    @staticmethod
    def distances_from_start(p_vec):
        distances = np.diff(p_vec[:, 0:3], axis=0)
        distances = np.sqrt(np.sum(np.multiply(distances, distances), axis=1))
        distances = np.cumsum(distances)
        distances = np.concatenate(([0], distances))
        return distances

    @staticmethod
    def distance(p_vec):
        accum_distances = Trajectory.distances_from_start(p_vec)
        return accum_distances[-1]

    ####################################################################################################################
    ######### PLOTTING #################################################################################################
    ####################################################################################################################
    def get_pos_data(self, cfg):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts = self.t_vec
        xs = self.p_vec[:, 0]
        ys = self.p_vec[:, 1]
        zs = self.p_vec[:, 2]
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

        if cfg.scale and cfg.scale != 1.0:
            scale = float(cfg.scale)
            xs *= scale
            ys *= scale
            zs *= scale

        return ts, xs, ys, zs, dist_vec

    def get_q_data(self, cfg):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts = self.t_vec
        qx = self.q_vec[:, 0]
        qy = self.q_vec[:, 1]
        qz = self.q_vec[:, 2]
        qw = self.q_vec[:, 3]
        dist_vec = self.get_accumulated_distances()

        if cfg.white_list:
            print("white list args: " + str(cfg.white_list))
        if any([flag == 'qx' for flag in cfg.white_list]):
            qx = []
            print("clear qx")
        if any([flag == 'qy' for flag in cfg.white_list]):
            qy = []
            print("clear qy")
        if any([flag == 'qz' for flag in cfg.white_list]):
            qz = []
            print("clear qz")
        if any([flag == 'qw' for flag in cfg.white_list]):
            qw = []
            print("clear qw")

        if not (len(qx) and isinstance(qx[0], np.float64) and not math.isnan(qx[0])):
            qx = []
        if not (len(qy) and isinstance(qy[0], np.float64) and not math.isnan(qy[0])):
            qy = []
        if not (len(qz) and isinstance(qz[0], np.float64) and not math.isnan(qz[0])):
            qz = []
        if not (len(qw) and isinstance(qw[0], np.float64) and not math.isnan(qz[0])):
            qw = []
        return ts, qx, qy, qz, qw, dist_vec

    def get_rpy_data(self, cfg):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        rpy_vec = self.get_rpy_vec()

        rpy_vec = np.unwrap(rpy_vec, axis=0)
        if not cfg.radians:
            rpy_vec = np.rad2deg(rpy_vec)

        ts = self.t_vec
        rs = rpy_vec[:, 0]
        ps = rpy_vec[:, 1]
        ys = rpy_vec[:, 2]
        dist_vec = self.get_accumulated_distances()

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
            rs = []
        if not (len(ps) and isinstance(ps[0], np.float64) and not math.isnan(ps[0])):
            ps = []
        if not (len(ys) and isinstance(ys[0], np.float64) and not math.isnan(ys[0])):
            ys = []

        return ts, rs, ps, ys, dist_vec

    ###### PLOT-AXIS
    def ax_plot_pos(self, ax, cfg,
                    x_label_prefix='',
                    y_label_prefix='',
                    colors=['r', 'g', 'b'],
                    labels=['x', 'y', 'z'],
                    ls=PlotLineStyle()):

        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts, xs, ys, zs, dist_vec = self.get_pos_data(cfg)

        TrajectoryPlotUtils.ax_plot_pos(ax=ax, ts=ts, xs=xs, ys=ys, zs=zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)

    def ax_plot_rpy(self, ax, cfg,
                    x_label_prefix='',
                    y_label_prefix='Rz(y)Ry(p)Rx(r)',
                    colors=['r', 'g', 'b'],
                    labels=['x', 'y', 'z'],
                    ls=PlotLineStyle()):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts, xs, ys, zs, dist_vec = self.get_rpy_data(cfg)

        TrajectoryPlotUtils.ax_plot_rpy(ax=ax,ts=ts, xs=xs, ys=ys, zs=zs, dist_vec=dist_vec,
                                        x_label_prefix=x_label_prefix, y_label_prefix=y_label_prefix,
                                        plot_type=cfg.plot_type,
                                        radians=cfg.radians,
                                        relative_time=cfg.relative_time,
                                        colors=colors, labels=labels, ls=ls)

    def ax_plot_q(self, ax, cfg,
                    x_label_prefix='',
                    y_label_prefix='',
                    colors=['r', 'g', 'b', 'k'],
                    labels=['qx', 'qy', 'qz', 'qw'],
                    ls=PlotLineStyle()):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts, qx, qy, qz, qw, dist_vec = self.get_q_data(cfg)

        TrajectoryPlotUtils.ax_plot_q(ax=ax, ts=ts, qx=qx, qy=qy, qz=qz, qw=qw, dist_vec=dist_vec,
                                      x_label_prefix=x_label_prefix,
                                      y_label_prefix=y_label_prefix,
                                      plot_type=cfg.plot_type,
                                      relative_time=cfg.relative_time,
                                      colors=colors, labels=labels, ls=ls)

    def ax_plot_angle(self, ax, cfg,
                      x_label_prefix='',
                      y_label_prefix='angle',
                      colors=['r'],
                      labels=['phi'], ls=PlotLineStyle()):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        unit = 'deg'
        if cfg.radians:
            unit='rad'
        phi_vec, axis_vec = self.get_angle_axis_vec(unit=unit)
        dist_vec = self.get_distance()
        ts  = self.t_vec

        TrajectoryPlotUtils.ax_plot_angle(ax=ax, phi_vec=phi_vec, ts=ts, dist_vec=dist_vec, radians=cfg.radians,
                                          x_label_prefix=x_label_prefix,
                                          y_label_prefix=y_label_prefix,
                                          plot_type=cfg.plot_type,
                                          relative_time=cfg.relative_time,
                                          colors=colors, labels=labels, ls=ls)

    def ax_plot_frames_3D(self, ax, cfg, plot_origin=True, origin_name="0", num_markers=0):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        if plot_origin and not self.traj_obj.is_empty():
            T = SE3(self.p_vec[0, 0], self.p_vec[0, 1], self.p_vec[0, 2])
            base.trplot(T.A, axes=ax, frame=origin_name, rviz=True, length=1, width=0.2, block=False)

        if num_markers > 0 and not self.traj_obj.is_empty():
            for i in range(1, self.num_elems(), int(self.num_elems() / num_markers)):
                T_i = SpatialConverter.p_q_HTMQ_to_SE3(self.p_vec[i, :], self.q_vec[i, :])
                base.trplot(T_i.A, axes=ax, rviz=True,
                            length=1, width=0.1, block=False)
        pass

    def ax_plot_pos_3D(self, ax, cfg, label='cnspy_trajectory'):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        ts, xs, ys, zs, dist_vec = self.get_pos_data(cfg)

        if cfg.plot_type == TrajectoryPlotTypes.scatter_3D:
            ax.scatter(xs, ys, zs, zdir='z', label=str(label))
        elif cfg.plot_type == TrajectoryPlotTypes.plot_3D:
            ax.plot3D(xs, ys, zs, label=str(label))
        pass

    def plot_pose(self, cfg=TrajectoryPlotConfig(), fig=None, angles=False, plot_angle=False):
        assert (isinstance(cfg, TrajectoryPlotConfig))

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        ax1 = fig.add_subplot(211)
        self.ax_plot_pos(ax=ax1, cfg=cfg)
        ax2 = fig.add_subplot(212)

        if plot_angle:
            self.ax_plot_angle(ax=ax2, cfg=cfg)
        else:
            if angles:
                self.ax_plot_rpy(ax=ax2, cfg=cfg)
            else:
                self.ax_plot_q(ax=ax2, cfg=cfg)

        TrajectoryPlotConfig.show_save_figure(cfg, fig)
        return fig, ax1, ax2

    def plot_3D(self, fig=None, ax=None, cfg=TrajectoryPlotConfig()):
        assert (isinstance(cfg, TrajectoryPlotConfig))

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

    ####################################################################################################################