import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # <--- This is important for 3d plotting

from csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from trajectory.Trajectory import Trajectory
from trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes


class TrajectoryPlotter:
    traj_obj = None
    traj_df = None
    config = None

    def __init__(self, traj_obj, config=TrajectoryPlotConfig()):
        if config.num_points > 0:
            # subsample trajectory first:
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
                      labels=['x', 'y', 'z']):
        assert len(colors) == len(labels)
        if len(colors) > 1:
            assert len(colors) == values.shape[1]
            for i in range(len(colors)):
                ax.plot(x_linespace, values[:, i],
                        colors[i] + '-', label=labels[i])
        else:
            ax.plot(x_linespace, values, colors[0] + '-', label=labels[0])

    def ax_plot_pos(self, ax, cfg):
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
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, xs, colors=['r'], labels=['x'])
        if len(ys):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, ys, colors=['g'], labels=['y'])
        if len(zs):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, zs, colors=['b'], labels=['z'])

    def ax_plot_rpy(self, ax, cfg):
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
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, xs, colors=['r'], labels=['x'])
        if len(ys):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, ys, colors=['g'], labels=['y'])
        if len(zs):
            TrajectoryPlotter.ax_plot_n_dim(ax, linespace, zs, colors=['b'], labels=['z'])

    def ax_plot_q(self, ax, cfg):
        q_vec = self.traj_obj.q_vec
        if cfg.plot_type == TrajectoryPlotTypes.plot_2D_over_dist:
            x_linespace = self.traj_obj.get_accumulated_distances()
            ax.set_xlabel('distance [m]')
        else:
            x_linespace = self.traj_obj.t_vec
            x_linespace = x_linespace - x_linespace[0]
            ax.set_xlabel('rel. time [sec]')
        ax.set_ylabel('quaternion')

        TrajectoryPlotter.ax_plot_n_dim(ax, x_linespace, q_vec, colors=['r', 'g', 'b', 'k'],
                                        labels=['qx', 'qy', 'qz', 'qw'])

    def ax_plot_pos_3D(self, ax, cfg=None, label="trajectory"):
        if cfg is None:
            cfg = self.config

        ts, xs, ys, zs, dist_vec = self.get_pos_data(cfg)

        if cfg.plot_type == TrajectoryPlotTypes.scatter_3D:
            ax.scatter(xs, ys, zs, zdir='z', label=str(label))
        elif cfg.plot_type == TrajectoryPlotTypes.plot_3D:
            ax.plot3D(xs, ys, zs, label=str(label))

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

        TrajectoryPlotter.show_save_figure(cfg, fig)

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

        ax.legend(shadow=True, fontsize='x-small')
        ax.grid()
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        TrajectoryPlotter.show_save_figure(cfg, fig)

        return fig, ax

    @staticmethod
    def multi_plot_3D(traj_plotter_list, cfg, name_list=[]):

        num_plots = len(traj_plotter_list)

        fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))
        ax = fig.add_subplot(111, projection='3d')
        colors = plt.cm.spectral(np.linspace(0.1, 0.9, num_plots))
        ax.set_prop_cycle('color', colors)

        idx = 0
        for traj in traj_plotter_list:
            traj.ax_plot_pos_3D(ax=ax, label=name_list[idx])
            idx += 1

        ax.legend(shadow=True, fontsize='x-small')
        ax.grid()
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        plt.draw()
        plt.pause(0.001)

        TrajectoryPlotter.show_save_figure(cfg, fig)

        return fig, ax

    @staticmethod
    def plot_pose_err(plotter_est, plotter_err, fig=None, cfg=None, angles=False):
        if cfg is None:
            cfg = plotter_est.config

        if fig is None:
            fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))

        ax1 = fig.add_subplot(221)
        ax2 = fig.add_subplot(222)
        ax3 = fig.add_subplot(223)
        ax4 = fig.add_subplot(224)

        plotter_est.ax_plot_pos(ax=ax1, cfg=cfg)
        plotter_err.ax_plot_pos(ax=ax2, cfg=cfg)
        ax1.set_ylabel('position est [m]')
        ax2.set_ylabel('position err [m]')

        if angles:
            plotter_est.ax_plot_rpy(ax=ax3, cfg=cfg)
            plotter_err.ax_plot_rpy(ax=ax4, cfg=cfg)
            if cfg.radians:
                ax3.set_ylabel('rotation est [rad]')
                ax4.set_ylabel('rotation err [rad]')
            else:
                ax3.set_ylabel('rotation est [deg]')
                ax4.set_ylabel('rotation err [deg]')
        else:
            plotter_est.ax_plot_q(ax=ax3, cfg=cfg)
            plotter_err.ax_plot_q(ax=ax4, cfg=cfg)
            ax4.set_ylabel('quaternion err')

        TrajectoryPlotter.show_save_figure(cfg, fig)

        return fig, ax1, ax2, ax3, ax4

    @staticmethod
    def show_save_figure(cfg, fig):
        plt.draw()
        plt.pause(0.001)
        if cfg.save_fn:
            filename = os.path.join(cfg.result_dir, cfg.save_fn)
            print("save to file: " + filename)
            plt.savefig(filename, dpi=int(cfg.dpi))
        if cfg.show:
            plt.show()
        if cfg.close_figure:
            plt.close(fig)


########################################################################################################################
#################################################### T E S T ###########################################################
########################################################################################################################
import unittest
import math


class TrajectoryPlotter_Test(unittest.TestCase):
    def load_trajectory_from_CSV(self):
        traj = Trajectory()
        traj.load_from_CSV(filename='../test/example/stamped_groundtruth.csv')
        self.assertFalse(traj.is_empty())
        return traj

    def load_trajectory2_from_CSV(self):
        traj = Trajectory()
        traj.load_from_CSV(filename='../test/example/est.csv')
        self.assertFalse(traj.is_empty())
        return traj

    def test_plot_3D(self):
        traj = self.load_trajectory_from_CSV()

        plotter = TrajectoryPlotter(traj_obj=traj, config=TrajectoryPlotConfig(show=False, close_figure=False))
        plotter.plot_3D()

    def test_plot_pose(self):
        traj = self.load_trajectory_from_CSV()

        plotter = TrajectoryPlotter(traj_obj=traj, config=TrajectoryPlotConfig(show=False, close_figure=False))
        plotter.plot_pose()
        plotter.plot_pose(angles=True)
        plotter.config.radians = False
        plotter.plot_pose(angles=True)
        plotter.plot_pose(angles=True, cfg=TrajectoryPlotConfig(show=False,
                                                                close_figure=False,
                                                                radians=False,
                                                                plot_type=TrajectoryPlotTypes.plot_2D_over_dist))
        print('done')

    def test_plot_multi(self):
        plotter1 = TrajectoryPlotter(traj_obj=self.load_trajectory_from_CSV(),
                                     config=TrajectoryPlotConfig(num_points=120000))
        plotter2 = TrajectoryPlotter(traj_obj=self.load_trajectory2_from_CSV())

        TrajectoryPlotter.multi_plot_3D([plotter1, plotter2], cfg=TrajectoryPlotConfig(show=True),
                                        name_list=['gt', 'est'])


if __name__ == "__main__":
    unittest.main()
