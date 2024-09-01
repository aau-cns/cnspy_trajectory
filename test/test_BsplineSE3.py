#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2024, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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
import unittest

from cnspy_trajectory.TrajectoryPlotter import *
from cnspy_trajectory.BsplineSE3 import *
SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class BsplineSE3_Test(unittest.TestCase):
    def test_interp(self):
        traj_sub = Trajectory(fn=SAMPLE_DATA_DIR+'/pose-sub.csv')
        traj = Trajectory(fn=SAMPLE_DATA_DIR+'/pose-raw.csv')

        spline = BsplineSE3(traj=traj_sub)

        traj_cubic = spline.get_trajectory(traj.t_vec, interp_type=TrajectoryInterpolationType.cubic)
        traj_linear = spline.get_trajectory(traj.t_vec, interp_type=TrajectoryInterpolationType.linear)

        fig = plt.figure(figsize=(20, 15), dpi=int(300))
        ax = fig.add_subplot(111, projection='3d')
        traj.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=False), num_markers=0,
                     ls=PlotLineStyle(linecolor='k', linestyle='-', linewidth=1, label='true'))
        traj_sub.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=False,  plot_type=TrajectoryPlotTypes.scatter_3D), num_markers=0,
                         ls=PlotLineStyle(linecolor='b', linestyle='-', linewidth=1, label='noisy ctrl pts', markersize=3, markerfacecolor='orange'))
        traj_cubic.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=False), num_markers=0,
                           ls=PlotLineStyle(linecolor='g', linestyle='-', linewidth=1, label='cubic'))
        traj_linear.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=True), num_markers=0,
                            ls=PlotLineStyle(linecolor='r', linestyle='-', linewidth=1, label='linear'))


    def test_traj(self):
        traj_gt = Trajectory(fn=str(SAMPLE_DATA_DIR + '/ID1-pose-gt.csv'))
        traj_est = TrajectoryEstimated(fn=str(SAMPLE_DATA_DIR + '/ID1-pose-est-posorient-cov.csv'))
        traj_est.subsample(num_max_points=100)

        spline = BsplineSE3()
        spline.feed_trajectory(traj=traj_est, uniform_timestamps=True, min_dt=0.1)
        traj_cubic = spline.get_trajectory(traj_gt.t_vec, interp_type=TrajectoryInterpolationType.cubic)
        fig = plt.figure(figsize=(20, 15), dpi=int(300))
        ax = fig.add_subplot(111, projection='3d')

        traj_gt.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=False), num_markers=0,
                     ls=PlotLineStyle(linecolor='k', linestyle='-', linewidth=1, label='true'))


        traj_est.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=False,
                                                                  plot_type=TrajectoryPlotTypes.scatter_3D),
                         num_markers=0,
                         ls=PlotLineStyle(linecolor='b', linestyle='-', linewidth=1, label='noisy ctrl pts',
                                          markersize=3, markerfacecolor='orange'))

        traj_cubic.plot_3D(fig=fig, ax=ax, cfg=TrajectoryPlotConfig(close_figure=False, show=True), num_markers=0,
                           ls=PlotLineStyle(linecolor='g', linestyle='-', linewidth=1, label='cubic'))

if __name__ == "__main__":
    unittest.main()