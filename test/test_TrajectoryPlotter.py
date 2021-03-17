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
import unittest
import math
from cnspy_trajectory.TrajectoryPlotter import *

SAMPLE_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sample_data')

class TrajectoryPlotter_Test(unittest.TestCase):
    def load_trajectory_from_CSV(self):
        traj = Trajectory()
        traj.load_from_CSV(filename=str(SAMPLE_DATA_DIR + '/ID1-pose-gt.csv')) 
        self.assertFalse(traj.is_empty())
        return traj

    def load_trajectory2_from_CSV(self):
        traj = TrajectoryEstimated()
        traj.load_from_CSV(filename=str(SAMPLE_DATA_DIR + '/ID1-pose-est-cov.csv'))
        self.assertFalse(traj.is_empty())
        return traj

    def test_plot_3D(self):
        traj = self.load_trajectory_from_CSV()

        plotter = TrajectoryPlotter(traj_obj=traj, config=TrajectoryPlotConfig(show=True, close_figure=False,
                                                                                       save_fn=str(SAMPLE_DATA_DIR + '/../../doc/plot_3D.png')))
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
                                                                plot_type=TrajectoryPlotTypes.plot_2D_over_t))
        print('done')

    def test_plot_multi(self):
        plotter1 = TrajectoryPlotter(traj_obj=self.load_trajectory_from_CSV(),
                                     config=TrajectoryPlotConfig(num_points=120000))
        plotter2 = TrajectoryPlotter(traj_obj=self.load_trajectory2_from_CSV())

        TrajectoryPlotter.multi_plot_3D([plotter1, plotter2], cfg=TrajectoryPlotConfig(show=True,
                                                                                       save_fn=str(SAMPLE_DATA_DIR + '/../../doc/multi.png'),
                                                                                       view_angle=(40, 20)),
                                        name_list=['gt', 'est'])

    def test_plot_estimated_traj(self):
        plotter = TrajectoryPlotter(traj_obj=self.load_trajectory2_from_CSV())
        plotter.plot_pose(angles=True, cfg=TrajectoryPlotConfig(radians=False,
                                                                plot_type=TrajectoryPlotTypes.plot_2D_over_t,
                                                                save_fn=str(SAMPLE_DATA_DIR + '/../../doc/pose_plot.png')))
        plotter.plot_3D()
        print('done')


if __name__ == "__main__":
    unittest.main()
