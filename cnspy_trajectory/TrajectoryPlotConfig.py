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
import os
import matplotlib.pyplot as plt
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes


# https://matplotlib.org/2.1.2/api/_as_gen/matplotlib.pyplot.plot.html
class TrajectoryPlotConfig():
    white_list = []
    num_points = 0
    plot_type = TrajectoryPlotTypes.plot_3D
    dpi = 200
    title = ""
    scale = 1.0
    save_fn = ""
    result_dir = "."
    show = True
    close_figure = False
    radians = True
    view_angle = (45, 45)  # viewing angle of 3D plots tuple (azimuth, elevation)

    def __init__(self, white_list=[], num_points=0,
                 plot_type=TrajectoryPlotTypes.plot_3D, dpi=200, title="",
                 scale=1.0, save_fn="", result_dir=".", show=True, close_figure=False, radians=True,
                 view_angle=(45, 45)):
        self.white_list = white_list
        self.num_points = num_points
        self.plot_type = plot_type
        self.dpi = dpi
        self.title = title
        self.scale = scale
        self.save_fn = save_fn
        self.result_dir = result_dir
        self.show = show
        self.close_figure = close_figure
        self.radians = radians
        self.view_angle = view_angle

    @staticmethod
    def set_view_angle(cfg, ax):
        """
        set the view angle of a 3D plot

        Input:
        cfg -- TrajectoryPlotConfig
        ax  -- axes.Axes
        """
        assert (isinstance(cfg, TrajectoryPlotConfig))
        ax.view_init(azim=cfg.view_angle[0], elev=cfg.view_angle[1])

    @staticmethod
    def show_save_figure(cfg, fig):
        assert (isinstance(cfg, TrajectoryPlotConfig))
        assert(isinstance(fig, plt.Figure))
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
