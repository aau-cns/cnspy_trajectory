#!/usr/bin/env python
# Software License Agreement (GNU GPLv3  License)
#
# Copyright (c) 2020, Roland Jung (roland.jung@aau.at) , AAU, KPK, NAV
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

import time
import os
import argparse
import yaml
import csv
from tqdm import tqdm

from cnspy_trajectory import TrajectoryPlotTypes
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
from cnspy_trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes
from cnspy_trajectory.TrajectoryPlotter import TrajectoryPlotter


class PlotTrajectory:
    def __init__(self):
        pass

    @staticmethod
    def plot_single(fn, result_dir=None, verbose=False, plot_3D=True, plot_pose=True, show_plots=True, save_metrics=True):
        if not os.path.isfile(fn):
            print("PlotTrajectory: could not find file: %s" % fn)
            return False

        ## create result dir:
        if result_dir is None:
            folder = str(fn).replace(".csv", "")
        else:
            folder = result_dir
        folder = os.path.abspath(folder)
        try:  # else already exists
            os.makedirs(folder)
        except:
            pass
        if verbose:
            print("* result_dir: \t " + str(folder))

        traj = Trajectory()
        traj.load_from_CSV(fn=fn)
        if not traj.is_empty():
            if plot_3D:
                cfg = TrajectoryPlotConfig(show=show_plots, close_figure=True,
                                           save_fn=str(folder + '/plot_3D.png'))
                traj.plot_3D(cfg=cfg)
            if plot_pose:
                cfg=TrajectoryPlotConfig(show=show_plots,
                                         close_figure=True,
                                         radians=True,
                                         plot_type=TrajectoryPlotTypes.plot_2D_over_t,
                                         save_fn=str(folder + '/pose_over_t.png'))
                traj.plot_pose(cfg=cfg)
                cfg.plot_type=TrajectoryPlotTypes.plot_2D_over_dist
                cfg.save_fn=str(folder + '/pose_over_dist.png')
                traj.plot_pose(cfg=cfg)

            if save_metrics:
                if verbose:
                    traj.print_statistics()
                with open(folder + '/metrics.txt', 'w') as file:
                    traj.print_statistics(file=file)

        else:
            print("* Trajectory was not loaded!")
            return False
        return True

    @staticmethod
    def plot_multi(fns, result_dir=None, verbose=False, show_plots=True):
        folder = result_dir
        traj_arr = []
        for fn in fns:
            if not os.path.isfile(fn):
                print("* WARNING: could not find file: %s" % fn)
                continue
            ## create result dir:
            if folder is None:
                folder = str(fn).replace(".csv", "")
                folder = os.path.abspath(folder)
                try:  # else already exists
                    [head, tail] = os.path.split(folder)
                    folder = head
                    os.makedirs(folder)
                except:
                    pass
                if verbose:
                    print("* result_dir: \t " + str(folder))

            traj = Trajectory()
            traj.load_from_CSV(fn=fn)
            if not traj.is_empty():
                traj_arr.append(traj)
            else:
                print("* WARNING trajectory is empty! fn=" + str(fn))

        if traj_arr:
            cfg = TrajectoryPlotConfig(show=show_plots, close_figure=True,
                                       save_fn=str(folder + '/multiplot_3D.png'))
            TrajectoryPlotter.multi_plot_3D(traj_arr, cfg=cfg)

        return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='PlotTrajectory: tool to save plots and metrics of a trajectory')
    parser.add_argument('--filename', help='csv filename', default=None)
    parser.add_argument('--filenames', type=str, nargs='+',
                        help='a list of files to be plotted in 3D', default=None)
    parser.add_argument('--result_dir', help='directory to store results]',
                        default=None)
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--plot_3D', action='store_true', default=False)
    parser.add_argument('--plot_pose', action='store_true', default=False)
    parser.add_argument('--show_plots', action='store_true', default=False)
    parser.add_argument('--save_metrics', action='store_true', default=False)


    tp_start = time.time()
    args = parser.parse_args()

    if args.filename is not None:
        PlotTrajectory.plot_single(args.filename, result_dir=args.result_dir, verbose=args.verbose,
                               plot_3D=args.plot_3D, plot_pose=args.plot_pose, show_plots=args.show_plots,
                               save_metrics=args.save_metrics)

    elif args.filenames is not None:
        PlotTrajectory.plot_multi(args.filenames, result_dir=args.result_dir, show_plots=args.show_plots, verbose=args.verbose)

    print("finished after [%s sec]\n" % str(time.time() - tp_start))
