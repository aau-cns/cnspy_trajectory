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
########################################################################################################################
import os

import pandas
from cnspy_csv2dataframe.TUMCSV2DataFrame import TUMCSV2DataFrame
from cnspy_csv2dataframe.CSV2DataFrame import CSV2DataFrame
from cnspy_spatial_csv_formats.CSVSpatialFormatType import CSVSpatialFormatType
from cnspy_spatial_csv_formats.EstimationErrorType import EstimationErrorType
from cnspy_trajectory.Trajectory import Trajectory
from cnspy_trajectory.TrajectoryErrorType import TrajectoryErrorType


class TrajectoryError(Trajectory):
    traj_err_type = None # TrajectoryErrorType

    def __init__(self, t_vec=None, p_vec=None, q_vec=None,
                 traj_err_type=TrajectoryErrorType(err_type=EstimationErrorType.type1)):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)
        self.traj_err_type = traj_err_type

    # TODO: plot leads to a circular dependency!
    # def plot(self, fig=None, cfg=TrajectoryPlotConfig()):
    #     plotter = TrajectoryPlotter(traj_obj=self, config=cfg)
    #
    #     if fig is None:
    #         fig = plt.figure(figsize=(20, 15), dpi=int(cfg.dpi))
    #     # create 2x2 grid
    #     ax1 = fig.add_subplot(211)
    #     ax2 = fig.add_subplot(212)
    #     plotter.ax_plot_pos(ax=ax1, cfg=cfg)
    #     plotter.ax_plot_angle(ax=ax2, cfg=cfg)
    #
    #     p_err_text, R_err_text = self.traj_err_type.error_def()
    #     # Error type text:
    #     text_p_err = 'local '
    #     if not self.traj_err_type.is_local_position_error():
    #         text_p_err = 'global '
    #     # Error type text:
    #     text_R_err = 'local '
    #     if not self.traj_err_type.is_local_position_error():
    #         text_R_err = 'global '
    #
    #     ax1.set_ylabel(text_p_err + 'position err [m] ' + p_err_text)
    #     if cfg.radians:
    #         ax2.set_ylabel(text_R_err + 'rotation err [rad] ' + R_err_text)
    #     else:
    #         ax2.set_ylabel(text_R_err + 'rotation err [deg] ' + R_err_text)

    def load_from_DataFrame(self, df):
        print('currently not supported! We need a CSV Format with EstErrorType first')
        assert False
        #self.t_vec, self.p_vec, self.q_vec = TUMCSV2DataFrame.DataFrame_to_TPQ(data_frame=df)

    def to_DataFrame(self):
        print('currently not supported! We need a CSV Format with EstErrorType first')
        assert False
        #return TUMCSV2DataFrame.TPQ_to_DataFrame(self.t_vec, self.p_vec, self.q_vec)

    def load_from_CSV(self, filename):
        if not os.path.isfile(filename):
            print("Trajectory: could not find file %s" % os.path.abspath(filename))
            return False

        loader = CSV2DataFrame(filename=filename)
        self.load_from_DataFrame(loader.data_frame)
        return loader.data_loaded

    def save_to_CSV(self, filename):
        if self.is_empty():
            return False
        df = self.to_DataFrame()
        TUMCSV2DataFrame.save_CSV(df, filename=filename, fmt=CSVSpatialFormatType.TUM)
        return True