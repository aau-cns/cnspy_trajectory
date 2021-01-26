# import os
# import numpy as np
# import pandas as pandas
# from matplotlib import pyplot as plt
#
# from trajectory.PlotLineStyle import PlotLineStyle
# from trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes
# from trajectory.TrajectoryPlotter import TrajectoryPlotter
# from trajectory.TrajectoryPlotConfig import TrajectoryPlotConfig
# from trajectory.pyplot_utils import set_axes_equal
# from spatial_csv_formats.CSVFormatPose import CSVFormatPose
# from numpy_utils.accumulated_distance import *
#
#
# class PosePlotter:
#     data_frame = None
#     fmt = None
#
#     def __init__(self, df):
#         assert (isinstance(df, pandas.DataFrame))
#         self.data_frame = df
#
#         if {CSVFormatPose.get_format(CSVFormatPose.TUM)}.issubset(df.columns):
#             self.fmt = CSVFormat.TUM
#         elif {CSVFormatPose.get_format(CSVFormatPose.TUM_short)}.issubset(df.columns):
#             self.fmt = CSVFormat.TUM_short
#         else:
#             print('columns in data frame not supported {0}'.format(str(df.coumns)))
#
#     @staticmethod
#     def ax_plot_n_dim(ax, x_linespace, values,
#                       colors=['r', 'g', 'b'],
#                       labels=['x', 'y', 'z'], ls=PlotLineStyle()):
#         assert len(colors) == len(labels)
#         if len(colors) > 1:
#             assert len(colors) == values.shape[1]
#             for i in range(len(colors)):
#                 ax.plot(x_linespace, values[:, i],
#                         color=colors[i], label=labels[i], linestyle=ls.linestyle, linewidth=ls.linewidth,
#                         marker=ls.marker)
#         else:
#             ax.plot(x_linespace, values, color=colors[0], label=labels[0], linestyle=ls.linestyle,
#                     linewidth=ls.linewidth)
#
#     @staticmethod
#     def get_pos_data(data_frame, white_list=[], scale=1.0, get_distance=False):
#         ts = data_frame.as_matrix(['t'])
#
#         if not any([flag == 'x' for flag in white_list]):
#             xs = data_frame.as_matrix(['tx'])
#         else:
#             xs = np.zeros(ts.shape)
#         if not any([flag == 'y' for flag in white_list]):
#             ys = data_frame.as_matrix(['ty'])
#         else:
#             ys = np.zeros(ts.shape)
#         if not any([flag == 'z' for flag in white_list]):
#             zs = data_frame.as_matrix(['tz'])
#         else:
#             zs = np.zeros(ts.shape)
#
#         if scale and scale != 1.0:
#             scale = float(scale)
#             xs *= scale
#             ys *= scale
#             zs *= scale
#
#         if get_distance:
#             dist_vec = accumulated_distance([xs, ys, zs])
#         else:
#             dist_vec = np.zeros(ts.shape)
#
#         return ts, xs, ys, zs, dist_vec
#
#
#
#     @staticmethod
#     def get_rpy_data(data_frame, white_list=[], radians=False):
#         ts = data_frame.as_matrix(['t'])
#
#         if not any([flag == 'roll' for flag in white_list]):
#             xs = data_frame.as_matrix(['tx'])
#         else:
#             xs = np.zeros(ts.shape)
#         if not any([flag == 'pitch' for flag in white_list]):
#             ys = data_frame.as_matrix(['ty'])
#         else:
#             ys = np.zeros(ts.shape)
#         if not any([flag == 'yaw' for flag in white_list]):
#             zs = data_frame.as_matrix(['tz'])
#         else:
#             zs = np.zeros(ts.shape)
