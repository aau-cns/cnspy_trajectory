from enum import Enum


class TrajectoryPlotTypes(Enum):
    scatter_3D = 'scatter_3D'
    plot_3D = 'plot_3D'
    plot_2D_over_t = 'plot_2D_over_t'
    plot_2D_over_dist = 'plot_2D_over_dist'

    def __str__(self):
        return self.value
