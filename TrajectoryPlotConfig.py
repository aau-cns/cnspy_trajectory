from trajectory.TrajectoryPlotTypes import TrajectoryPlotTypes


# https://matplotlib.org/2.1.2/api/_as_gen/matplotlib.pyplot.plot.html
class TrajectoryPlotConfig():
    white_list = []
    num_points = []
    plot_type = TrajectoryPlotTypes.plot_3D
    dpi = 200
    title = ""
    scale = 1.0
    save_fn = ""
    result_dir = "."
    show = True
    close_figure = False
    radians = True
    view_angle = (45, 45)  # viewing angle of 3D plots tuple(elevation, azimuth)

    def __init__(self, white_list=[], num_points=[],
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
