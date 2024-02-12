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


class PlotTrajectory:
    def __init__(self):
        pass

    @staticmethod
    def plot(fn, result_dir="", verbose=False, plot_3D=True, plot_pose=True, show_plots=True):
        if not os.path.isfile(fn):
            print("PlotTrajectory: could not find file: %s" % fn)
            return False

        ## create result dir:
        if result_dir == "":
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

            if verbose:
                traj.print_statistics()
            with open(folder + '/metrics.txt', 'w') as file:
                traj.print_statistics(file=file)

        else:
            print("* Trajectory was not loaded!")
            return False
        return True


if __name__ == "__main__":
    # test3: python3 ROSbag2CSV.py --bagfile ../test/example.bag --topics /CS_200_MAV1/estimated_poseWithCov  /pose_sensor/pose --verbose --filename mav_PoseWithCov.csv sensor_PoseWithCov.csv --format PoseWithCov
    # test4: python3 ROSbag2CSV.py --bagfile ./sample_data/empty_bag.bag --topics /uwb_trilateration/tagDistance_raw /pose_sensor/pose /fcu/current_pose --verbose  --filenames uwb /rasdf/body_pose imu_pose.csv
    # test5: python3 ROSbag2CSV.py --bagfile ./sample_data/dummy.bag --topics /pose_est /pose_gt --verbose  --filenames est gt --format TUM
    parser = argparse.ArgumentParser(
        description='Trajectory: extract and store given topics of a rosbag into a CSV file')
    parser.add_argument('--filename', help='csv filename', default="not specified")
    parser.add_argument('--result_dir', help='directory to store results]',
                        default='')
    parser.add_argument('--verbose', action='store_true', default=False)
    parser.add_argument('--plot_3D', action='store_true', default=False)
    parser.add_argument('--plot_pose', action='store_true', default=False)
    parser.add_argument('--show_plots', action='store_true', default=False)


    tp_start = time.time()
    args = parser.parse_args()
    if PlotTrajectory.plot(args.filename, result_dir=args.result_dir, verbose=args.verbose,
                           plot_3D=args.plot_3D, plot_pose=args.plot_pose, show_plots=args.show_plots):
        print(" ")
        print("finished after [%s sec]\n" % str(time.time() - tp_start))
    else:
        print("failed! after [%s sec]\n" % str(time.time() - tp_start))
