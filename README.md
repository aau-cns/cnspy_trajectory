# The cnspy_trajectory Package

A [Trajectory](./cnspy_trajectory/Trajectory.py) object consists of a matrix of positions, orientations (Quaternions) and associated timestamps.
The object can be created by passing the these three matrices or by passing a `Pandas` `DataFrame` object containing them. 
The `DataFrame` will be converted using the class `TUMCSV2DataFrame`.

A trajectory object offers some conversions, load/store methods, plot, spatial transformations, etc.  

A `TrajectoryEstimated` is a specialization of a `Trajectory`, holding a covariances of the position and orientation to the timestamps, which can be loaded form a `DataFrame` too.

The `TrajectoryPlotter` offers methods to visualize a (or multiple) `Trajectory` objects.

![class_diagram](./doc/class_diagram.png "folder structure")


## Installation

Install the current code base from GitHub and pip install a link to that cloned copy
```
git clone https://gitlab.aau.at/aau-cns/py3_pkgs/cnspy_trajectory.git
cd cnspy_trajectory
pip install -e .
```


## Dependencies

* [numpy]()
* [matplotlib]()
* [pandas]()
* [scipy]()
* [cnspy_numpy_utils]()
* [cnspy_trajectory]()
* [cnspy_csv2dataframe]()
* [cnspy_spatial_csv_formats]()
* [matplotlib.pyplot]()


## Examples

Please refer to the unit-tests `Trajectory_Test` and `TrajectoryEstimated_Test` in `Trajectory/TrajectoryEstimated.py`.
For plotting options to the unit-test `TrajectoryPlotter_Test`.

### Pose plot
![pose](./doc/pose_plot.png "folder structure")

### Pose plot 3D
![plot3d](./doc/plot_3D.png "folder structure")

### Mutli pose plot 3D
![multi](./doc/multi.png "folder structure")


## License

Software License Agreement (GNU GPLv3  License), refer to the LICENSE file.

*Sharing is caring!* - [Roland Jung](https://github.com/jungr-ait)  
