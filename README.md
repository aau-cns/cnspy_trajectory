# Trajectory

A trajectory object consists of a array/vector of positions, orientations (Quaternions) and associated timestamps.
The object can be created by passing the these three vectors or by passing a `Pandas` `DataFrame` object containing them. 
The `DataFrame` will be converted using the class `TUMCSV2DataFrame`.

A trajectory object offers some conversions, load/store methods, plot, spatial transformations, etc.  

A `TrajectoryEstimated` is a specialization of a `Trajectory`, holding a covariances of the position and orientation to the timestamps, which can be loaded form a `DataFrame` too.

The `TrajectoryPlotter` offers methods to visualize a (or multiple) `Trajectory`.

![class_diagram](./doc/class_diagram.png "folder structure")

## Dependencies

* [numpy]()
* [enum]()
* [numpy_utils]()
* [trajectory]()
* [csv2dataframe]()
* [ros_csv_formats]()
* [matplotlib.pyplot]()


## Examples

Please refer to the unit-tests `Trajectory_Test` and `TrajectoryEstimated_Test` in `Trajectory/TrajectoryEstimated.py`.
For plotting options to the unit-test `TrajectoryPlotter_Test`.

### pose plot
![pose](./doc/pose.png "folder structure")

### pose plot 3D
![plot3d](./doc/plot3d.png "folder structure")

### mutli pose plot 3D
![multi](./doc/multi.png "folder structure")


## License

Software License Agreement (GNU GPLv3  License), refer to the LICENSE file.

*Sharing is caring!* - [Roland Jung](https://github.com/jungr-ait)  
