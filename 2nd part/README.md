# polimi-slam 2nd project

Compute accurate and high frequency GPS position fusing:

![This is an image](https://github.com/DT-Repo/polimi-slam/blob/master/2nd%20part/img/Board.png)

- Odometry
(from
wheels
encoders and steering angle)
- IMU (Piksi Multi board)
- GPS (Piksi Multi board) <br/>

## Solution
The archive contains:
- src/car_loc.cpp node to compute odometry /OdomProject from the bag's values.
- launch file which initializes the Odom node, EKF node and navsat node.
EKF and navsat nodes are taken from `robot_localization_tutorial` package.

To compute the localization of the car
1.  play the bag by using `rosbag play --clock /path/to/bag/project.bag` .
2. start the launch file by running `roslaunch project-ros2 localization_car.launch`.

In the launch file, it's set up 'use_sim_time=yes' to use clock published by the bag, then it's launched odometry node and complementary_filter node. Complementary filter node subscribes to `/swiftnav/rear/imu` topic by means of the `<remap from="/imu/data_raw" to="/swiftnav/rear/imu"/>`.
<br />
The next step is to fuse computated odometry and imu data by using EKF node. To this purpose, it's added the odometry `/OdomProject` and `/imu/data` following EKF node's rule, in fact it's defined `<param name="odom0" value="/OdomProject"/>` and `<param name="imu0" value="/imu/data"/> `. <br /> <br />
Process_noise_covariance and initial_estimate_covariance matrices are taken from `robot_localization_tutorial` package.
The last step is to integrate GPS data from bag, `/swiftnav/rear/gps`, with odometry filtered from ekf node by means of **navsat_transform_node**.
It's remapped `<remap from="/gps/fix" to="/swiftnav/rear/gps"/>` to subscribe directly to gps topic of the bag. 
<br /><br />
An Important parameter is `<param name="use_odometry_yaw" value="true"/>`, to take heading from odometry filtered.
Other parameters are taken from `project-ros2/params/navsat_transform_param.yaml` as :
- *GPS output frequency* : 30Hz 
- *magnetic_declination_radians* : 0.04868099 calculated by using longitude and latitude of Milan and a calculator of magnetic_declination (*longitude* : 9.1859243 and *latitude* : 45.4654219, https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml).
## RQT_GRAPH
![This is an image](https://github.com/DT-Repo/polimi-slam/blob/master/2nd%20part/img/rqt_graph_crop.png)

## Mapviz Sensor Fused-Trajectory Visualization:
![This is an image](https://github.com/DT-Repo/polimi-slam/blob/master/2nd%20part/img/localization_mapviz.png)
