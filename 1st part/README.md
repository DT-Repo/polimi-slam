# polimi-slam 1st project
The directory **1st part** contains:
- the subscriber/publisher node src/sub_car.ccp
-  the custom mesage msg/dtOdom.msg 
- dynamic_reconfigure file cfg/parameters.cfg.

To change source of Odometry you can execute :
`rosrun rqt_reconfigure rqt_reconfigure` <br>
and then select odomtype Differential_Drive (choice 0) or Ackermann(choice 1).
Instead to change init values of position you can select init_value_x and init_value_y in a range of (0,0) - (100,100);

The structure of tf tree is composed by (odom_DDK)-->(car) for Differential Drive Model
instead for Ackerman model (odom_ACK)-->(car). The Broadcaster is the same i.e. /sub_car.   (World-Fixed) ---> (Robot-Fixed)

The custom message used for the odometry is dtOdom, which is based on nav_msgs/Odometry as a matter of fact it's composed by a std_msgs/Header, string child_frame_id, geometry_msgs/PoseWithCovariance and geometry_msgs/TwistWithCovariance.

## Run the 1st project
- Init roscore and play the bag publishing speedL_stamped, speedR_stamped and steer_stamped.
- run the node sub_car.cpp : `rosrun progetto_ROS sub_car`.  
- Launch the dynamc reconfigure tool `rosrun rqt_reconfigure rqt_reconfigure`
to switch between Differential Drive and Ackerman Odometries or to reset initial position (init_value_x, init_value_y) to a (0,0)-(100,100).
(Only the odometry selected will publish values).
- To see published values select an Odometry and run `rostopic echo /OdometryDDK` or `rostopic echo /OdometryACK`.
- To see tf tree, instead, run `rosrun rqt_tf_tree rqt_tf_tree` while for the
vector transformations run `rviz` and add TF and select odom_DDK (or odom_ACK) as Fixed Frame.

