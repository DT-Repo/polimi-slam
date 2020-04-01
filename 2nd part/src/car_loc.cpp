


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

//Init Variables
 const double dist= 1.765; // dist in [m]


const int s_ratio = 18;

double speed=0.0;
double steer=0.0;



   
    //init displacement variables DDK
   
   double x=0.0;
   double y =0.0;
   double th=0.0;
   


   //init velocity variables
   double V = 0.0;
   double W = 0.0;
  



//CalBack Odom

void callback(const geometry_msgs::PointStamped::ConstPtr& msg1)
{
  ROS_INFO ("speed-steer: ( %f , %f ) ", msg1->point.y, msg1->point.x);
  speed=msg1->point.y; //coordinate y is for speed
  steer=msg1->point.x; //coordinate x is for steer
  
}
int main(int argc,char** argv){



 //Init Node
    ros::init(argc,argv,"car_loc");

    ros::NodeHandle n;
   
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("OdomProject", 60);
    ros::Rate loop_rate(10);


    //Subscriber for odometry
    ros::Subscriber sub_speed = n.subscribe("/speedsteer", 1000, callback);



  



//Init TF

     tf::TransformBroadcaster odom_broadcaster;
     tf::Transform transform;


//Create Odometry variable
nav_msgs::Odometry odom;



ros::Time current_time = ros::Time::now();
   ros::Time last_time = ros::Time::now();

while (ros::ok())
     {
	//check incoming messages
	ros::spinOnce(); 
	current_time = ros::Time::now();
	

	double dt = (current_time - last_time).toSec();

	//Init Kinematic Variables
	double V=speed/3.6;
	
	double steer_angle=(steer)/s_ratio;
	double W=V*tan(steer_angle)/dist;   //ackermann steering


	
	//compute variations by 2^Order Runge-Kutta DDK
	double dth=W*dt;
        double dx=V*dt*cos(th+(W*dt)/2);
	double dy=V*dt*sin(th+(W*dt)/2);


	//compute new coordinates DDK
	x=x+dx;
	y=y+dy;
	th=th+dth;

	geometry_msgs::Quaternion odom_quatDDK = tf::createQuaternionMsgFromYaw(th);


	//compute tf DDK
	transform.setOrigin( tf::Vector3(x, y, 0.0) );  
        tf::Quaternion q;
        q.setRPY(0, 0, th);
        transform.setRotation(q);



	odom.header.stamp = current_time;
        odom.header.frame_id = "odom_DDK";




	 //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quatDDK;

	
        //set the velocity
        odom.child_frame_id = "base_link_DDK";
        odom.twist.twist.linear.x = V;
    	odom.twist.twist.linear.y = 0.0;
    	odom.twist.twist.angular.z = W;




	odom_pub.publish(odom);
	odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_DDK", "base_link_DDK"));
	
	last_time=current_time;
	loop_rate.sleep();
}

	    ros::spin();
	return 0;
}
