#include <ros/ros.h>
#include "progetto_ROS/floatStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "progetto_ROS/dtOdom.h"
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <progetto_ROS/parametersConfig.h>

# define M_PI       3.14159265358979323846




//Init Variables
 const double dist= 1.3;
const double base = 17.65;

const int s_ratio = 18;
double speed_L=0.0;
double speed_R=0.0;
double steer=0.0;
int odom_type=0;

   
    //init displacement variables DDK
   double x_init;
   double y_init;
   double x=x_init;
   double y=y_init ;
   double th;

   //init displacement variables ACK
   double xA = x_init;
   double yA = y_init;
   double thA = th;


   //init velocity variables
   double V = 0.0;
   double W = 0.0;
   double wA = 0.0;




//CalBack Odom

void callback(const progetto_ROS::floatStamped::ConstPtr& msg1, const progetto_ROS::floatStamped::ConstPtr& msg2, const progetto_ROS::floatStamped::ConstPtr& msg3)
{
  ROS_INFO ("speedL-speedR-steer: ( %f , %f, %f ) ", msg1->data, msg2->data,msg3->data);
  speed_L=msg1->data;
  speed_R=msg2->data;
  steer=msg3->data;
  
}

//CallBack change params

void callbackPar(progetto_ROS::parametersConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %d", 
            config.init_value_x,
	    config.init_value_y,
            config.odomtype);
            
            ROS_INFO ("%d",level);
	    if(config.odomtype==0){odom_type=0;}
	    if(config.odomtype==1){odom_type=1;}
	    x=config.init_value_x;
	    y=config.init_value_y;
            xA=config.init_value_x;
	    yA=config.init_value_y;
}



int main(int argc,char** argv){

    

    //Init Node
    ros::init(argc,argv,"sub_car");

    ros::NodeHandle n;
   
    ros::Publisher odom_pub = n.advertise<progetto_ROS::dtOdom>("OdomProjectDDK", 60);

    ros::Publisher odom_pub2 = n.advertise<progetto_ROS::dtOdom>("OdomProjectACK", 60);

    ros::Rate loop_rate(10);


     
     //Init TF

     tf::TransformBroadcaster odom_broadcaster;
     tf::Transform transform;

     tf::TransformBroadcaster odom_broadcasterACK;
     tf::Transform transformA;
     



     //Syncronyze topics
     message_filters::Subscriber<progetto_ROS::floatStamped> sub1(n, "speedL_stamped", 100);
     message_filters::Subscriber<progetto_ROS::floatStamped> sub2(n, "speedR_stamped", 100);
     message_filters::Subscriber<progetto_ROS::floatStamped> sub3(n, "steer_stamped", 100);



     typedef message_filters::sync_policies::ApproximateTime<progetto_ROS::floatStamped, progetto_ROS::floatStamped,progetto_ROS::floatStamped>    MySyncPolicy;
  
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2,sub3);

     sync.registerCallback(boost::bind(&callback, _1, _2,_3));



   //Dynamic Reconfigure
    dynamic_reconfigure::Server<progetto_ROS::parametersConfig> server;
    dynamic_reconfigure::Server<progetto_ROS::parametersConfig>::CallbackType f;

   
   f = boost::bind(&callbackPar, _1, _2);
   server.setCallback(f);


   //dt computed by interval of time between two messages
   ros::Time current_time = ros::Time::now();
   ros::Time last_time = ros::Time::now();
	
	
   while (ros::ok())
     {
	//check incoming messages
	ros::spinOnce(); 
	current_time = ros::Time::now();
	

	double dt = (current_time - last_time).toSec();

	//Init KInematic Variables
	double V=(speed_L+speed_R)/2;
	double W=(speed_R-speed_L)/dist;
	double steer_angle=(steer)/s_ratio*(2*M_PI/360);

	//Ackerman angular speed
	double wA=V*tan(steer_angle)/base;
	
	//compute variations by 2^Order Runge-Kutta DDK
	double dth=W*dt;
        double dx=V*dt*cos(th+(W*dt)/2);
	double dy=V*dt*sin(th+(W*dt)/2);

	
	//compute variations by 2^Order Runge-Kutta ACK
	double dthA=wA*dt;
        double dxA=V*dt*cos(thA+(wA*dt)/2);
	double dyA=V*dt*sin(thA+(wA*dt)/2);

	//compute new coordinates DDK
	x=x+dx;
	y=y+dy;
	th=th+dth;

	geometry_msgs::Quaternion odom_quatDDK = tf::createQuaternionMsgFromYaw(th);

	
	//compute new coordinates ACK
	xA=xA+dxA;
	yA=yA+dyA;
	thA=thA+dthA;

	geometry_msgs::Quaternion odom_quatACK = tf::createQuaternionMsgFromYaw(thA);

	//compute tf DDK
	transform.setOrigin( tf::Vector3(x, y, 0.0) );  
        tf::Quaternion q;
        q.setRPY(0, 0, th);
        transform.setRotation(q);
       
		
	
	
	//compute Odometry DDK
	progetto_ROS::dtOdom ddkOdom;


	ddkOdom.header.stamp = current_time;
        ddkOdom.header.frame_id = "odom_DDK";

        //set the position
        ddkOdom.pose.pose.position.x = x;
        ddkOdom.pose.pose.position.y = y;
        ddkOdom.pose.pose.position.z = 0.0;
        ddkOdom.pose.pose.orientation = odom_quatDDK;

	
        //set the velocity
        ddkOdom.child_frame_id = "base_link_DDK";
        ddkOdom.twist.twist.linear.x = V;
    	ddkOdom.twist.twist.linear.y = 0.0;
    	ddkOdom.twist.twist.angular.z = W;
	

	//compute tf ACK
	transformA.setOrigin( tf::Vector3(xA, yA, 0.0) );  
        tf::Quaternion qA;
        qA.setRPY(0, 0, thA);
        transformA.setRotation(qA);
        
	
	
	//compute Odometry ACK

	progetto_ROS::dtOdom ackOdom;
	 

	//compute Odometry DDK

	ackOdom.header.stamp = current_time;
        ackOdom.header.frame_id = "odom_ACK";

        //set the position
        ackOdom.pose.pose.position.x = xA;
        ackOdom.pose.pose.position.y = yA;
        ackOdom.pose.pose.position.z = 0.0;
        ackOdom.pose.pose.orientation = odom_quatACK;
	
	 //set the velocity
        ackOdom.child_frame_id = "base_link_ACK";
        ackOdom.twist.twist.linear.x = V;
    	ackOdom.twist.twist.linear.y = 0.0;
    	ackOdom.twist.twist.angular.z = wA;


	
	if(odom_type==0) {
	odom_pub.publish(ddkOdom);
	odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_DDK", "car"));

	} else {
	odom_pub2.publish(ackOdom);
	odom_broadcasterACK.sendTransform(tf::StampedTransform(transformA, ros::Time::now(), "odom_ACK", "car"));
 	}

	last_time=current_time;
	loop_rate.sleep();
      }
    



    ros::spin();


    return 0;
}


