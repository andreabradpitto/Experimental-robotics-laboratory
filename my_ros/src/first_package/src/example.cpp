#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h" //this is for the service
#include "custom_messages/Two.h"//notice it's Two.h not .msg, it's an automatically generated file
#include "custom_messages/Sum.h"

ros::Publisher pub;
ros::Publisher pub2;

void turtleCallback (const turtlesim::Pose::ConstPtr& msg){
	ROS_INFO("Subscriber, %f, %f, %f", msg->x, msg->y, msg->theta);
	
	//geometry_msgs::Twist vel;
	//vel.angular.z = 0.1;
	//pub.publish(vel); //queste 3 righe sono state cancellate alla fine dal prof, per poterle mettere in addCallback

	custom_messages::Two two;
	two.a = msg->x;
	two.b = msg->y;
	pub2.publish(two);
}


bool addCallback (custom_messages::Sum::Request& req, custom_messages::Sum::Response& res){

	res.sum = req.a + req.b;
	geometry_msgs::Twist vel;
	vel.linear.x = req.a;
	vel.linear.y = req.b;
	pub.publish(vel);
	return true;
}


int main (int argc, char **argv){
	ros::init(argc,argv,"example");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv;
	srv.request.x = 2.0;
	srv.request.y = 2.0;
	srv.request.name = "my_turtle_from_code";
	client.call(srv);
	
	pub = nh.advertise<geometry_msgs::Twist>("/my_turtle/cmd_vel", 1); 	  //my_turtle al posto di turtle1 qui
									   	  //e riga sotto per vedere lo stato
									 	  //della mia tartaruga invece di quella "di base"

	ros::Subscriber sub = nh.subscribe("/my_turtle/pose", 1, turtleCallback); //name of the topic,
										  //size of the buffer, name of the callback

	pub2 = nh.advertise<custom_messages::Two>("/custom_msgs/Two", 1); //personal msg

	ros::ServiceServer service = nh.advertiseService("/sum", addCallback); //personal srv

	ros::spin();
	return 0;
}
