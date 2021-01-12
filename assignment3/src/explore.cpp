#include <include/explore.h>
#include "std_msgs/Int64.h"

#include <thread>

#include <action/IntAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<action::IntAction> Server;

void execute(const action::IntGoalConstPtr& goal, Server* as) 
{
  // Do lots of awesome groundbreaking robot stuff here



  
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore_server_node");
  ros::NodeHandle n;
  Server server(n, "explore", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}