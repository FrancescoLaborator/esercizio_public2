#include <ros/ros.h>

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "first_cpp_node");
	ros::NodeHandle nh;
  
	ROS_INFO("Nodo partito!");
	ros::Duration(1.0).sleep();
	ROS_INFO("Nodo terminato!");
}
