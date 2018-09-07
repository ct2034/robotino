/*
 * main.cpp
 *
 */


#include "RobotinoConfigNode.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_config_node");
	RobotinoConfigNode rn;
	rn.spin();
	return 0;
}
