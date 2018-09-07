/*
 * main.cpp
 *
 */


#include "RobotinoLaserRangeFinderNode.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_laserrangefinder_node");
	RobotinoLaserRangeFinderNode rn;
	rn.spin();
	return 0;
}
