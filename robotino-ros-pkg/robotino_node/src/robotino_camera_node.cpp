/*
 * main.cpp
 *
 */


#include "RobotinoCameraNode.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_camera_node");
	RobotinoCameraNode rn;
	rn.spin();
	return 0;
}
