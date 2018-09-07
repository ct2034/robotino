/*
 * RobotinoNode.h
 *
 */

#ifndef RobotinoCameraNode_H
#define RobotinoCameraNode_H

#include "ComROS.h"
#include "CameraROS.h"

#include <ros/ros.h>

class RobotinoCameraNode
{
public:
	RobotinoCameraNode();
	~RobotinoCameraNode();

	bool spin();

private:
	ros::NodeHandle nh_;

	std::string hostname_;
	int cameraNumber_;

	ComROS com_;
	CameraROS camera_;

	void initModules();
};

#endif /* RobotinoCameraNode_H_ */
