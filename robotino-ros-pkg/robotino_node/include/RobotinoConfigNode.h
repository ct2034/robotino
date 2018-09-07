#ifndef RobotinoConfigNode_H
#define RobotinoConfigNode_H

#include "ComROS.h"

#include <ros/ros.h>

class RobotinoConfigNode
{
public:
	RobotinoConfigNode();
	~RobotinoConfigNode();

	bool spin();

private:
	ros::NodeHandle nh_;

	std::string hostname_;
	int confignumber_;

	ComROS com_;

	void initModules();
};

#endif /* RobotinoConfigNode_H_ */
