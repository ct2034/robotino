/*
 * RobotinoConfigNode.cpp
 */

#include "RobotinoConfigNode.h"

RobotinoConfigNode::RobotinoConfigNode()
	: nh_("~")
{
	nh_.param<std::string>("hostname", hostname_, "172.26.1.1" );
	nh_.param<int>("confignumber", confignumber_, 0 );

	std::ostringstream os;

	initModules();
}

RobotinoConfigNode::~RobotinoConfigNode()
{
}

void RobotinoConfigNode::initModules()
{
	com_.setAddress( hostname_.c_str() );
	com_.connectToServer( false );
}

bool RobotinoConfigNode::spin()
{
	ros::Rate loop_rate( 30 );

	while(nh_.ok())
	{
		ros::Time curr_time = ros::Time::now();

		com_.processEvents();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return true;
}

