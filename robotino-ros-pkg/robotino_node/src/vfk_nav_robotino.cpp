#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("id", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(std_msgs::String data)
  {
   geometry_msgs::PoseStamped goal;
   goal.header.frame_id = "map";
   goal.header.stamp = ros::Time::now();
 	
	if(data.data == "Middle2")
	{
		goal.pose.position.x = -9.187704; 
		goal.pose.position.y = 3.86997842;
		goal.pose.orientation.z = -0.0349923222;
		goal.pose.orientation.w = 0.999387;
    	pub_.publish(goal);
   	ROS_INFO("Sending goal");
	} else if (data.data == "Gate")
	{
		goal.pose.position.x = -14.920475; 
		goal.pose.position.y = 6.157745;
		goal.pose.orientation.z = -0.731415722;
		goal.pose.orientation.w = 0.681931;
    	pub_.publish(goal);
   	ROS_INFO("Sending Outside goal");
	} else if (data.data == "Home")
	{
		goal.pose.position.x = 0.116755; 
		goal.pose.position.y = 0.044355;
		goal.pose.orientation.z = -0.12338;
		goal.pose.orientation.w = 0.9923;
    	pub_.publish(goal);
   	ROS_INFO("Sending Home goal");
	} else if(data.data == "Middle")
	{
		goal.pose.position.x = -7.38350582; 
		goal.pose.position.y = 4.717759609;
		goal.pose.orientation.z = 0.01228957;
		goal.pose.orientation.w = 0.99992448;
    	pub_.publish(goal);
   	ROS_INFO("Sending Middle goal");
	}
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
