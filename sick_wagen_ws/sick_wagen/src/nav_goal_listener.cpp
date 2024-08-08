#include <ros/ros.h>
#include <stdio.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <string>

void callback(const geometry_msgs::PoseStamped& goal)
{
//	printf("[(%f,%f,0.0),(0.0,0.0,%f,%f)]",goal.pose.position.x, goal.pose.position.y,goal.pose.orientation.z,goal.pose.orientation.w);
	ROS_INFO("[(%f,%f,0.0),(0.0,0.0,%f,%f)]",goal.pose.position.x, goal.pose.position.y,goal.pose.orientation.z,goal.pose.orientation.w);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "nav_goal_listener");
	ros::NodeHandle nh;
	ros::Subscriber nav_goal_listener = nh.subscribe("/move_base_simple/goal", 10, callback);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	
/*	writing_file.open(file_name, std::ios::app);
	std::string writing_text = nav_goal_pose;

	writing_file << writing_text << std::endl;
	writing_file.close();
*/

	return 0;
}
