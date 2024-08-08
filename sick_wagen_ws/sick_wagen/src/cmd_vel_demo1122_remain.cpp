#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <string.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseAction.h>

geometry_msgs::Twist cmd_vel;
geometry_msgs::Twist cmd_vel_move_base;
geometry_msgs::Twist cmd_vel_nav;

// 0 : normal, 1 : Pause, 2 : intersection 
int navigation_status = 0;
int signal_sign;
bool security_verification = true;
int queue[5] = {};
int queue_num;
double waypoints_x[3] ={1.0, 2.0, 5.0};
double waypoints_y[3] ={10.0, 22.0, 55.0};
ros::Publisher output_pub;


void check_point(double x, double y){
	int n = 3;
	double target_x = x;
	double target_y = y;
	bool exists_x = std::find(waypoints_x, waypoints_x+n, target_x) !=waypoints_x +n;
	bool exists_y = std::find(waypoints_y, waypoints_y+n, target_y) !=waypoints_y +n;
	if(exists_x && exists_y){
		ROS_INFO("-------------Pause intersection--------------");
		navigation_status = 1;
	}else{
		navigation_status = 0;
	}
}


//------------------velocity detail ------------------
void set_zero_velocity(){
	cmd_vel_nav.linear.x = 0;
	cmd_vel_nav.angular.z = 0;
}
void set_move_base_velocity(){
	cmd_vel_nav = cmd_vel_move_base;
}

void stop_intersection_velocity(){
	if(security_verification){
		ROS_INFO("GO %d", navigation_status);
		set_move_base_velocity();
	}else{
		ROS_INFO("STOP %d", navigation_status);
		set_zero_velocity();
	}	
}
//------------------CALLBACK FUNCTION------------------
void waypoint_callback(const move_base_msgs::MoveBaseActionGoal& waypoint){
	//move_base_msgs::MoveBaseGoal waypoint;
	// Trafic Signal Point
	double waypoint_x = waypoint.goal.target_pose.pose.position.x;
	double waypoint_y = waypoint.goal.target_pose.pose.position.y;
	check_waypoint(waypoint_x, waypoint_y);
	if(waypoint.goal.target_pose.pose.position.x == 1.0 &&
	   waypoint.goal.target_pose.pose.position.y ==  1.0){
		ROS_INFO("-------------Signal intersection--------------");
		ROS_INFO("Trafic Signal Point Now");
		navigation_status = 2;
	}else if(waypoint.goal.target_pose.pose.position.x == 198.656 &&
	   waypoint.goal.target_pose.pose.position.y == -17.907){
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else{
		ROS_INFO("-------------  Normal --------------");
		printf("Normal Mode");
		navigation_status = 0;
	}

}

void whill_callback(const sensor_msgs::Joy& whill_joy_msg)
{
	geometry_msgs::Twist cmd_vel_whill;
	cmd_vel_whill.linear.x = whill_joy_msg.axes[1];
	cmd_vel_whill.angular.z = whill_joy_msg.axes[0];

	switch(navigation_status){
		case 1:
			stop_intersection_velocity();
		default:
			set_move_base_velocity();
	}

	if(cmd_vel_whill.linear.x != 0 || cmd_vel_whill.angular.z != 0){
		cmd_vel = cmd_vel_whill;
	}else{
		cmd_vel = cmd_vel_nav;
	}
	ROS_INFO("navigation_status : %d", navigation_status);
	output_pub.publish(cmd_vel);
}

void move_base_callback(const geometry_msgs::Twist& cmd_vel_move_base_call)
{
	cmd_vel_move_base = cmd_vel_move_base_call;
}


void joy_callback(const sensor_msgs::Joy& joy_msg)
{
	if(joy_msg.buttons[4] == 1){
		printf("Button Push!");
		security_verification = true;
	}
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cmd_vel_tsukuba2022");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	output_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber move_base_cmd_vel_sub = nh.subscribe("/cmd_vel_move_base", 10, move_base_callback);
	ros::Subscriber waypoint_sub = nh.subscribe("/move_base/goal", 10, waypoint_callback);
	ros::Subscriber gamepad_joy_sub = nh.subscribe("/joy", 10, joy_callback);
	ros::Subscriber whill_joy_sub = nh.subscribe("/whill/states/joy", 10, whill_callback);

	ros::spin();
	return 0;
}


