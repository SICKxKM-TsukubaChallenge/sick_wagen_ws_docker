#include <ros/ros.h>
#include <stdio.h>
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

ros::Publisher output_pub;


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
void trafic_signal_velocity(){
	int sum = 0;
	for (int i=0; i<5;i++){
		sum += queue[i];
	}
	double ave = sum / 5;
	if(ave == 0 && navigation_status == 2){
		ROS_INFO("STOP");
		set_zero_velocity();
	}else{
		ROS_INFO("GO");
		set_move_base_velocity();
	}
}
//------------------CALLBACK FUNCTION------------------
//void waypoint_callback(const move_base_msgs::MoveBaseActionGoal& waypoint){
void waypoint_callback(const move_base_msgs::MoveBaseActionGoal& waypoint){
	//move_base_msgs::MoveBaseGoal waypoint;
	// Trafic Signal Point
	if(waypoint.goal.target_pose.pose.position.x == 1.0 &&
	   waypoint.goal.target_pose.pose.position.y ==  1.0){
		ROS_INFO("-------------Signal intersection--------------");
		ROS_INFO("Trafic Signal Point Now");
		navigation_status = 2;
	}else if(waypoint.goal.target_pose.pose.position.x == 414.030 &&
	   waypoint.goal.target_pose.pose.position.y == -98.816){
		ROS_INFO("-------------Pause intersection--------------");
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else if(waypoint.goal.target_pose.pose.position.x == 377.382 &&
	   waypoint.goal.target_pose.pose.position.y == -104.080){
		ROS_INFO("-------------Pause intersection--------------");
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else if(waypoint.goal.target_pose.pose.position.x == 304.232 &&
	   waypoint.goal.target_pose.pose.position.y == -103.340){
		ROS_INFO("-------------Pause intersection--------------");
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else if(waypoint.goal.target_pose.pose.position.x == 209.488 &&
	   waypoint.goal.target_pose.pose.position.y == -24.172){
		ROS_INFO("-------------Pause intersection--------------");
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else if(waypoint.goal.target_pose.pose.position.x == 206.241 &&
	   waypoint.goal.target_pose.pose.position.y == -20.733){
		ROS_INFO("-------------Pause intersection--------------");
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else if(waypoint.goal.target_pose.pose.position.x == 65.039 &&
	   waypoint.goal.target_pose.pose.position.y == -14.944){
		ROS_INFO("-------------Pause intersection--------------");
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else if(waypoint.goal.target_pose.pose.position.x == 68.171 &&
	   waypoint.goal.target_pose.pose.position.y == -6.696){
		ROS_INFO("-------------Pause intersection--------------");
		ROS_INFO("Stop intersection Now");
		//Trafic Pause Point
		security_verification = false;
		navigation_status = 1;
	}else if(waypoint.goal.target_pose.pose.position.x == 198.656 &&
	   waypoint.goal.target_pose.pose.position.y == -17.907){
		ROS_INFO("-------------Pause intersection--------------");
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
		case 2:
			trafic_signal_velocity();
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

void trafic_signal_callback(const std_msgs::Int32& signal_sign_topic){
	// 0 means red signal, 1 means Blue signal
	//signal_sign = signal_sign_topic.data;
	queue_num = (queue_num+1)%5;
	queue[queue_num] = signal_sign_topic.data;	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cmd_vel_tsukuba2022");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	output_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber move_base_cmd_vel_sub = nh.subscribe("/cmd_vel_move_base", 10, move_base_callback);
	ros::Subscriber trafic_signal = nh.subscribe("/trafic_signal", 10, trafic_signal_callback);
	ros::Subscriber waypoint_sub = nh.subscribe("/move_base/goal", 10, waypoint_callback);
	ros::Subscriber gamepad_joy_sub = nh.subscribe("/joy", 10, joy_callback);
	ros::Subscriber whill_joy_sub = nh.subscribe("/whill/states/joy", 10, whill_callback);

	ros::spin();
	return 0;
}


