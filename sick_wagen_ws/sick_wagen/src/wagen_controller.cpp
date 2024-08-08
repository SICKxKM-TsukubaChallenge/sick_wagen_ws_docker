#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>

geometry_msgs::Twist cmd_vel;
geometry_msgs::Twist sub_cmd_vel;
geometry_msgs::Twist cmd_vel_nav;
geometry_msgs::Twist cmd_vel_joy;
geometry_msgs::Twist former_cmd_vel_joy;
geometry_msgs::Twist cmd_vel_whill;
sensor_msgs::Joy joy_msg;
double linearGain = 0.5;
double angularGain = 0.6;
double speedUpRate = 1.6;

void nav_callback(const geometry_msgs::Twist& cmd_vel_nav)
{
	sub_cmd_vel.linear.x = cmd_vel_nav.linear.x;
	sub_cmd_vel.angular.z = cmd_vel_nav.angular.z;
}

void joy_callback(const sensor_msgs::Joy& joy_msg)
{
	//double LStickX = joy_msg.axes[0];
	//double LStickY = joy_msg.axes[1];
	double RStickX = joy_msg.axes[2];
	double RStickY = joy_msg.axes[3];
	// double LeftRight = joy_msg.axes[4]; // right : -1.0 , left : 1.0
	// double UpDown = joy_msg.axes[5]; // up : 1.0, down : -1.0

	// uint8_t X_btn = joy_msg.buttons[0];
	// uint8_t A_btn = joy_msg.buttons[1];
	// uint8_t B_btn = joy_msg.buttons[2];
	// uint8_t Y_btn = joy_msg.buttons[3];
	uint8_t LB_btn = joy_msg.buttons[4];
	// uint8_t RB_btn = joy_msg.buttons[5];
	// uint8_t LT_btn = joy_msg.buttons[6];
	// uint8_t RT_btn = joy_msg.buttons[7];

	cmd_vel_joy.linear.x = pow(RStickY,2) * linearGain;
	if (RStickY < 0) cmd_vel_joy.linear.x *= -1.0;
	if (LB_btn == 1) {
		// Speed up mode
		cmd_vel_joy.linear.x *= speedUpRate;
	}
	cmd_vel_joy.angular.z = RStickX * angularGain;
}

void whill_callback(const sensor_msgs::Joy& whill_joy_msg)
{
	cmd_vel_whill.linear.x = whill_joy_msg.axes[1];
	cmd_vel_whill.angular.z = whill_joy_msg.axes[0];
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "gamepad_pub");
	ros::NodeHandle nh;

	// Get param
	// If it doesn't set value, set to default parameter
	if (nh.getParam("/wagen_controller/linearGain", linearGain)) {
		nh.setParam("/wagen_controller/linearGain", linearGain);
	}
	if (nh.getParam("/wagen_controller/angularGain", angularGain)) {
		nh.setParam("/wagen_controller/angularGain", angularGain);
	}
	if (nh.getParam("/wagen_controller/speedUpRate", speedUpRate)) {
		nh.setParam("/wagen_controller/speedUpRate", speedUpRate);
	}

	ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Subscriber move_base_cmd_vel_sub = nh.subscribe("/cmd_vel_move_base", 10, nav_callback);
	ros::Subscriber gamepad_joy_sub = nh.subscribe("/joy", 10, joy_callback);
	ros::Subscriber whill_joy_sub = nh.subscribe("/whill/states/joy", 10, whill_callback);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		if(cmd_vel_whill.linear.x != 0 || cmd_vel_whill.angular.z != 0){
			cmd_vel = cmd_vel_whill;
		}else
		{
			if(cmd_vel_joy.linear.x != 0 || cmd_vel_joy.angular.z != 0){
				cmd_vel = cmd_vel_joy;
			}else{
				cmd_vel = sub_cmd_vel;
			}
		}
		cmd_pub.publish(cmd_vel);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
