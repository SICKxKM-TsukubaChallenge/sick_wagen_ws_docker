#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
ros::Publisher pub;
ros::Subscriber sub;
nav_msgs::Odometry msg;

void odomCallback(nav_msgs::Odometry msg)
{
    msg.pose.covariance[0] = 0.01;
    msg.pose.covariance[7] = 0.01;
    msg.pose.covariance[14] = 99999;
    msg.pose.covariance[21] = 99999;
    msg.pose.covariance[28] = 99999;
    msg.pose.covariance[35] = 0.03;
    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navsat");
    ros::NodeHandle nh;
    pub = nh.advertise<nav_msgs::Odometry>("whill/odom/modified", 1);
    sub = nh.subscribe<nav_msgs::Odometry>("whill/odom", 1, odomCallback);
    ros::spin();    
    return 0;
}