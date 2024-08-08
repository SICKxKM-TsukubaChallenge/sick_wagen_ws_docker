// enabled to set odomtopi parentframe childframe from param, tsukattene

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

static std::string gps_topic = "/odometry/filtered";
static std::string whill_topic = "/whill/odom";
static std::string pub_topic = "/odometry/selected";

nav_msgs::Odometry current_gps_odom, current_whill_odom;
static int cov_MAX_ = 8;
static int exaggeration_rate_ = 10;

void gps_callback(const nav_msgs::OdometryConstPtr& odom) {current_gps_odom = *odom;}
void whill_callback(const  nav_msgs::OdometryConstPtr& odom) {current_whill_odom = *odom;}

// if covariance is not zero, makes it big
nav_msgs::Odometry gps_covariance_exaggerater(const nav_msgs::OdometryConstPtr& odom){
    nav_msgs::Odometry exaggerated_odom_msg;
    exaggerated_odom_msg = *odom;

    for (size_t i = 0; i < odom->pose.covariance.size(); i++)
    {
        if (odom->pose.covariance[i]!=0)
        {
            exaggerated_odom_msg.pose.covariance[i] = odom->pose.covariance[i]*exaggeration_rate_;
        }
    }
    return exaggerated_odom_msg;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"odom_mng");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    p_nh.getParam("cov_MAX",cov_MAX_);
    p_nh.getParam("exaggeration_rate",exaggeration_rate_);

    ros::Subscriber sub_gps_odom = nh.subscribe(gps_topic,10,gps_callback);
    // ros::Subscriber sub_whill_odom = nh.subscribe(whill_topic,10,whill_callback);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry> (pub_topic,100);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        nav_msgs::Odometry msg;
        // if(current_gps_odom.pose.covariance[0]<cov_MAX_){
        //     msg = current_gps_odom;
        // }
        // else{
        //     msg = current_whill_odom;
        // }

        // pub_odom.publish(msg);
        msg = gps_covariance_exaggerater(&current_gps_odom);

        pub_odom.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}