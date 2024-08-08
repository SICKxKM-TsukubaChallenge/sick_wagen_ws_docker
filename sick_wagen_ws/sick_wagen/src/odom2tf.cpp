// enabled to set odomtopi parentframe childframe from param, tsukattene

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static std::string topic = "/odometry/gps";
static std::string parent_frame = "/odom_filterd";
static std::string child_frame = "/base_link";

void callback(const nav_msgs::OdometryConstPtr& odom){
    static tf::TransformBroadcaster br;
    tf::Transform tf;
    geometry_msgs::Pose odom_pose = odom->pose.pose;

    tf::poseMsgToTF(odom_pose,tf);

    tf::StampedTransform stamped_tf(tf,odom->header.stamp,parent_frame,child_frame);
    br.sendTransform(stamped_tf);
}

int main(int argc,char **argv){
    ros::init(argc,argv,"odom2tf");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    p_nh.getParam("odom_topic",topic);
    p_nh.getParam("parent_frame",parent_frame);
    p_nh.getParam("child_frame",child_frame);

    std::cout << "odom_topic:" << topic << std::endl;
    std::cout << "parent_frame:" << parent_frame << std::endl;
    std::cout << "child_frame" << child_frame << std::endl;

    ros::Subscriber sub_odom = nh.subscribe(topic,10,callback);
    ros::spin();
}