#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/*
This node is needed to create the trajectory starting from the localization
*/

class Trajectory{
    private:
        ros::Subscriber sub;
        ros::Publisher pub;
        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose;

        void tjCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){
            path.header.stamp = ros::Time::now();
            pose.header.stamp = ros::Time::now();
            pose.pose = msg->pose.pose;
            path.poses.push_back(pose);
            pub.publish(path);
        }
    public:
        Trajectory(ros::NodeHandle n){
            ROS_INFO("Init of the trajectory node constructor");
            sub = n.subscribe("/amcl_pose", 1000, &Trajectory::tjCallback, this);
            pub = n.advertise<nav_msgs::Path>("path",1000);
            path.header.frame_id = "map"; //fixed
            pose.header.frame_id = "map"; //fixed
        }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "trajectory");
	ROS_INFO("Trajectory node started.");
	ros::NodeHandle n;
	Trajectory trajectory(n);
	ros::spin();
	return 0;
}