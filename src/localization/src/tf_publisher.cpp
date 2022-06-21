#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

/*
This node is the responsible of the publication of odometry as tf
*/

class tf_publisher{
    private:
        ros::Subscriber sub;
        tf2_ros::TransformBroadcaster broadcaster;

        void tfCallback(const nav_msgs::Odometry::ConstPtr &msg){
            geometry_msgs::TransformStamped msg_tf;
			msg_tf.header.stamp = ros::Time::now();
			msg_tf.header.frame_id = "odom";
			msg_tf.child_frame_id = "base_footprint";
			msg_tf.transform.translation.x = msg->pose.pose.position.x;
			msg_tf.transform.translation.y = msg->pose.pose.position.y;
			msg_tf.transform.translation.z = 0;
			msg_tf.transform.rotation = msg->pose.pose.orientation;
			broadcaster.sendTransform(msg_tf);
        }
    public:
        tf_publisher(ros::NodeHandle n){
            ROS_INFO("Init of the tf_publisher node constructor");
            sub = n.subscribe("/odom", 1000, &tf_publisher::tfCallback, this);
        }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "tf_publisher");
	ROS_INFO("Tf broadcaster node started.");
	ros::NodeHandle n;
	tf_publisher tf_publisher(n);
	ros::spin();
	return 0;
}