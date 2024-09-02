#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
{

	ROS_INFO("Turtle subscriber@[%f, %f, %f]",
	msg->x, msg->y, msg->theta);

	geometry_msgs::Twist my_vel;
	my_vel.linear.x = 1.0;  // Move forward with linear velocity
	my_vel.angular.z = 1.0; // Rotate with angular velocity
	
	ROS_INFO("Publishing velocity command");// to ensure	
//publish
	pub.publish(my_vel);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlebot_subscriber");
	ros::NodeHandle nh;

    // Define the publisher
	pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    // Define the subscriber
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1, turtleCallback);

	ros::spin();
	return 0;
}

