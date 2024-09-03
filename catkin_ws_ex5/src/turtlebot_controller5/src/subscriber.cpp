#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/Kill.h"
#define _USE_MATH_DEFINES
#include <cmath>
double PI = 3.141592653589793;
ros::Publisher velocity_pub;
ros::ServiceClient kill_client;


// Function to spawn a new turtle
void spawnTurtle(ros::NodeHandle& nh) {
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.5;
    spawn_srv.request.y = 5.5;
    spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle1";
    ros::Duration(1.0).sleep();
    if (spawn_client.call(spawn_srv)) {
        ROS_INFO("Turtle spawned successfully");
    } else {
        ROS_ERROR("Failed to spawn turtle");
    }
}

// Function to teleport the turtle to a specific position
void teleportTurtle(ros::NodeHandle& nh, double x, double y, double theta) {
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute teleport_srv;
    teleport_srv.request.x = x;
    teleport_srv.request.y = y;
    teleport_srv.request.theta = theta;
    if (teleport_client.call(teleport_srv)) {
        ROS_INFO("Turtle teleported to position [%f, %f, %f]", x, y, theta);
    } else {
        ROS_ERROR("Failed to teleport turtle");
    }
}

// Function to move the turtle in a square pattern
void moveSquare() {
    geometry_msgs::Twist vel_msg;
    ros::Rate rate(100);

    for (int i = 0; i < 4; ++i) {
        // Turn 90 degrees
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = -M_PI/2; // -90 degrees in radians here I tried to use a const and a
        // library, but in both cases it does not work properly during turning.
        velocity_pub.publish(vel_msg);
        ros::Duration(1.0).sleep(); // Turn for 1 second

        // Move forward
        vel_msg.linear.x = 10.1;
        vel_msg.angular.z = 0.0;
        velocity_pub.publish(vel_msg);
        ros::Duration(2.0).sleep(); 
    }
}

// Function to move the turtle in a triangular pattern
void moveTriangle() {
    geometry_msgs::Twist vel_msg;
    ros::Rate rate(1);

    for (int i = 0; i < 3; ++i) {
        if (i != 2) {
            // Turn 90 degrees
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = -PI/2; // -90 degrees in radians here I tried to use a const and 
	    // library, but in both cases it does not work properly during turning.
            velocity_pub.publish(vel_msg);
            ros::Duration(1.0).sleep(); // Turn for 1 second
            
            // Move forward
            vel_msg.linear.x = 10.1;
            vel_msg.angular.z = 0.0;
            velocity_pub.publish(vel_msg);
            ros::Duration(2.0).sleep();
        } else {
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = -PI*3/4; // -135 degrees in radians
            velocity_pub.publish(vel_msg);
            ros::Duration(1.0).sleep(); // Turn for 1 second

            vel_msg.linear.x = 10.1 * 1.41; // Move forward in the diagonal direction
            vel_msg.angular.z = 0.0;
            velocity_pub.publish(vel_msg);
            ros::Duration(2.0).sleep(); 
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtlebot_controller");
    ros::NodeHandle nh;

    // Kill the default turtle
    kill_client = nh.serviceClient<turtlesim::Kill>("/kill");
    turtlesim::Kill kill_srv;
    kill_srv.request.name = "turtle1";
    if (kill_client.call(kill_srv)) {
        ROS_INFO("Turtle killed successfully");
    } else {
        ROS_ERROR("Failed to kill turtle");
    }

    // Spawn a new turtle
    spawnTurtle(nh);

    // Teleport the new turtle to the first corner
    teleportTurtle(nh, 0.5, 10.7, 0.0);

    // Initialize publisher for the turtle's velocity
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // Perform square and triangular movements
    moveSquare();
    moveTriangle();

    return 0;
}

