#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot:
    def __init__(self):
        # Initialize the node
        rospy.init_node('turtlebot_controller_py', anonymous=True)

        # Publisher for the turtle's velocity
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Subscriber for the turtle's pose
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)  # 10 Hz

    def update_pose(self, data):
        """Callback function to update the current pose of the turtle."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Calculate the Euclidean distance to the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """Calculate linear velocity."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Calculate the steering angle to the goal."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """Calculate angular velocity."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move_to_position(self, x_goal, y_goal, tolerance=0.1):
        """Move the turtle to a specific position."""
        goal_pose = Pose()
        goal_pose.x = x_goal
        goal_pose.y = y_goal

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= tolerance:
            # Linear and angular velocities
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publish velocity command
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop the turtle
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def run(self):
        """Run the turtlebot through a sequence of positions."""
        # Sequence of target positions
        positions = [(0.0, 0.0), (10.7, 0.0), (10.7, 10.7), (0.0, 10.7)]

        for pos in positions:
            self.move_to_position(pos[0], pos[1])

        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        bot = TurtleBot()
        bot.run()
    except rospy.ROSInterruptException:
        pass

