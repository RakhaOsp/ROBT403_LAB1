
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <vector>
#include <string>


std::vector<int> getDigits(const std::string& id) {
    std::vector<int> digits;
    for (char c : id) {
        if (isdigit(c)) {
            digits.push_back(c - '0');
        }
    }
    return digits;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_topic_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("Azamat", 10);
    std::string nu_id = "201843730";  // NU ID
    std::vector<int> id_digits = getDigits(nu_id);

    ros::Rate rate_1hz(1);  // 1 Hz rate
    ros::Rate rate_50hz(50);  // 50 Hz rate

    while (ros::ok()) {
        // Publish each digit at 1 Hz
        for (int digit : id_digits) {
            std_msgs::Int32 msg;
            msg.data = digit;

            ROS_INFO("Publishing at 1Hz: %d", msg.data);
            pub.publish(msg);

            ros::spinOnce();
            rate_1hz.sleep();
        }

        // Publish each digit at 50 Hz
        for (int digit : id_digits) {
            std_msgs::Int32 msg;
            msg.data = digit;

            ROS_INFO("Publishing at 50Hz: %d", msg.data);
            pub.publish(msg);

            ros::spinOnce();
            rate_50hz.sleep();
        }
    }

    return 0;
}
