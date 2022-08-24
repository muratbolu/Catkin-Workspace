#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "twist_publisher");
    ros::NodeHandle node;
    ros::Publisher twist_publisher = node.advertise<geometry_msgs::Twist>("/wheelbarrow/vel_cmd", 1000);
    ros::Rate loop_rate(100);

    double frequency = 0.1;
    double omega = 2 * M_PI * frequency;
    double radius = 10;

    for (int count = 0; ros::ok(); count++) {
        double time = count/100.0;
        geometry_msgs::Twist twist;
        twist.linear.x  = -1 * omega * radius * cos(time * omega);
        twist.linear.y  = -1 * omega * radius * sin(time * omega);
        twist.linear.z  = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 4 * omega;
        ROS_INFO("Twist: linear: %f, %f, %f, angular: %f, %f, %f",
                 twist.linear.x,  twist.linear.y,  twist.linear.z,
                 twist.angular.x, twist.angular.y, twist.angular.z);
        twist_publisher.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
