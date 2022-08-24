#include <ros/ros.h>
#include <ros_exercises/prime.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "prime_publisher");
    prime_application obj;
    return 0;

}
