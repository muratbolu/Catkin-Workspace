#include <ros/ros.h>
#include <ros_exercises/check_palindrome.h>

bool check_palindrome(ros_exercises::check_palindrome::Request  &req,
		      ros_exercises::check_palindrome::Response &res) {
	int length = req.input.length();
	res.bit = true;
	for (int i = 0; i < (length/2 + 1); i++) {
		if (req.input[i] != req.input[length-i-1]) {
			res.bit = false;
			break;
		}
	}
	ROS_INFO("request: input = %s", req.input.c_str());
	ROS_INFO("response: bit = %d", (int)res.bit);
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "palindrome_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("palindrome", check_palindrome);
	ROS_INFO("Ready to check string");
	ros::spin();

	return 0;
}
