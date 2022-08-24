#include <ros/ros.h>
#include <ros_exercises/check_palindrome.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "palindrome_client");
	if (argc != 2) {
		ROS_INFO("enter a single string");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<ros_exercises::check_palindrome>("palindrome");
	ros_exercises::check_palindrome srv;
	srv.request.input = argv[1];
	if (client.call(srv)) {
		ROS_INFO("Answer: %d", (int)srv.response.bit);
	}
	else {
		ROS_ERROR("Failed to call service palindrome");
		return 1;
	}
	return 0;
}
