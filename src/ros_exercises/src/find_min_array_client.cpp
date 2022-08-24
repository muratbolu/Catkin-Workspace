#include <ros/ros.h>
#include <ros_exercises/find_min_array.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "find_min_array_client");

	if (argc < 2) {
		ROS_INFO_STREAM("Usage: rosrun ros_exercises " << argv[0] << " <a list of numbers>");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<ros_exercises::find_min_array>("find_min");
	ros_exercises::find_min_array srv;

	for (int i = 1; i < argc; i++) srv.request.array.push_back(atof(argv[i]));

	if (client.call(srv)) {
		ROS_INFO("Minimum value: %f", (float)srv.response.min);
		return 0;
	}
	else {
		ROS_ERROR("Failed to call service find min");
		return 1;
	}

}

