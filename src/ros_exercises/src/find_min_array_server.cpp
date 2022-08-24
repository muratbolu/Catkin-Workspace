#include <ros/ros.h>
#include <ros_exercises/find_min_array.h>
#include <ros_exercises/print_array.h>
#include <algorithm>

bool find_min_array(ros_exercises::find_min_array::Request  &req, 
                    ros_exercises::find_min_array::Response &res) {

	size_t size = req.array.size();
	if (req.array.empty()) return false;
	else res.min = *std::min_element(req.array.begin(), req.array.end());

	ROS_INFO("%s", (print_array::to_string(req)).c_str());
	ROS_INFO("response: min = %f", res.min);

	return true;

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "find_min_array_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("find_min", find_min_array);
	ROS_INFO("Ready to receive array");
	ros::spin();

	return 0;

}
