#include <ros/ros.h>
#include <ros_exercises/compute_statistics.h>
#include <ros_exercises/print_array.h>
#include <algorithm>

void median(ros_exercises::compute_statistics::Request  &req, 
            ros_exercises::compute_statistics::Response &res) {

	size_t size = req.input.size();
	res.median = (size % 2) ? (req.input[size/2]) : 
                              ((req.input[size/2 - 1] + req.input[size/2])/2);

}

void mode(ros_exercises::compute_statistics::Request  &req, 
          ros_exercises::compute_statistics::Response &res) {

	size_t size = req.input.size();
	int mode_count = 1, current_count = 1;
	res.mode = req.input[0];
	for (int i = 1; i < size; i++) {
		if (req.input[i] == req.input[i-1]) current_count++;
		if (current_count > mode_count) {
			res.mode = req.input[i-1];
			mode_count = current_count;
		}
		if (req.input[i] != req.input[i-1]) current_count = 1;
	}

}

void mean(ros_exercises::compute_statistics::Request  &req, 
          ros_exercises::compute_statistics::Response &res) {

	size_t size = req.input.size();
	float sum = 0;
	for (int i = 0; i < size; i++) sum += req.input[i];
	res.mean = sum/size;

}

bool statistics_callback(ros_exercises::compute_statistics::Request  &req, 
                         ros_exercises::compute_statistics::Response &res) {
	
	if(req.input.empty()) return false;

	std::sort(req.input.begin(), req.input.end());	

	median(req, res);
	mode  (req, res);
	mean  (req, res);

	ROS_INFO("%s", (print_array::to_string(req)).c_str());
	ROS_INFO("response: median = %f, mode = %f, mean = %f", res.median, res.mode, res.mean);

	return true;

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "statistics_server");
	ros::NodeHandle node;

	ros::ServiceServer service = node.advertiseService("statistics", statistics_callback);
	ROS_INFO("Ready to receive array");
	ros::spin();

	return 0;

}

