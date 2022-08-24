#include <ros/ros.h>
#include <ros_exercises/compute_statistics.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "statistics_client");

    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<ros_exercises::compute_statistics>("statistics");
    ros_exercises::compute_statistics srv;

    std::vector<float> input;
    if(node.getParam("lst", input)) {
        for (int i = 0; i < input.size(); i++) srv.request.input.push_back(input[i]);
    }
    else {
        ROS_ERROR("Failed to receive input");
	return 1;
    }

    if (client.call(srv)) {
		ROS_INFO("Median: %f, mode: %f, mean: %f", 
                          srv.response.median, srv.response.mode, srv.response.mean);
		return 0;
	}
	else {
		ROS_ERROR("Failed to call service statistics");
		return 1;
	}

}
