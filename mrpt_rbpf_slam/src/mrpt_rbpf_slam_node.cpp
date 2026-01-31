#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>

int main(int argc, char** argv)
{
	// Initialize ROS2
	rclcpp::init(argc, argv);

	// Create node
	auto node = std::make_shared<rclcpp::Node>("mrpt_rbpf_slam");

	// Get update frequency parameter
	node->declare_parameter("update_loop_frequency", 100.0);
	double frequency = node->get_parameter("update_loop_frequency").as_double();

	// Create SLAM wrapper
	auto slam = std::make_shared<mrpt_rbpf_slam::PFslamWrapper>();

	// Initialize
	if (!slam->getParams(node) || !slam->init(node))
	{
		RCLCPP_ERROR(node->get_logger(), "Failed to initialize SLAM");
		rclcpp::shutdown();
		return EXIT_FAILURE;
	}

	// Brief sleep for initialization
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// If rawlog playback mode, play and exit
	if (slam->rawlogPlay())
	{
		rclcpp::shutdown();
		return EXIT_SUCCESS;
	}

	// Main loop with executor
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(node);

	rclcpp::Rate rate(frequency);

	while (rclcpp::ok())
	{
		executor.spin_some();
		rate.sleep();
	}

	rclcpp::shutdown();
	return EXIT_SUCCESS;
}
