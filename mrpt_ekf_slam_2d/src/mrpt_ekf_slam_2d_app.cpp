#include "mrpt_ekf_slam_2d/mrpt_ekf_slam_2d_wrapper.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
	// Initialize ROS2
	rclcpp::init(argc, argv);

	// Create SLAM node with default options
	auto slam_node = std::make_shared<mrpt_ekf_slam_2d::EKFslamWrapper>();

	// Initialize parameters and SLAM
	slam_node->get_param();
	if (!slam_node->init())
	{
		RCLCPP_ERROR(slam_node->get_logger(), "Failed to initialize SLAM");
		rclcpp::shutdown();
		return EXIT_FAILURE;
	}

	// If rawlog playback mode, play and exit
	if (slam_node->rawlogPlay())
	{
		rclcpp::shutdown();
		return EXIT_SUCCESS;
	}

	// Main loop with executor
	rclcpp::executors::SingleThreadedExecutor executor;
	executor.add_node(slam_node);

	// Spin - callbacks will be invoked by subscribers
	executor.spin();

	rclcpp::shutdown();
	return EXIT_SUCCESS;
}
