#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d_wrapper.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mrpt_icp_slam_2d");
	ros::NodeHandle n;
	ros::Rate r(100);
	ICPslamWrapper slam;
	slam.get_param();
	slam.init();

	ROS_INFO_STREAM("About to enter the main spin loop.");

	// if (!slam.rawlogPlay())
	{  // if not play from rawlog file
		ros::spin();
	}
}
