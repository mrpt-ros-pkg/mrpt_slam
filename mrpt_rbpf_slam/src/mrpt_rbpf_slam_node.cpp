#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mrpt_rpbf_slam");
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  // Setup ros loop frequency from params
  double frequency;
  nh_p.param("update_loop_frequency", frequency, 100.);
  ros::Rate rate(frequency);

  PFslamWrapper slam;
  // Read parameters and configure node
  slam.getParams(nh_p);
  // Setup callbacks
  slam.init(nh);

  ros::Duration(1).sleep();

  // If play from rawlog file options is specified
  // play and then terminate application
  if (slam.rawlogPlay())
  {
    return EXIT_SUCCESS;
  }

  // Otherwise work as a usual rosnode
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
