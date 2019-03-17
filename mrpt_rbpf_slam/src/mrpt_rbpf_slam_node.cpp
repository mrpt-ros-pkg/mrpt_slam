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
  slam.getParams(nh_p);
  slam.init(nh);

  ros::Duration(1).sleep();

  if (!slam.rawlogPlay())
  {  // if not play from rawlog file

    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }
  }
}
