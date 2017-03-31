
#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d_wrapper.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrpt_ekf_slam_3d");
  ros::NodeHandle n;
  ros::Rate r(100);
  EKFslamWrapper slam;
  slam.get_param();
  slam.init();
  ros::Duration(3).sleep();

  if (!slam.rawlogPlay())
  {
    while (ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  }
}
