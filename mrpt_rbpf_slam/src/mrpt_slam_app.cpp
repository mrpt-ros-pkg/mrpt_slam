#include "mrpt_slam/mrpt_slam_wrapper.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "mrpt_slam");
    ros::NodeHandle n;
    ros::Rate r(10);
    PFslamWrapper slam;
    slam.get_param();
    slam.init();
    slam.publishMap();
  while (ros::ok()) {

        
        ros::spinOnce();
        r.sleep();
    }
  return 0;
}

