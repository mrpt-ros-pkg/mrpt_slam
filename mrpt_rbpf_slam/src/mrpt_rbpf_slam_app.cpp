#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "mrpt_rpbf_slam");
    ros::NodeHandle n;
    ros::Rate r(10);
    PFslamWrapper slam;
    slam.get_param();
    slam.init();
    int count=0;
    slam.rawlogPlay();//if exists rawlog file
  while (ros::ok()) {
    if (count++ > 30) {
           count = 0;
         slam.loop(); 
    }
         slam.publishTF();
        ros::spinOnce();
        r.sleep();
    }
  return 0;
}   

