#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "mrpt_rpbf_slam");
    ros::NodeHandle n;
    ros::Rate r(100);
    PFslamWrapper slam;
    slam.get_param();
    slam.init();


    if(!slam.rawlogPlay()){//if not play from rawlog file

     while (ros::ok()) {
            slam.publishTF();
            ros::spinOnce();
            r.sleep();
        }
    }

}   

