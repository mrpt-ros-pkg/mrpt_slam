/*
 * File: mrpt_icp_slam_2d.h
 * Author: Vladislav Tananaev
 *
 */


#ifndef MRPT_ICP_SLAM_2D_H
#define MRPT_ICP_SLAM_2D_H

#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;


class ICPslam{
public:
    ICPslam();
    ~ICPslam();
    void read_iniFile(std::string ini_filename);


    CMetricMapBuilderICP mapBuilder;
private:



};



#endif /* MRPT_ICP_SLAM_2D_H */
