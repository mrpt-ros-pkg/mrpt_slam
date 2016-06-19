/*
 *  File: mrpt_slam.h
 *  Author: Vladislav Tananaev
 *
 *
 */
#ifndef MPRT_SLAM_H
#define MRPT_SLAM_H


#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace mrpt::poses;


class PFslam{
public:
     PFslam();
    ~PFslam();

    //reads parameters from ini file
    void read_iniFile(std::string ini_filename);

    void read_rawlog(std::vector<std::pair<CActionCollection,CSensoryFrame>>& data,std::string rawlog_filename);
    //COccupancyGridMap2D run_slam(CActionCollection action,CSensoryFrame observations);
     void run_slam(CActionCollection action,CSensoryFrame observations, CPose3DPDFParticles&	curPDF,COccupancyGridMap2D& map);


protected:
    std::string ini_filename_;
    CMetricMapBuilderRBPF mapBuilder;

    /*****************************************************
    			Config params
    *****************************************************/
    std::string  RAWLOG_FILE;
    std::string         METRIC_MAP_CONTINUATION_GRIDMAP_FILE; // .gridmap file
    mrpt::math::TPose2D METRIC_MAP_CONTINUATION_START_POSE;
    // ---------------------------------
    //		MapPDF opts
    // ---------------------------------
    CMetricMapBuilderRBPF::TConstructionOptions		rbpfMappingOptions;


};

#endif /*MRPT_SLAM_H*/
