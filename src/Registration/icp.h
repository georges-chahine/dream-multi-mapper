#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#pragma once
#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/IO.h"



namespace mapper
{
class ICP {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ICP();
    virtual ~ICP();

    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    void createMap(std::string currentPath,Eigen::Matrix4d imu2base,Eigen::Matrix4d pc2base,Eigen::Matrix4d gps2base, IO::Datacontainer imuContainer, IO::pCloudcontainer pcContainer, IO::Datacontainer gpsUTMContainer, std::string filename, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string inputFilters2ConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic, IO::IcpLogger icpLog, bool semantics);

    //bool compareFunction (std::string a, std::string b);

private:





};
}

