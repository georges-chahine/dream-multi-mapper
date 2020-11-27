#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#pragma once
namespace mapper
{
class GPS {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GPS();
    virtual ~GPS();

    void createMap(std::string currentPath,Eigen::Matrix4d imu2base,Eigen::Matrix4d pc2base,Eigen::Matrix4d gps2base, IO::Datacontainer imuContainer, IO::pCloudcontainer pcContainer, IO::Datacontainer gpsUTMContainer, std::string filename, float leafSize);

    //bool compareFunction (std::string a, std::string b);

private:





};
}

