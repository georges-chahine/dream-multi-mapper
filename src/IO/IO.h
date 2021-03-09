
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/filter.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "vector"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <geodesy/utm.h>

#define foreach BOOST_FOREACH
#pragma once
namespace mapper
{
class IO {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IO();
    virtual ~IO();

    struct Datacontainer {
        std::vector<std::vector<double>> vect;
        std::vector<std::string> header;
        std::string frame_id;
    } ;
    struct TFcontainer {
        std::vector<std::string> frame_id;
        std::vector<std::string> child_frame_id;
        std::vector<std::string> bothFrames;
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transform; // 4x4 affine transform
      //  Eigen::Matrix3d rotation;  // qx qy qz qw

    } ;

    struct pCloudcontainer {
        std::vector<pcl::PointCloud<pcl::PointXYZRGBL>> XYZRGBL;
        std::vector<pcl::PointCloud<pcl::Normal>> normals;
        pcl::PointCloud<pcl::Intensity> Intensity;
        std::vector<double> timestamp;
        std::string frame_id;
    } ;

    void matchDataToPointCloudRate(std::vector<double>& timestamp , Datacontainer& input, Datacontainer& output);
   // double interpolate( std::vector<double> &xData, std::vector<double> &yData, double x, bool extrapolate );
    typedef std::vector<std::string> stringvec;
    void interpolateRefRange(Datacontainer& ref, Datacontainer& input, Datacontainer& output);
  //  void interpolateRefRange(Datacontainer& ref, pCloudcontainer& input, pCloudcontainer& output);
    std::vector<double> getTimeLimits(std::vector<std::vector<double>>& timeRange);
    std::uint32_t checkPointLabel(int& r, int& g, int& b);
    Eigen::Matrix4d parseTFcontainer(TFcontainer& container, std::string& sensor_frame, std::string& base_link);
    void readBags(std::string sourceBags, std::string currentPath, std::vector<std::string> topics, bool autoGenerateMaps, float autoDist, double lat, double lon, double radius, std::string base_link, std::string rMethod , int mapNumber, bool semantics, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic, bool closeLoopFlag, bool walkingMode);
    void read_directory(const std::string& name, stringvec& v);
    std::vector<std::vector<double>> localDataFilter(Datacontainer& gpsData, std::vector<double>& UTM_ref, double& radius );
    std::vector<std::vector<double>> localDataFilterAuto(Datacontainer& gpsData, std::vector<double>& UTM_ref, double& radius, double prevStamp );
    void timeDataFilter(Datacontainer& dataContainer, std::vector<std::vector<double>>& timeRange, Datacontainer& output);
    void timeDataFilter(pCloudcontainer& pcContainer, std::vector<std::vector<double>>& timeRange, pCloudcontainer& output);
    //bool compareFunction (std::string a, std::string b);

private:





};
}
