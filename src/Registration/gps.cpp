#include "IO/IO.h"
//#include "IO/settings.h"
#include "Registration/gps.h"


namespace mapper
{
GPS::GPS()
{


}

GPS::~GPS()
{

}

void GPS::createMap(std::string currentPath,Eigen::Matrix4f imu2base,Eigen::Matrix4f pc2base,Eigen::Matrix4f gps2base, IO::Datacontainer imuContainer, IO::pCloudcontainer pcContainer, IO::Datacontainer gpsUTMContainer, std::string filename, float leafSize)
{
    IO::pCloudcontainer pcContainer2=pcContainer;
    std::cout<<"GPS Map Registration...."<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    std::cout<<"Preaparing voxel grid with leafSize "<<leafSize<<" m"<<std::endl;
    for (unsigned int i=0; i<pcContainer.timestamp.size();i++){
        Eigen::Quaternion<float> rotation;
        rotation.x()=( imuContainer.vect[i][1] );
        rotation.y()=( imuContainer.vect[i][2]  );
        rotation.z()=( imuContainer.vect[i][3]  );
        rotation.w()=( imuContainer.vect[i][4]  );
        Eigen::Matrix3f rotM=rotation.toRotationMatrix();
        Eigen::Matrix4f transform= Eigen::Matrix4f::Identity();
        transform.block(0,0,3,3) = rotM;
        transform=imu2base*transform;
        transform(0,3)=gpsUTMContainer.vect[i][1]; transform(1,3)=gpsUTMContainer.vect[i][2]; transform(2,3)=gpsUTMContainer.vect[i][3];

        // pcContainer.XYZRGBL[i];
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
        *tempCloud=pcContainer.XYZRGBL[i];
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
        sor.setInputCloud (tempCloud);
        sor.setMeanK (10);
        sor.setStddevMulThresh (1.0);
        sor.filter (*tempCloud);


        pcl::VoxelGrid<pcl::PointXYZRGBL> vgrid;
        vgrid.setInputCloud (tempCloud);

        vgrid.setMinimumPointsNumberPerVoxel(2);
        vgrid.setLeafSize (leafSize, leafSize, leafSize);
        // vgrid.setDownsampleAllData(false);
        vgrid.filter (*tempCloud);



        pcContainer.XYZRGBL[i]=*tempCloud;




        for (unsigned int j=0; j<pcContainer.XYZRGBL[i].points.size(); j++){

            double x=pcContainer.XYZRGBL[i].points[j].x; double y=pcContainer.XYZRGBL[i].points[j].y; double z=pcContainer.XYZRGBL[i].points[j].z;
            Eigen::Vector4f pcPoints(x,y,z,1.0);
            Eigen::Vector4f pcPointsTransformed=transform*pc2base*pcPoints;

            pcContainer2.XYZRGBL[i].points[j].x=pcPointsTransformed[0];
            pcContainer2.XYZRGBL[i].points[j].y=pcPointsTransformed[1];
            pcContainer2.XYZRGBL[i].points[j].z=pcPointsTransformed[2];

            pointCloud->points.push_back(pcContainer2.XYZRGBL[i].points[j]);


        }

    }

    pcContainer2.XYZRGBL.clear();
    pcContainer2.XYZRGBL.shrink_to_fit();
    std::string fullPath1= currentPath + "/" + filename + ".pcd" ;
    std::string fullPath2= currentPath + "/" + filename + ".ply" ;


    //std::cout<<pcContainer.XYZRGBL.size()<<std::endl;
    //std::cout<<pcContainer.XYZRGBL[0].points.size()<<std::endl;
    std::cout<<std::endl;
    std::cout<<fullPath1<<std::endl;
    std::cout<<fullPath2<<std::endl;
    std::cout<<std::endl;
    pointCloud->height=1;
    pointCloud->width=pointCloud->points.size();



    //  std::cout<<"Statistical Outlier Removal... "<<std::endl;
    //  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> sor;
    //  sor.setInputCloud (pointCloud);
    //  sor.setMeanK (5000);
    //  sor.setStddevMulThresh (1);
    //  sor.filter (*pointCloud);


    //   pcl::VoxelGrid<pcl::PointXYZRGBL> vgrid;
    //   vgrid.setInputCloud (pointCloud);

    //   vgrid.setLeafSize (leafSize, leafSize, leafSize);
    //  pcl::PointCloud<pcl::PointXYZRGBL> pointCloudVoxelized;
    //  vgrid.filter (*pointCloud);





    std::cout<<"Saving "<<pointCloud->points.size()<<" points"<<std::endl;
    pcl::io::savePCDFileASCII (fullPath1, *pointCloud);
    pcl::io::savePLYFileASCII (fullPath2, *pointCloud);


}
}
