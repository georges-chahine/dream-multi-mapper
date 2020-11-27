#include "IO/IO.h"
//#include "IO/settings.h"
#include "Registration/icp.h"
namespace mapper
{
ICP::ICP()
{
}

ICP::~ICP()
{
}

void ICP::createMap(std::string currentPath,Eigen::Matrix4d imu2base,Eigen::Matrix4d pc2base,Eigen::Matrix4d gps2base, IO::Datacontainer imuContainer, IO::pCloudcontainer pcContainer, IO::Datacontainer gpsUTMContainer, std::string filename, float leafSize)
{
    PM::ICP icp;
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;
    //icp.setDefault();

    // Prepare reading filters
    name = "MinDistDataPointsFilter";
    params["minDist"] = "1.0";
    std::shared_ptr<PM::DataPointsFilter> minDist_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.05";
    std::shared_ptr<PM::DataPointsFilter> rand_read =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // Prepare reference filters
    name = "MinDistDataPointsFilter";
    params["minDist"] = "1.0";
    std::shared_ptr<PM::DataPointsFilter> minDist_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "RandomSamplingDataPointsFilter";
    params["prob"] = "0.05";
    std::shared_ptr<PM::DataPointsFilter> rand_ref =
            PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    // Prepare matching function
    name = "KDTreeMatcher";
    params["knn"] = "1";
    params["epsilon"] = "3.16";
    std::shared_ptr<PM::Matcher> kdtree =
            PM::get().MatcherRegistrar.create(name, params);
    params.clear();

    // Prepare outlier filters
    name = "TrimmedDistOutlierFilter";
    params["ratio"] = "0.75";
    std::shared_ptr<PM::OutlierFilter> trim =
            PM::get().OutlierFilterRegistrar.create(name, params);
    params.clear();

    // Prepare error minimization
    name = "PointToPointErrorMinimizer";
    std::shared_ptr<PM::ErrorMinimizer> pointToPoint =
            PM::get().ErrorMinimizerRegistrar.create(name);

    // Prepare outlier filters
    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "150";
    std::shared_ptr<PM::TransformationChecker> maxIter =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    name = "DifferentialTransformationChecker";
    params["minDiffRotErr"] = "0.001";
    params["minDiffTransErr"] = "0.01";
    params["smoothLength"] = "4";
    std::shared_ptr<PM::TransformationChecker> diff =
            PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    // Prepare inspector
    std::shared_ptr<PM::Inspector> nullInspect =
            PM::get().InspectorRegistrar.create("NullInspector");

    // Prepare transformation
    std::shared_ptr<PM::Transformation> rigidTrans =
            PM::get().TransformationRegistrar.create("RigidTransformation");

    // Build ICP solution
    icp.readingDataPointsFilters.push_back(minDist_read);
    icp.readingDataPointsFilters.push_back(rand_read);

    icp.referenceDataPointsFilters.push_back(minDist_ref);
    icp.referenceDataPointsFilters.push_back(rand_ref);

    icp.matcher = kdtree;

    icp.outlierFilters.push_back(trim);

    icp.errorMinimizer = pointToPoint;

    icp.transformationCheckers.push_back(maxIter);
    icp.transformationCheckers.push_back(diff);

    // toggle to write vtk files per iteration
    icp.inspector = nullInspect;




    IO::pCloudcontainer pcContainer2=pcContainer;
    std::cout<<"ICP Map Registration...."<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    // std::cout<<"Preaparing voxel grid with leafSize "<<leafSize<<" m"<<std::endl;
    Eigen::Affine3d transform0= Eigen::Affine3d::Identity();
    Eigen::Affine3d transformPrev= Eigen::Affine3d::Identity();
    Eigen::Affine3d initialEstimate= Eigen::Affine3d::Identity();
    Eigen::Affine3d transformRot0= Eigen::Affine3d::Identity();
    Eigen::Affine3d transformPrevRot= Eigen::Affine3d::Identity();

    Eigen::Affine3d transformICP= Eigen::Affine3d::Identity();

    for (unsigned int i=0; i<pcContainer.timestamp.size();i++){

        Eigen::Quaternion<double> rotation;
        rotation.x()=( imuContainer.vect[i][1] );
        rotation.y()=( imuContainer.vect[i][2]  );
        rotation.z()=( imuContainer.vect[i][3]  );
        rotation.w()=( imuContainer.vect[i][4]  );
        Eigen::Matrix3d rotM=rotation.toRotationMatrix();
        Eigen::Affine3d transform= Eigen::Affine3d::Identity();
        Eigen::Affine3d transformRot= Eigen::Affine3d::Identity();
        transform.matrix().block(0,0,3,3) = rotM;
        transformRot.matrix().block(0,0,3,3) = rotM;
        transform.matrix()=imu2base*transform.matrix();
        transform(0,3)=gpsUTMContainer.vect[i][1]; transform(1,3)=gpsUTMContainer.vect[i][2]; transform(2,3)=gpsUTMContainer.vect[i][3];

        if (i==0){
            transform0=transform;
            transformRot0.matrix().block(0,0,3,3) =transform0.matrix().block(0,0,3,3);
            //transformICP=transform0*pc2base;
        }

        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
        *tempCloud=pcContainer.XYZRGBL[i];


        pcContainer.XYZRGBL[i]=*tempCloud;
        DP data;
        DP ref;

        data.addFeature("x", pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(0));
        data.addFeature("y", pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(1));
        data.addFeature("z", pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(2));

        if (i>=1) {

            // ref.features=pcContainer.XYZRGBL[i-1].getMatrixXfMap();
            //    ref=pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,4,0);
            ref.addFeature("x", pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(0));
            ref.addFeature("y", pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(1));
            ref.addFeature("z", pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(2));

            std::cout<<"---------------------------"<<std::endl;

            // std::cout<<"REF Features is "<<ref.features<<std::endl;
            //  std::cout<<"REF Features size is "<<ref.features.size()<<std::endl;
            //   std::cout<<"matrix points size is "<<  pcContainer.XYZRGBL[i-1].points.size()<<std::endl;
            // std::cout<<"matrix size is "<<  pcContainer.XYZRGBL[i-1].getMatrixXfMap().size()<<std::endl;
            //std::cout<<"original matrix is "<<std::endl;
            // std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(0)<<std::endl;
            std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(0).size()<<std::endl;
            std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(1).size()<<std::endl;
            std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(2).size()<<std::endl;
            //std::cout<<"x1 is "<<  pcContainer.XYZRGBL[i-1].points[0].x<<std::endl;
            //  std::cout<<"x2 is "<<  pcContainer.XYZRGBL[i-1].points[1].x<<std::endl;
            // std::cout<<"y1 is "<<  pcContainer.XYZRGBL[i-1].points[0].y<<std::endl;
            //  std::cout<<"z1 is "<<  pcContainer.XYZRGBL[i-1].points[0].z<<std::endl;



            DP data_out(data);
            initialEstimate=transformPrev.inverse()*transform;
            //std::cout<< "transformPrev.inverse() is "<<std::endl;
            //std::cout<<transformPrev.inverse()<<std::endl;
            //  std::cout<< "transformPrev.transpose() is "<<std::endl;
            // std::cout<<transformPrev.transpose()<<std::endl;

            //  initialEstimate(2,3)=0;
            std::cout<<"initial estimate is "<<std::endl;
            std::cout<<initialEstimate.matrix()<<std::endl;

            //Eigen::Affine3d T;



            //icp.inspector = vtkInspect;
            // *rigidTrans=initialEstimate.matrix().cast<float>();
            icp.transformations.push_back(rigidTrans);

            PM::TransformationParameters correction=PM::TransformationParameters::Identity(4,4);
            //icp.transformations.apply(data_out, initialEstimate.matrix().cast<float>());
            try {
              const PM::TransformationParameters prior = initialEstimate.matrix().cast<float>();
              correction = icp(data, ref,prior  );
            }
            catch (PM::ConvergenceError& error)
            {
                std::cout << "ERROR PM::ICP failed to converge: " << std::endl;
                std::cout << "   " << error.what() << std::endl;
                continue;
            }


            PM::TransformationParameters T;
            std::cout<<"correction is"<<correction<<std::endl;
            T=initialEstimate.matrix().cast<float>()* correction ;//
            //T=initialEstimate.matrix().cast <float>();

            // std::cout<<"REF Features SIZE is "<<ref.features.size()<<std::endl;
            // std::cout<<"DATAOUT Features SIZE is "<<data_out.features.size()<<std::endl;

            // Eigen::Matrix4d T= Eigen::Matrix4d::Identity();
            transformICP.matrix()= transformICP.matrix()*T.cast <double> ();
        }
        for (unsigned int j=0; j<pcContainer.XYZRGBL[i].points.size(); j++){

            double x=pcContainer.XYZRGBL[i].points[j].x; double y=pcContainer.XYZRGBL[i].points[j].y; double z=pcContainer.XYZRGBL[i].points[j].z;
            Eigen::Vector4d pcPoints(x,y,z,1.0);
            Eigen::Vector4d pcPointsTransformed=transform0.matrix()*transformICP.matrix()*pc2base*pcPoints;
            //transform0*pc2base*
            pcContainer2.XYZRGBL[i].points[j].x=pcPointsTransformed[0];
            pcContainer2.XYZRGBL[i].points[j].y=pcPointsTransformed[1];
            pcContainer2.XYZRGBL[i].points[j].z=pcPointsTransformed[2];
            pointCloud->points.push_back(pcContainer2.XYZRGBL[i].points[j]);
        }
        transformPrev=transform;
        transformPrevRot.matrix().block(0,0,3,3)=transform.matrix().block(0,0,3,3);
    }
    pcContainer2.XYZRGBL.clear();
    pcContainer2.XYZRGBL.shrink_to_fit();
    std::string fullPath1= currentPath + "/" + filename + ".pcd" ;
    std::string fullPath2= currentPath + "/" + filename + ".ply" ;
    std::cout<<std::endl;
    std::cout<<fullPath1<<std::endl;
    std::cout<<fullPath2<<std::endl;
    std::cout<<std::endl;
    pointCloud->height=1;
    pointCloud->width=pointCloud->points.size();
    std::cout<<"Saving "<<pointCloud->points.size()<<" points"<<std::endl;
    pcl::io::savePCDFileASCII (fullPath1, *pointCloud);
    pcl::io::savePLYFileASCII (fullPath2, *pointCloud);
}
}
