#include "IO/IO.h"
#include "IO/settings.h"
#include "Registration/icp.h"
namespace mapper
{
ICP::ICP()
{
}

ICP::~ICP()
{
}

void ICP::createMap(std::string currentPath,Eigen::Matrix4d imu2base,Eigen::Matrix4d pc2base,Eigen::Matrix4d gps2base, IO::Datacontainer imuContainer, IO::pCloudcontainer pcContainer, IO::Datacontainer gpsUTMContainer, std::string filename, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic, bool semantics)
{
    std::ofstream poseStream;
    poseStream.precision(16);
    std::string posePath= currentPath + "/" + filename + ".csv" ;
    poseStream.open (posePath.c_str());


    std::ofstream timeStream;
    timeStream.precision(16);
    std::string timePath= currentPath + "/" + filename + ".txt" ;
    timeStream.open (timePath.c_str());


    poseStream <<"%timestamp,x,y,z,qx,qy,qz,qw\n";
    // ---------------------------------------------------------------------------------------------------------------//
    typedef PointMatcher<float> PM;
    //typedef PointMatcherIO<float> PMIO;
    typedef PM::TransformationParameters TP;
    typedef PM::DataPoints DP;
    float priorDynamic;
    using namespace PointMatcherSupport;
    //string outputFileName(argv[0]);

    // Rigid transformation
    std::shared_ptr<PM::Transformation> rigidTrans;
    rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

    // Create filters manually to clean the global map
    std::shared_ptr<PM::DataPointsFilter> densityFilter =
            PM::get().DataPointsFilterRegistrar.create(
                "SurfaceNormalDataPointsFilter",
    {
                    {"knn", "10"},
                    {"epsilon", "5"},
                    {"keepNormals", "0"},
                    {"keepDensities", "1"}
                }
                );

    std::shared_ptr<PM::DataPointsFilter> maxDensitySubsample =
            PM::get().DataPointsFilterRegistrar.create(
                "MaxDensityDataPointsFilter",
    {{"maxDensity", toParam(30)}}
                );
    // Main algorithm definition
    PM::ICP icp;


    PM::DataPointsFilters inputFilters;
    PM::DataPointsFilters mapPostFilters;
    if(!icpConfigFilePath.empty())
    {
        std::ifstream ifs(icpConfigFilePath.c_str());
        icp.loadFromYaml(ifs);
        std::cout<<"loaded icp yaml!"<<std::endl;
        ifs.close();
    }
    else
    {
        icp.setDefault();
    }

    if(!inputFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(inputFiltersConfigFilePath.c_str());
        inputFilters = PM::DataPointsFilters(ifs);
        std::cout<<"loaded input filter yaml!"<<std::endl;
        ifs.close();
    }

    if(!mapPostFiltersConfigFilePath.empty())
    {
        std::ifstream ifs(mapPostFiltersConfigFilePath.c_str());
        mapPostFilters = PM::DataPointsFilters(ifs);
        std::cout<<"loaded post filter yaml!"<<std::endl;
        ifs.close();
    }


    // load YAML config
    //ifstream ifs(icpyml);
    //validateFile(argv[1]);
    // icp.loadFromYaml(ifs);

    //PMIO::FileInfoVector list(argv[2]);

    PM::DataPoints mapPointCloud, newCloud;
    TP T_to_map_from_new = TP::Identity(4,4); // assumes 3D

    //-----------------------------------------------------------------------------/////////////////////////////////---------------------------/
    /*  for (unsigned int i=0; i<pcContainer.timestamp.size();i++){
        for (unsigned int j=0; j<pcContainer.XYZRGBL[i].points.size(); j++){
            pcContainer.XYZRGBL[i].points[j].x =pcContainer.XYZRGBL[i].points[j].x-gpsUTMContainer.vect[i][1];
            pcContainer.XYZRGBL[i].points[j].y =pcContainer.XYZRGBL[i].points[j].y-gpsUTMContainer.vect[i][2];
            pcContainer.XYZRGBL[i].points[j].z =pcContainer.XYZRGBL[i].points[j].z-gpsUTMContainer.vect[i][3];
        }
    }
*/
    IO::pCloudcontainer pcContainer2=pcContainer;
    std::cout<<"ICP Map Registration...."<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    // std::cout<<"Preaparing voxel grid with leafSize "<<leafSize<<" m"<<std::endl;
    Eigen::Affine3d transform0= Eigen::Affine3d::Identity();
    Eigen::Affine3d transformPrevIcp= Eigen::Affine3d::Identity();
    Eigen::Affine3d transformPrev= Eigen::Affine3d::Identity();
    Eigen::Affine3d initialEstimate= Eigen::Affine3d::Identity();
    Eigen::Affine3d icpIncrement= Eigen::Affine3d::Identity();
    Eigen::Affine3d pose= Eigen::Affine3d::Identity();
    Eigen::Affine3d transformRot0= Eigen::Affine3d::Identity();
    Eigen::Affine3d transformPrevRot= Eigen::Affine3d::Identity();

    Eigen::Affine3d transformICP= Eigen::Affine3d::Identity();
    // double xOff=0; double yOff=0; double zOff=0;
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
            pose=transform0;


        }

        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
        *tempCloud=pcContainer.XYZRGBL[i];


        pcContainer.XYZRGBL[i]=*tempCloud;
        DP data;
        DP ref;

        //std::cout<<" pcContainer.normals[i].getMatrixXfMap() " << std::endl;
        //std::cout<<pcContainer.normals[i].getMatrixXfMap(3,8,0).row(0)<<std::endl;
        Eigen::MatrixXf dataNormals(3,pcContainer.normals[i].getMatrixXfMap(3,8,0).row(0).size());

        dataNormals.row(0)=pcContainer.normals[i].getMatrixXfMap(3,8,0).row(0);
        dataNormals.row(1)=pcContainer.normals[i].getMatrixXfMap(3,8,0).row(1);
        dataNormals.row(2)=pcContainer.normals[i].getMatrixXfMap(3,8,0).row(2);
        //std::cout<<" n_x0 is "<<pcContainer.normals[i].points[0].normal_x<<std::endl;
        //std::cout<<" n_x1 is "<<pcContainer.normals[i].points[1].normal_x<<std::endl;
        //std::cout<<" n_y1 is "<<pcContainer.normals[i].points[0].normal_y<<std::endl;
        //std::cout<<" n_z1 is "<<pcContainer.normals[i].points[0].normal_z<<std::endl;




        Eigen::MatrixXf datax(1,pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(0).size());
        Eigen::MatrixXf datay(1,pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(1).size());
        Eigen::MatrixXf dataz(1,pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(2).size());
        datax=pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(0);
        datay=pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(1);
        dataz=pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(2);
        // datax.array()=datax.array()-xOff;
        //datay.array()=datay.array()-yOff;
        //dataz.array()=dataz.array();

        data.addFeature("x", datax);
        data.addFeature("y", datay);
        data.addFeature("z", dataz);
        //data.addFeature("x", pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(0));
        // data.addFeature("y", pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(1));
        // data.addFeature("z", pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(2));
        // data.addDescriptor("normals", dataNormals);
        if (semantics)
        {
            Eigen::MatrixXf dataSemantics(1,pcContainer.normals[i].getMatrixXfMap(3,8,0).row(0).size());
            Eigen::MatrixXf semanticWeightsFull(1,pcContainer.normals[i].getMatrixXfMap(3,8,0).row(0).size());



            //  data.Labels=
            //   std::cout<<"-------LABELS--------------------------------------"<<std::endl;
            // std::cout<<pcContainer.XYZRGBL[i].getMatrixXfMap(7,8,0).row(5)<<std::endl;

            // std::cout<< " ---------------Labels--------------------------SHOULD BE"<<std::endl;
            for (unsigned int j=0; j<pcContainer.XYZRGBL[i].points.size(); j++){
                dataSemantics(0,j)=(float)pcContainer.XYZRGBL[i].points[j].label;
                semanticWeightsFull(0,j)=semanticWeights[pcContainer.XYZRGBL[i].points[j].label];
                //std::cout<<" semanticWeightsFull is "<<semanticWeightsFull(0,j)<<std::endl;

            }

            //  std::vector<std::string> labels1;

            //=======dataSemantics.row(0)=pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(0);


            data.addDescriptor("semantics", dataSemantics);
            data.addDescriptor("semanticWeights",semanticWeightsFull);
            // for (int l=0; l< pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(6).size() ; l++)
            // {

            //        labels1.push_back(labels[ pcContainer.XYZRGBL[i].getMatrixXfMap(3,8,0).row(6)[l] ]) ;

            //  }
            //   data.Labels(labels1);

        }
        else
        {
            Eigen::MatrixXf dataSemantics, semanticWeightsFull;
            dataSemantics=Eigen::MatrixXf::Constant (1,pcContainer.normals[i].getMatrixXfMap(3,8,0).row(0).size(),1);
            semanticWeightsFull=Eigen::MatrixXf::Constant (1,pcContainer.normals[i].getMatrixXfMap(3,8,0).row(0).size(),1);
            data.addDescriptor("semantics", dataSemantics);
            data.addDescriptor("semanticWeights",semanticWeightsFull);


        }
        if (i>=1) {

            // ref.features=pcContainer.XYZRGBL[i-1].getMatrixXfMap();
            //    ref=pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,4,0);
            Eigen::MatrixXf refNormals(3,pcContainer.normals[i-1].getMatrixXfMap(3,8,0).row(0).size());

            refNormals.row(0)=pcContainer.normals[i-1].getMatrixXfMap(3,8,0).row(0);
            refNormals.row(1)=pcContainer.normals[i-1].getMatrixXfMap(3,8,0).row(1);
            refNormals.row(2)=pcContainer.normals[i-1].getMatrixXfMap(3,8,0).row(2);

            ref.addFeature("x", pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(0));
            ref.addFeature("y", pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(1));
            ref.addFeature("z", pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(2));
            ref.addDescriptor("normals", refNormals);

            std::cout<<"---------------------------"<<std::endl;

            // std::cout<<"REF Features is "<<ref.features<<std::endl;
            //  std::cout<<"REF Features size is "<<ref.features.size()<<std::endl;
            //   std::cout<<"matrix points size is "<<  pcContainer.XYZRGBL[i-1].points.size()<<std::endl;
            // std::cout<<"matrix size is "<<  pcContainer.XYZRGBL[i-1].getMatrixXfMap().size()<<std::endl;
            //std::cout<<"original matrix is "<<std::endl;
            // std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(0)<<std::endl;
            //     std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(0).size()<<std::endl;
            //    std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(1).size()<<std::endl;
            //     std::cout<<pcContainer.XYZRGBL[i-1].getMatrixXfMap(3,8,0).row(2).size()<<std::endl;
            //std::cout<<"x1 is "<<  pcContainer.XYZRGBL[i-1].points[0].x<<std::endl;
            //  std::cout<<"x2 is "<<  pcContainer.XYZRGBL[i-1].points[1].x<<std::endl;
            // std::cout<<"y1 is "<<  pcContainer.XYZRGBL[i-1].points[0].y<<std::endl;
            //  std::cout<<"z1 is "<<  pcContainer.XYZRGBL[i-1].points[0].z<<std::endl;

            initialEstimate=transformPrev.inverse()*transform;

            //initialEstimate=transform0.inverse()*transform;


            //---------------------------------------------------------------------------------------------------------//
            //            for(unsigned i=0; i < list.size(); i++)
            //           {
            //  cout << "---------------------\nLoading: " << list[i].readingFileName << endl;

            // It is assume that the point cloud is express in sensor frame
            //newCloud = DP::load(list[i].readingFileName);
            newCloud=data;

            if(computeProbDynamic)
            {
                newCloud.addDescriptor("probabilityDynamic", PM::Matrix::Constant(1, newCloud.features.cols(), priorDynamic));
            }
            inputFilters.apply(newCloud);
            //mapPointCloud=ref;
            if(mapPointCloud.getNbPoints()  == 0)
            {
                mapPointCloud = newCloud;
                continue;
            }

            // call ICP
            try
            {
                // We use the last transformation as a prior
                // this assumes that the point clouds were recorded in
                // sequence.
                //const TP prior = initialEstimate.matrix().cast<float>();
                //const TP prior = T_to_map_from_new;
                const TP prior = T_to_map_from_new*initialEstimate.matrix().cast<float>();

                T_to_map_from_new = icp(newCloud, mapPointCloud, prior);


            }
            catch (PM::ConvergenceError& error)
            {
                std::cout << "ERROR PM::ICP failed to converge: " << std::endl;
                std::cout << "   " << error.what() << std::endl;
                continue;
            }

            for (int n=0; n<2; n++){

                if (!T_to_map_from_new.isIdentity(0.01)){break;}

                try
                {
                    const TP prior=T_to_map_from_new;
                    T_to_map_from_new = icp(newCloud, mapPointCloud, prior);
                }

                catch (PM::ConvergenceError& error)
                {
                    std::cout << "ERROR PM::ICP failed to converge: " << std::endl;
                    std::cout << "   " << error.what() << std::endl;
                    continue;
                }

            }

            // This is not necessary in this example, but could be
            // useful if the same matrix is composed in the loop.
            //std::cout<<"before correction "<<std::endl;
            //std::cout<<T_to_map_from_new<<std::endl;
            T_to_map_from_new = rigidTrans->correctParameters(T_to_map_from_new);
            // std::cout<<"after correction "<<std::endl;
            // std::cout<<T_to_map_from_new<<std::endl;

            // Move the new point cloud in the map reference
            newCloud = rigidTrans->compute(newCloud, T_to_map_from_new);

            // Merge point clouds to map

            mapPointCloud.concatenate(newCloud);
            mapPostFilters.apply(mapPointCloud);
            // Clean the map


            // Save the map at each iteration
            //   stringstream outputFileNameIter;
            //  outputFileNameIter << outputFileName << "_" << i << ".vtk";

            //  cout << "outputFileName: " << outputFileNameIter.str() << endl;
            //  mapPointCloud.save(outputFileNameIter.str());

            //---------------------------------------------------------------------------------------------------//

            // DP data_out(data);

            //std::cout<< "transformPrev.inverse() is "<<std::endl;
            //std::cout<<transformPrev.inverse()<<std::endl;
            //  std::cout<< "transformPrev.transpose() is "<<std::endl;
            // std::cout<<transformPrev.transpose()<<std::endl;

            //  initialEstimate(2,3)=0;
            //  std::cout<<"initial estimate is "<<std::endl;
            //  std::cout<<initialEstimate.matrix()<<std::endl;

            //Eigen::Affine3d T;



            //icp.inspector = vtkInspect;
            // *rigidTrans=initialEstimate.matrix().cast<float>();
            // icp.transformations.push_back(rigidTrans);

            //   PM::TransformationParameters correction=PM::TransformationParameters::Identity(4,4);
            //icp.transformations.apply(data_out, initialEstimate.matrix().cast<float>());
            //       try {
            //          const PM::TransformationParameters prior = initialEstimate.matrix().cast<float>();
            //         correction = icp(data, ref,prior  );
            //     }
            //      catch (PM::ConvergenceError& error)
            //     {
            //        std::cout << "ERROR PM::ICP failed to converge: " << std::endl;
            //      std::cout << "   " << error.what() << std::endl;
            //     continue;
            // }


            PM::TransformationParameters T=T_to_map_from_new;
            // PM::TransformationParameters T=initialEstimate.matrix().cast<float>();
            //  std::cout<<"correction is"<<correction<<std::endl;
            //  T=initialEstimate.matrix().cast<float>()* correction ;//
            //T=initialEstimate.matrix().cast <float>();

            // std::cout<<"REF Features SIZE is "<<ref.features.size()<<std::endl;
            // std::cout<<"DATAOUT Features SIZE is "<<data_out.features.size()<<std::endl;

            // Eigen::Matrix4d T= Eigen::Matrix4d::Identity();
            //transformICP.matrix()= transformICP.matrix()*T.cast <double> ();
            transformICP.matrix()=T.cast <double> ();
        }

        icpIncrement=transformPrevIcp.inverse()*transformICP;
        //Eigen::Matrix4d pose=transform0.matrix()*pc2base*transformICP.matrix();

        pose.matrix()=transform0.matrix()*pc2base*transformICP.matrix();


        Eigen::Matrix3d poseRot=pose.matrix().block(0,0,3,3);
        Eigen::Quaterniond q(poseRot);
        poseStream<<pcContainer.timestamp[i]<<","<<pose.matrix()(0,3)<<","<<pose.matrix()(1,3)<<","<<pose.matrix()(2,3)<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<std::endl;
        double t=pcContainer.timestamp[i];
        for (unsigned int j=0; j<pcContainer.XYZRGBL[i].points.size(); j++){

            double x=pcContainer.XYZRGBL[i].points[j].x; double y=pcContainer.XYZRGBL[i].points[j].y; double z=pcContainer.XYZRGBL[i].points[j].z;
            Eigen::Vector4d pcPoints(x,y,z,1.0);
            Eigen::Vector4d pcPointsTransformed=pose.matrix()*pcPoints;

            //  Eigen::Vector4d pcPointsTransformed=transform0.matrix()*pc2base*transformICP.matrix()*pcPoints;

            //transform0*pc2base*

            pcContainer2.XYZRGBL[i].points[j].x=pcPointsTransformed[0]-gpsUTMContainer.vect[0][1];
            pcContainer2.XYZRGBL[i].points[j].y=pcPointsTransformed[1]-gpsUTMContainer.vect[0][2];
            pcContainer2.XYZRGBL[i].points[j].z=pcPointsTransformed[2]-gpsUTMContainer.vect[0][3];
            pointCloud->points.push_back(pcContainer2.XYZRGBL[i].points[j]);
            timeStream<<t<<std::endl;

        }
        transformPrev=transform;
        transformPrevIcp=transformICP;
        transformPrevRot.matrix().block(0,0,3,3)=transform.matrix().block(0,0,3,3);
    }

    mapPointCloud = densityFilter->filter(mapPointCloud);
    mapPointCloud = maxDensitySubsample->filter(mapPointCloud);
    DP mapPointCloud2=mapPointCloud;

    mapPointCloud2 = rigidTrans->compute(mapPointCloud2, (transformRot0.matrix()*pc2base).cast <float> ());


    //  mapPointCloud.save("map.vtk");
    poseStream.close();
    timeStream.close();
    pcContainer2.XYZRGBL.clear();
    pcContainer2.XYZRGBL.shrink_to_fit();
    std::string fullPath1= currentPath + "/" + filename + ".pcd" ;
    std::string fullPath2= currentPath + "/" + filename + ".ply" ;
    std::string fullPath3= currentPath + "/" + filename + ".vtk" ;
    mapPointCloud2.save(fullPath3);
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
