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


    typedef PointMatcher<float> PM;
    typedef PointMatcherIO<float> PMIO;
    typedef PM::TransformationParameters TP;
    typedef PM::DataPoints DP;

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

    // load YAML config
    ifstream ifs(argv[1]);
    //validateFile(argv[1]);
    icp.loadFromYaml(ifs);

    //PMIO::FileInfoVector list(argv[2]);

    PM::DataPoints mapPointCloud, newCloud;
    TP T_to_map_from_new = TP::Identity(4,4); // assumes 3D

    for(unsigned i=0; i < list.size(); i++)
    {
        cout << "---------------------\nLoading: " << list[i].readingFileName << endl;

        // It is assume that the point cloud is express in sensor frame
        newCloud = DP::load(list[i].readingFileName);

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
            const TP prior = T_to_map_from_new;

            T_to_map_from_new = icp(newCloud, mapPointCloud, prior);
        }
        catch (PM::ConvergenceError& error)
        {
            cout << "ERROR PM::ICP failed to converge: " << endl;
            cout << "   " << error.what() << endl;
            continue;
        }

        // This is not necessary in this example, but could be
        // useful if the same matrix is composed in the loop.
        T_to_map_from_new = rigidTrans->correctParameters(T_to_map_from_new);

        // Move the new point cloud in the map reference
        newCloud = rigidTrans->compute(newCloud, T_to_map_from_new);

        // Merge point clouds to map
        mapPointCloud.concatenate(newCloud);

        // Clean the map
        mapPointCloud = densityFilter->filter(mapPointCloud);
        mapPointCloud = maxDensitySubsample->filter(mapPointCloud);

        // Save the map at each iteration
     //   stringstream outputFileNameIter;
      //  outputFileNameIter << outputFileName << "_" << i << ".vtk";

      //  cout << "outputFileName: " << outputFileNameIter.str() << endl;
      //  mapPointCloud.save(outputFileNameIter.str());
    }




}
}
