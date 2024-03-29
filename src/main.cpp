#include <rosbag/bag.h>
#include <stdlib.h>
#include <stdio.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/types.h>
#include <time.h>
#include <sys/stat.h>
#include "IO/IO.h"

using namespace mapper;

int main()
{

    //  if (argc<3){
    //      std::cout <<"usage: ./backpack_bags <path-to-output> <path-to-bags>"<<std::endl;
    //  }

    int dir;
    YAML::Node config = YAML::LoadFile("../config.yaml");

    std::string gpsTopic = config["gpsTopic"].as<std::string>();
    std::string base_link = config["base_link"].as<std::string>();
    std::string tfTopic = config["tfTopic"].as<std::string>();
    std::string radiusStr = config["radius"].as<std::string>();
    std::string pathOut = config["pathOut"].as<std::string>();
    std::string imuTopic = config["imuTopic"].as<std::string>();
    std::string ptCloudTopic = config["ptCloudTopic"].as<std::string>();
    std::string leafSize0 = config["voxelSize"].as<std::string>();
    std::string autoDist0 = config["autoGenerateDist"].as<std::string>();
    float leafSize=std::stod(leafSize0);
    float autoDist=std::stod(autoDist0);

    std::string icpConfigFilePath = config["icpConfig"].as<std::string>();
    std::string inputFiltersConfigFilePath = config["icpInputFilters"].as<std::string>();
    std::string inputFilters2ConfigFilePath = config["boundingBoxFilter"].as<std::string>();

    // std::string icpConfigFilePath;  //get path
    //std::cout<<icpConfigFilePath<<std::endl;
    std::string mapPostFiltersConfigFilePath = config["icpPostFilters"].as<std::string>();

    std::string	semanticPtCloudTopic="";
    std::string	rtkTopic="";
    std::vector<std::string> sourceBags = config["sourceBags"].as<std::vector<std::string>>();
    std::vector<std::string> Coordinates = config["Coordinates"].as<std::vector<std::string>>();
    double lat=std::stod(Coordinates[0]);
    double lon=std::stod(Coordinates[1]);
    double radius=std::stod(radiusStr);
    std::string RTKstr = config["RTK"].as<std::string>();
    bool RTK=false;
    if (RTKstr=="True" || RTKstr=="true")
    {
        RTK=true;
    }
    std::string walkingStr = config["autoWalkingMode"].as<std::string>();
      bool walkingMode=false;
      if (walkingStr=="True" || walkingStr=="true")
      {
          walkingMode=true;
      }

    std::string autoMapstr = config["autoGenerateMaps"].as<std::string>();
    bool autoGenerateMaps=false;
    if (autoMapstr=="True" || autoMapstr=="true")
    {
        autoGenerateMaps=true;
    }


    std::string closeLoopstr = config["closeLoop"].as<std::string>();
    bool closeLoopFlag=false;
    if (closeLoopstr=="True" || closeLoopstr=="true")
    {
        closeLoopFlag=true;
    }



    std::string computeProbDynamicstr = config["icpComputeProbDynamic"].as<std::string>();
    bool computeProbDynamic=false;

    if (computeProbDynamicstr=="True" || computeProbDynamicstr=="true")
    {
        computeProbDynamic=true;
    }


    bool semantics=false;
    std::string semanticsstr = config["semantics"].as<std::string>();

    if (semanticsstr=="True" || semanticsstr=="true")
    {
        semantics=true;
    }

    if (RTK)
    {
        rtkTopic = config["rtkTopic"].as<std::string>();
    }

    if (semantics)
    {
        semanticPtCloudTopic = config["semanticPtCloudTopic"].as<std::string>();
    }

    //std::cout<<sourceBags.size()<<std::endl;
    int mapNumber=sourceBags.size();
    std::vector<std::string> topics;
    //topics.push_back(std::string("/camera1/image_raw")   );
    //topics.push_back(std::string("/camera1/shutter_time"));
    topics.push_back(std::string(gpsTopic));
    if (semantics)
    {
        topics.push_back(std::string(semanticPtCloudTopic));
    }
    else
    {
        topics.push_back(std::string(ptCloudTopic));
    }
    topics.push_back(std::string(imuTopic));
    if (RTK){
        topics.push_back(std::string(rtkTopic));
    }
    else{
        topics.push_back(std::string(""));
    }
    topics.push_back(std::string(tfTopic));


    std::string currentPath="";
    for (int i=0; i<mapNumber; i++){

        //std::cout<<i<<std::endl;
        int mapCounter=i;
        //std::cout<<pathOut<<std::endl;
        std::ostringstream ss;
        ss<<(i);
        currentPath=pathOut+"/"+ss.str()+"/";
        std::cout<<currentPath<<std::endl;
        dir=mkdir (currentPath.c_str(),S_IRWXU);
        IO* Io =new IO();
        Io ->readBags(sourceBags[i], currentPath, topics, autoGenerateMaps, autoDist,  lat, lon, radius, base_link, mapCounter, semantics, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, inputFilters2ConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, closeLoopFlag, walkingMode);
        delete Io;
    }


}
