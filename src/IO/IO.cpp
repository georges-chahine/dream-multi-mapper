#include "IO/IO.h"
#include "IO/settings.h"
#include "Registration/gps.h"
#include "Registration/icp.h"


namespace mapper
{
IO::IO()
{


}

IO::~IO()
{

}

struct path_leaf_string
{
    std::string operator()(const boost::filesystem::directory_entry& entry) const
    {
        return entry.path().leaf().string();
    }
};

void IO::read_directory(const std::string& name, stringvec& v)
{
    boost::filesystem::path p(name);
    boost::filesystem::directory_iterator start(p);
    boost::filesystem::directory_iterator end;
    std::transform(start, end, std::back_inserter(v), path_leaf_string());
}


std::vector<std::vector<double>> IO::localDataFilterAuto(Datacontainer& gpsData, std::vector<double>& UTM_ref, double& radius, double prevStamp)
{
    std::vector<std::vector<double>> timeRange;
    std::vector<double> emptyVec;
    timeRange.push_back(emptyVec);
    double timeOut=60.0;  //1 minute
    double distanceDifferential=0;
    double distanceReset=30.0;
    double lastTime=-999999999999999999;
    unsigned int rangeIndex=0;
    bool init=false;
    double x_0=gpsData.vect[0][1]; double y_0=gpsData.vect[0][2]; double z_0=gpsData.vect[0][3];
    if (prevStamp==0){

        timeRange[0].push_back(gpsData.vect[0][0]);  //meh, should be improved, this means we always start around the same position


    }

    for (unsigned int i=0; i<gpsData.vect.size(); i++)

    {




        if (gpsData.vect[i][0]<prevStamp && prevStamp!=0 ){continue;}
        std::cout.precision(20);
        std::cout<<std::setprecision(20);

        double x=gpsData.vect[i][1]; double y=gpsData.vect[i][2]; double z=gpsData.vect[i][3];
        double distance=sqrt(pow(UTM_ref[0]-gpsData.vect[i][1],2)+pow(UTM_ref[1]-gpsData.vect[i][2],2));
        //    std::cout<<"distance is"<<distance<<std::endl;
        //    std::cout<<"radius is"<<radius<<std::endl;
        //      std::cout<<"GPS TIME IS "<<gpsData.vect[i][0]<<" distance is "<<distance<<" radius*2 is "<<radius*2<<std::endl;
        if (distance<(radius*2)){
            //     std::cout<<"distance2 is"<<distance<<std::endl;
            //    std::cout<<"radius2 is"<<radius<<std::endl;
            distanceDifferential=sqrt(pow(x_0-x,2)+pow(y_0-y,2));
            //     std::cout<<"distance differential is "<<distance<<std::endl;
            //    if (init==false) {distanceDifferential=0;}
            if (distanceDifferential<distanceReset||(gpsData.vect[i][0]-lastTime)<timeOut || init==false)
            {

                timeRange[rangeIndex].push_back(gpsData.vect[i][0]);
                init=true;
            }
            else
            {
                rangeIndex++;
                std::vector<double> emptyVec;
                timeRange.push_back(emptyVec);
                timeRange[rangeIndex].push_back(gpsData.vect[i][0]);
            }
            lastTime=gpsData.vect[i][0];
            x_0=gpsData.vect[i][1]; y_0=gpsData.vect[i][2]; z_0=gpsData.vect[i][3];
        }

    }
    return timeRange;
}


std::vector<std::vector<double>> IO::localDataFilter(Datacontainer& gpsData, std::vector<double>& UTM_ref, double& radius)
{  std::vector<std::vector<double>> timeRange;
    std::vector<double> emptyVec;
    timeRange.push_back(emptyVec);
    double timeOut=8.0;
    double distanceDifferential=0;
    double distanceReset=30.0;
    double lastTime=-999999999999999999;
    unsigned int rangeIndex=0;
    bool init=false;
    double x_0=gpsData.vect[0][1]; double y_0=gpsData.vect[0][2]; double z_0=gpsData.vect[0][3];
    for (unsigned int i=0; i<gpsData.vect.size(); i++)

    {
        std::cout.precision(20);
        std::cout<<std::setprecision(20);

        double x=gpsData.vect[i][1]; double y=gpsData.vect[i][2]; double z=gpsData.vect[i][3];
        double distance=sqrt(pow(UTM_ref[0]-gpsData.vect[i][1],2)+pow(UTM_ref[1]-gpsData.vect[i][2],2));
        //    std::cout<<"distance is"<<distance<<std::endl;
        //    std::cout<<"radius is"<<radius<<std::endl;
        //      std::cout<<"GPS TIME IS "<<gpsData.vect[i][0]<<" distance is "<<distance<<" radius*2 is "<<radius*2<<std::endl;
        if (distance<(radius*2)){
            //     std::cout<<"distance2 is"<<distance<<std::endl;
            //    std::cout<<"radius2 is"<<radius<<std::endl;
            distanceDifferential=sqrt(pow(x_0-x,2)+pow(y_0-y,2));
            //     std::cout<<"distance differential is "<<distance<<std::endl;
            //    if (init==false) {distanceDifferential=0;}
            if (distanceDifferential<distanceReset||(gpsData.vect[i][0]-lastTime)<timeOut || init==false)
            {

                timeRange[rangeIndex].push_back(gpsData.vect[i][0]);
                init=true;
            }
            else
            {
                rangeIndex++;
                std::vector<double> emptyVec;
                timeRange.push_back(emptyVec);
                timeRange[rangeIndex].push_back(gpsData.vect[i][0]);
            }
            lastTime=gpsData.vect[i][0];
            x_0=gpsData.vect[i][1]; y_0=gpsData.vect[i][2]; z_0=gpsData.vect[i][3];
        }

    }
    return timeRange;
}
//void IO::packDataContainer(Datacontainer& dataContainer, Datacontainer& packedDataContainer)

//{

//}

std::vector<double> IO::getTimeLimits(std::vector<std::vector<double>>& timeRange)
{
    int maxSize=0;
    unsigned int index=0;
    std::cout<<"all size is "<<timeRange.size()<<std::endl;
    for (unsigned int i=0; i<timeRange.size(); i++ ){

        if (timeRange[i].size()>maxSize){
            maxSize=timeRange[i].size();
            index=i;
        }
    }
    std::cout<<"size is "<<timeRange[index].size()<<std::endl;
    double lowerTh=timeRange[0][0];//was [index][0]
    double upperTh=timeRange[0].back(); //was [index].back();
    return std::vector<double>{lowerTh, upperTh};
}
// Returns interpolated value at x from parallel arrays ( xData, yData )
//   Assumes that xData has at least two elements, is sorted and is strictly monotonic increasing
//   boolean argument extrapolate determines behaviour beyond ends of array (if needed)
/*
double IO::interpolate( std::vector<double> &xData, std::vector<double> &yData, double x, bool extrapolate )
{
    int size = xData.size();

    int i = 0;                                                                  // find left end of interval for interpolation
    if ( x >= xData[size - 2] )                                                 // special case: beyond right end
    {
        i = size - 2;
    }
    else
    {
        while ( x > xData[i+1] ) i++;
    }
    double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
    if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
    {
        if ( x < xL ) yR = yL;
        if ( x > xR ) yL = yR;
    }

    double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient

    return yL + dydx * ( x - xL );                                              // linear interpolation
}
*/

void IO::interpolateRefRange(Datacontainer& ref, Datacontainer& input, Datacontainer& output)
{

    output.frame_id=input.frame_id;
    output.header=input.header;
    int m=ref.vect.size();
    int n=input.vect[0].size();
    std::vector<std::vector<double>> vec(m, std::vector<double> (n, 0.0));
    output.vect=vec;
    std::vector<double> xData, yData;
    double x;
    //bool extrapolate=false;
    for (unsigned int i=0; i<ref.vect.size(); i++){
        double refTime=ref.vect[i][0];

        // std::cout<<"refTime is" <<refTime<<std::endl;
        xData.push_back(refTime);
    }
    for (unsigned int i=0; i<input.vect.size(); i++){
        double inputTime=input.vect[i][0];
        yData.push_back(inputTime);

    }
    std::vector<double> t(ref.vect.size(),999);

    for (unsigned int i=0; i<ref.vect.size(); i++){

        x=ref.vect[i][0];
        //output.vect[i][0]=interpolate(xData,yData,x,extrapolate);
        for (unsigned int m=0; m<yData.size(); m++){

            if (m==0 && yData[m]>=x){
                output.vect[i][0]=yData[m];
                break;
            }

            if (yData[m]>x)
            {
                output.vect[i][0]=yData[m-1]+(yData[m]-yData[m-1])*(  ( x- yData[m-1] )   /  (yData[m]- yData[m-1] ) );
                break;
            }

            if (  (m==(yData.size()-1)) ){
                output.vect[i][0]=yData[m];
                std::cout<<"extrapolating? not really"<<std::endl;
                break;


            }

        }
        std::cout.precision(17);
        //    std::cout<<" x is "<< x <<" output is "<<output.vect[i][0]<<std::endl;

        std::vector<double>::iterator lower, upper;

        lower=std::lower_bound (yData.begin(), yData.end(), output.vect[i][0]);
        upper=std::upper_bound (yData.begin(), yData.end(), output.vect[i][0]);
        int lowerBound, upperBound;
        bool errorFlag1=true;
        bool errorFlag2=true;
        for (unsigned int k=0; k<yData.size(); k++){
            if (yData[k]==*lower){
                lowerBound=k;
                errorFlag1=false;
            }

            if (yData[k]==*upper){
                upperBound=k;
                errorFlag2=false;
            }
        }
        if (yData.back()==output.vect[i][0])
        {
            upperBound=lowerBound;
            errorFlag2=false;
            upper=lower;

        }
        if (errorFlag1||errorFlag2)
        {
            std::cout<<"ERROR: couldn't find interpolation bounds"<<std::endl;
            std::cout<<"error flag1 is "<<errorFlag1<<" error flag2 is "<<errorFlag2<<std::endl;
            std::cout<<"*lower is "<<*lower<<" *upper is "<<*upper<<" upper-lower is "<<*upper-*lower<<" yData[0] is "<< yData[0] <<" *lower-yData[0] is "<< *lower-yData[0] <<std::endl;
            std::cout<<"yData.back() is "<<yData.back()<<" VAR is "<<output.vect[i][0]<< " diff is " <<yData.back()-output.vect[i][0] <<std::endl;
        }
        else
        {
            //     std::cout<<"interpolation success "<<std::endl;
            //    std::cout<<"*lower is "<<*lower<<" *upper is "<<*upper<<" upper-lower2 is "<<*upper-*lower<<std::endl;
        }

        if (*lower != *upper) {
            t[i]=((output.vect[i][0]-*lower) /(*upper-*lower));
        }
        else
        {
            t[i]=0;
            //   std::cout<<"*upper-*lower "<<std::endl;
        }
        // std::cout<<"t[i] is "<<t[i]<<std::endl;
        if (output.vect[i][0] == *lower)
        {
            //   std::cout<<"same as lower"<<std::endl;
        }
        for (unsigned int j=1; j<input.vect[0].size(); j++){
            output.vect[i][j]=input.vect[lowerBound][j]+t[i]*(input.vect[upperBound][j]-input.vect[lowerBound][j]);
        }
    }
}

//void IO::interpolateRefRange(Datacontainer& ref, pCloudcontainer& input, pCloudcontainer& output)
//{
//    output.frame_id=input.frame_id;

//}

void IO::matchDataToPointCloudRate(std::vector<double>& timestamp , Datacontainer& input, Datacontainer& output)
{

    output.header=input.header;
    output.frame_id=input.frame_id;
    // int m=timestamp.size();
    // int n=input.vect[0].size();
    // std::vector<std::vector<double>> vec(m, std::vector<double> (n, 0.0));
    // output.vect=vec;

    std::vector<int> indices;


    for (unsigned int i=0; i<timestamp.size();i++)
    {   int lastMinIdx=0;
        double lastMinVal=9999;
        for (unsigned int j=0; j<input.vect.size();j++)
        {
            // std::cout.precision(17);
            //  std::cout<<input.vect[j][0]<<std::endl;
            double minVal=fabs(input.vect[j][0]-timestamp[i]);
            if (minVal<lastMinVal)
            {
                lastMinVal=minVal;
                lastMinIdx=j;
            }
        }
        indices.push_back(lastMinIdx);
        output.vect.push_back(input.vect[lastMinIdx]);

        //  std::cout<<"indices are: "<<indices[i]<<std::endl;
    }
    std::cout<<"----------"<<std::endl;
}

void IO::timeDataFilter(Datacontainer& dataContainer, std::vector<std::vector<double>>& timeRange, Datacontainer& output)
{
    std::vector<double> Th=getTimeLimits(timeRange);
    double lowerTh=Th[0];
    double upperTh=Th[1];

    std::cout<<"lower th is "<<lowerTh<<" upper th is "<<upperTh<< " timeRange is " <<(upperTh-lowerTh)<<std::endl;

    output.frame_id= dataContainer.frame_id;
    output.header=dataContainer.header;
    for (unsigned int i=0; i<dataContainer.vect.size(); i++ )
    {
        if (dataContainer.vect[i][0]>lowerTh &&  dataContainer.vect[i][0]<upperTh){
            output.vect.push_back(dataContainer.vect[i]);

        }
    }
}

void IO::timeDataFilter(pCloudcontainer& pcContainer, std::vector<std::vector<double>>& timeRange, pCloudcontainer& output)
{

    std::vector<double> Th=getTimeLimits(timeRange);
    double lowerTh=Th[0];
    double upperTh=Th[1];
    std::cout<<"lower th is "<<lowerTh<<" upper th is "<<upperTh<<std::endl;
    output.frame_id= pcContainer.frame_id;
    for (unsigned int i=0; i<pcContainer.XYZRGBL.size(); i++ )
    {
        if (pcContainer.timestamp[i]>lowerTh &&  pcContainer.timestamp[i]<upperTh){
            output.XYZRGBL.push_back(pcContainer.XYZRGBL[i]);
            output.timestamp.push_back(pcContainer.timestamp[i]);
            output.normals.push_back(pcContainer.normals[i]);

        }
    }

}

Eigen::Matrix4d IO::parseTFcontainer(TFcontainer& container, std::string& sensor_frame, std::string& base_link)
{

    for (unsigned int i=0; i<container.frame_id.size(); i++)
    {
        //   std::cout<<"child_frame_id is "<<container.child_frame_id[i]<<" sensor frame is "<<sensor_frame<<" base_link is "<<base_link<<std::endl;
        if (sensor_frame==base_link){
            return Eigen::Matrix4d::Identity();
        }

        if ((container.child_frame_id[i]==sensor_frame) && (container.frame_id[i]==base_link))
        {
            return container.transform[i];
        }

        if ((container.child_frame_id[i]==sensor_frame) && (container.frame_id[i]!=base_link))
        {
            std::string temp_frame_id=container.frame_id[i];
            Eigen::Matrix4d temp_transform=container.transform[i];

            while (temp_frame_id!=base_link)
            {
                // do
                //   {
                //   std:: cout << '\n' << "Press a key to continue...";
                //   } while (std::cin.get() != '\n');


                //   std::cout<<"temp_frame_id is "<<temp_frame_id<<" child_frame_id is "<<container.child_frame_id[i]<<" base_link is "<<base_link<<std::endl;
                // temp_frame_id=container.frame_id[i];
                for (unsigned int j=0; j<container.frame_id.size(); j++)
                {

                    if ((container.child_frame_id[j]==temp_frame_id)){
                        temp_frame_id=container.frame_id[j];
                        temp_transform=container.transform[j]*temp_transform;
                        break;
                    }
                }
                if (container.frame_id[i] == temp_frame_id)
                {

                    std::cout<<"ERROR: error finding transform from "<<sensor_frame<<" to "<<base_link<<" , assuming Identity"<<std::endl;
                    return Eigen::Matrix4d::Identity();

                }
            }
            return temp_transform;
        }
    }
    std::cout<<"ERROR: could not find transform from "<<sensor_frame<<" to "<<base_link<<" , (no link found to parent frame), assuming Identity"<<std::endl;
    return Eigen::Matrix4d::Identity();
}

//bool IO::compareFunction (std::string a, std::string b) {return a<b;}

std::uint32_t IO::checkPointLabel(int& r, int& g, int& b)

{
    std::vector<int> current_color{r, g, b};
    //std::cout<<"current color1 is "<<r<<" "<<g<<" "<<b<< std::endl;
    for (unsigned int i=0; i<palette.size(); i++)
    {

        if (current_color==palette[i]) {
            //    std::cout<<"palette size is "<<palette.size()<<std::endl;
            //     std::cout<<"current color2 is "<<r<<" "<<g<<" "<<b<< std::endl;
            //     std::cout<<"corresponding palette color is "<<std::endl;
            //    std::copy(palette[i].begin(), palette[i].end(),std::ostream_iterator<int>(std::cout, "\n"));
            // std::cout<<"i is "<<i<<" std is "<< std::uint32_t(i)<<std::endl;
            return std::uint32_t(i);

            //std::copy(labels[i].begin(), labels[i].end(),std::ostream_iterator<std::string>(std::cout, "\n"));
            //    std::cout<<labels[i];
        }

    }
    return std::uint32_t((palette.size()-1));
    //if (r==0){

    //std::cout<<"r "<<(r)<<" g"<<(g)<<" b "<<(b)<<std::endl;
    //}
    //return 0;
}

void IO::readBags(std::string sourceBags, std::string currentPath, std::vector<std::string> topics, bool autoGenerateMaps, float autoDist, double lat, double lon, double radius, std::string base_link, std::string rMethod , int mapNumber, bool semantics, float leafSize, std::string icpConfigFilePath, std::string inputFiltersConfigFilePath, std::string mapPostFiltersConfigFilePath, bool computeProbDynamic, bool closeLoopFlag, bool walkingMode)
{
    stringvec v;
    read_directory(sourceBags, v);

    // std::copy(v.begin(), v.end(),std::ostream_iterator<std::string>(std::cout, "\n"));
    std::sort(v.begin(),v.end());
    //   std::cout<<"now sorted"<<std::endl;
    //std::copy(v.begin(), v.end(),std::ostream_iterator<std::string>(std::cout, "\n"));
    std::vector<std::string> filteredPath;
    for (unsigned int i=0; i<v.size(); i++)
    {
        // unsigned int dotPosition=	v[i].find_last_of('.');
        //if v[i][dotPosition]
        if (v[i].size()>4) {
            std::string w=v[i].substr(v[i].size() - 4);
            if (w==".bag"){
                filteredPath.push_back(v[i]);
            }
        }
    }
    //std::copy(filteredPath.begin(), filteredPath.end(),std::ostream_iterator<std::string>(std::cout, "\n"));
    // printf("Position of last dot in string: %i", v[0].find_last_of('.'));

    rosbag::Bag bag[filteredPath.size()];
    rosbag::View view1;
    // cout << "argggc "<<argc <<" argv "<<argv[1] <<endl;

    // bool whofirst1=false, whofirst2=false;
    std::cout <<"Loading Bags... "<<std::endl;
    for (unsigned int i=0; i<filteredPath.size(); i++){
        std::cout <<"Loading "<<i+1<< " out of "<<filteredPath.size()<<" bags"<<std::endl;
        std::string fullPath=sourceBags+"/"+filteredPath[i];
        std::cout <<fullPath<<std::endl;
        bag[i].open(fullPath, rosbag::bagmode::Read);
        view1.addQuery( bag[i], rosbag::TopicQuery(topics) );
    }
    std::cout <<"Bag loading complete"<<std::endl;
    std::cout <<"Parsing data..."<<std::endl;
    std::vector<std::vector<double>> vectGPS;

    Datacontainer gpsContainer;
    Datacontainer rtkContainer;
    Datacontainer imuContainer;
    pCloudcontainer pcContainer;
    TFcontainer tfContainer;

    gpsContainer.header={"timestamp", "latitude", "longitude", "altitude", "status"};
    imuContainer.header={"timestamp", "qx", "qy", "qz", "qw", "angular_vel_x", "angular_vel_y", "angular_vel_z", "linear_acc_x", "linear_acc_y", "linear_acc_z"};

    if (!topics[3].empty())
    {
        rtkContainer.header={"timestamp", "latitude", "longitude", "altitude", "status"};

    }
    ros::Time bag_begin_time = view1.getBeginTime();
    ros::Time bag_end_time = view1.getEndTime();
    int oldVal=999;
    foreach(rosbag::MessageInstance const m, view1)
    {
        float elapsedPercentage=100*((m.getTime().toSec()-bag_begin_time.toSec())/(bag_end_time.toSec()-bag_begin_time.toSec()) );
        elapsedPercentage=floor(elapsedPercentage);
        //         std::cout<<"----------------"<<std::endl;
        //        std::cout<<m.getTime().toSec()<<std::endl;

        if ( int(elapsedPercentage) % 10 == 0 && oldVal!= elapsedPercentage){
            std::cout<<elapsedPercentage<<"%"<<std::endl;
            oldVal=elapsedPercentage;
        }
        if (m.getTopic() == topics[0])
        {
            sensor_msgs::NavSatFix::ConstPtr GPS_INS = m.instantiate<sensor_msgs::NavSatFix>();
            if (GPS_INS != NULL){

                gpsContainer.frame_id=GPS_INS->header.frame_id;
                std::vector<double> tempvec;
                tempvec.push_back(GPS_INS->header.stamp.toSec());
                tempvec.push_back(GPS_INS->latitude);
                tempvec.push_back(GPS_INS->longitude);
                tempvec.push_back(GPS_INS->altitude);
                tempvec.push_back(GPS_INS->status.status);

                gpsContainer.vect.push_back(tempvec);
                /*        for (int i = 0; i < gpsContainer.vect.size(); i++)
                {
                    for (int j = 0; j < gpsContainer.vect[i].size(); j++)
                    {
                        std::cout << gpsContainer.vect[i][j]<<" ";
                    }
                    std::cout<<std::endl;
                }
*/
            }
        }
        if (m.getTopic() == topics[1])
        {
            sensor_msgs::PointCloud2::ConstPtr pc0 = m.instantiate<sensor_msgs::PointCloud2>();
            pcl::PCLPointCloud2::Ptr pc2 (new pcl::PCLPointCloud2 ());
            pcl_conversions::toPCL(*pc0,*pc2);

            // pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (pc2);
            sor.setMinimumPointsNumberPerVoxel(2);
            sor.setLeafSize (leafSize, leafSize, leafSize);
            sor.filter (*pc2);



            pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor2;
            sor2.setInputCloud (pc2);
            sor2.setMeanK (5);
            sor2.setStddevMulThresh (1.0);
            sor2.filter (*pc2);



            std::vector<int> indices;


            pcContainer.timestamp.push_back(pc0->header.stamp.toSec());
            pcContainer.frame_id=pc0->header.frame_id;

            //pcl::PCLPointCloud2 pcl_pc2;


            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::fromPCLPointCloud2(*pc2,*temp_cloud);
            pcl::removeNaNFromPointCloud(*temp_cloud, *temp_cloud, indices);


            pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> ne;
            ne.setInputCloud (temp_cloud);
            // Create an empty kdtree representation, and pass it to the normal estimation object.
            // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
            pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
            ne.setSearchMethod (tree);

            // Output datasets
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

            // Use all neighbors in a sphere of radius 3cm
            ne.setRadiusSearch (0.2);

            // Compute the features
            ne.compute (*cloud_normals);

            pc2=NULL;


            if (semantics)
            {
                pcl::PointCloud<pcl::PointXYZRGBL> temp2_cloud;
                pcl::copyPointCloud (*temp_cloud, temp2_cloud);


                pcContainer.XYZRGBL.push_back(temp2_cloud);

                for (size_t i = 0; i < temp_cloud->points.size(); i++) {
                    //  std::uint32_t rgb = *reinterpret_cast<int*>(&temp_cloud.points[i].rgb);
                    //    std::uint8_t r = (rgb >> 16) & 0x0000ff;
                    //    std::uint8_t g = (rgb >> 8)  & 0x0000ff;
                    //    std::uint8_t b = (rgb)       & 0x0000ff;
                    int r=int(temp2_cloud.points[i].r);
                    int g= int(temp2_cloud.points[i].g);
                    int b= int(temp2_cloud.points[i].b);
                    //       std::cout<<"current color0 is "<<r<<" "<<g<<" "<<b<< std::endl;
                    pcContainer.XYZRGBL.back().points[i].label = checkPointLabel(r,g,b);

                    //std::cout<<"IO STAT0 "<<checkPointLabel(r,g,b)<<std::endl;
                    //std::cout<<"IO STAT1"<<pcContainer.XYZRGBL[0].points[1].label<<std: :endl;
                }  //.cast<std::uint32_t>()
                //   std::cout<<pcContainer.XYZRGBL[0].getMatrixXfMap(7,8,0).row(6).cast<std::uint32_t>()<<std::endl;
                //  std::cout<<"IO label0 "<<pcContainer.XYZRGBL[0].points[0].label<<std::endl;
                // std::cout<<"IO label1 "<<pcContainer.XYZRGBL[0].points[1].label<<std::endl;
            }
            else
            {
                pcl::PointCloud<pcl::PointXYZRGBL> temp2_cloud;
                pcl::copyPointCloud (*temp_cloud, temp2_cloud);
                pcContainer.XYZRGBL.push_back(temp2_cloud);
            }
            temp_cloud->points.clear();
            temp_cloud->points.shrink_to_fit();
            pcContainer.normals.push_back(*cloud_normals);
            //    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            //    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

            //  pcContainer.XYZRGBL.x=;
            // pcContainer.Intensity=;

        }
        if (m.getTopic() == topics[2])
        {
            sensor_msgs::Imu::ConstPtr Imu = m.instantiate<sensor_msgs::Imu>();
            if (Imu != NULL){

                imuContainer.frame_id=Imu->header.frame_id;
                std::vector<double> tempvec;
                tempvec.push_back(Imu->header.stamp.toSec());
                tempvec.push_back(Imu->orientation.x);
                tempvec.push_back(Imu->orientation.y);
                tempvec.push_back(Imu->orientation.z);
                tempvec.push_back(Imu->orientation.w);
                tempvec.push_back(Imu->angular_velocity.x);
                tempvec.push_back(Imu->angular_velocity.y);
                tempvec.push_back(Imu->angular_velocity.z);
                tempvec.push_back(Imu->linear_acceleration.x);
                tempvec.push_back(Imu->linear_acceleration.y);
                tempvec.push_back(Imu->linear_acceleration.z);
                imuContainer.vect.push_back(tempvec);

            }
        }
        if (m.getTopic() == topics[3] &&  !topics[3].empty() )
        {
            sensor_msgs::NavSatFix::ConstPtr RTK = m.instantiate<sensor_msgs::NavSatFix>();
            if (RTK != NULL){

                rtkContainer.frame_id=RTK->header.frame_id;
                std::vector<double> tempvec;
                tempvec.push_back(RTK->header.stamp.toSec());
                tempvec.push_back(RTK->latitude);
                tempvec.push_back(RTK->longitude);
                tempvec.push_back(RTK->altitude);
                tempvec.push_back(RTK->status.status);

                rtkContainer.vect.push_back(tempvec);
                /*      for (int i = 0; i < rtkContainer.vect.size(); i++)
                {
                    for (int j = 0; j < rtkContainer.vect[i].size(); j++)
                    {
                        std::cout << rtkContainer.vect[i][j]<<" ";
                    }
                    std::cout<<std::endl;
                }
  */
            }
        }

        if (m.getTopic() == topics[4] &&  tfInit==false )
        {


            tf2_msgs::TFMessage::ConstPtr tf = m.instantiate<tf2_msgs::TFMessage>();
            if (tf != NULL){
                std::string frame_id=tf->transforms[0].header.frame_id;
                std::string child_frame_id=tf->transforms[0].child_frame_id;
                std::string bothFrames=frame_id+child_frame_id;
                if ( std::find(tfContainer.bothFrames.begin(), tfContainer.bothFrames.end(), bothFrames) ==  tfContainer.bothFrames.end()  ){
                    tfContainer.frame_id.push_back(frame_id);
                    tfContainer.child_frame_id.push_back(child_frame_id);
                    tfContainer.bothFrames.push_back(bothFrames);

                    std::vector<double> translation {( tf->transforms[0].transform.translation.x ),  ( tf->transforms[0].transform.translation.y ), ( tf->transforms[0].transform.translation.z )};

                    Eigen::Quaternion<double> rotation;
                    rotation.x()=( tf->transforms[0].transform.rotation.x );
                    rotation.y()=( tf->transforms[0].transform.rotation.y );
                    rotation.z()=( tf->transforms[0].transform.rotation.z );
                    rotation.w()=( tf->transforms[0].transform.rotation.w );
                    Eigen::Matrix3d rotM=rotation.toRotationMatrix();
                    Eigen::Matrix4d transform= Eigen::Matrix4d::Identity();
                    transform.block(0,0,3,3) = rotM;
                    transform(0,3)=translation[0]; transform(1,3)=translation[1]; transform(2,3)=translation[2];

                    //  std::cout<<"transform is" <<transform<<std::endl;
                    //    std::cout<<"tfContainer.transform is" <<transform<<std::endl;
                    tfContainer.transform.push_back(transform);  //todo EIGEN matrices
                    //    std::cout<<"passed " <<transform<<std::endl;
                    //  tfContainer.translation.push_back(rotation);

                }
                //  std::cout<<"------------------------------------- "<<std::endl;

                //         std::copy(tfContainer.frame_id.begin(), tfContainer.frame_id.end(),std::ostream_iterator<std::string>(std::cout, "\n"));
                //           std::cout<<"----------CHILDREN-------------- "<<std::endl;
                //      std::copy(tfContainer.child_frame_id.begin(), tfContainer.child_frame_id.end(),std::ostream_iterator<std::string>(std::cout, "\n"));
                //     std::cout<<"------------------------------------- "<<std::endl;

                /*      for (int i = 0; i < rtkContainer.vect.size(); i++)
                {
                    for (int j = 0; j < rtkContainer.vect[i].size(); j++)
                    {
                        std::cout << rtkContainer.vect[i][j]<<" ";
                    }
                    std::cout<<std::endl;
                }
  */
            }
        }


        //pointCloud accumulator with semantic condition TODO
        //log static tf
        //

    }


    std::cout <<"Calculating transforms..."<<std::endl;
    Eigen::Matrix4d imu2base=parseTFcontainer(tfContainer, imuContainer.frame_id, base_link);
    // std::cout<<"imu2base is \n"<<imu2base<<std::endl;
    Eigen::Matrix4d pc2base=parseTFcontainer(tfContainer, pcContainer.frame_id, base_link);
    // std::cout<<"pc2base is \n"<<pc2base<<std::endl;
    Eigen::Matrix4d gps2base=parseTFcontainer(tfContainer, gpsContainer.frame_id, base_link);
    //  std::cout<<"gps2base is \n"<<gps2base<<std::endl;

    //-----------------------UTM CONVERSION----------------------------

    Datacontainer gpsUTMContainer=gpsContainer;
    double humanSpeed=2;

    for (unsigned int i=0; i< gpsUTMContainer.vect.size(); i++){
        geographic_msgs::GeoPoint geo_pt;
        geo_pt.latitude = gpsUTMContainer.vect[i][1];
        geo_pt.longitude= gpsUTMContainer.vect[i][2];
        geo_pt.altitude = gpsUTMContainer.vect[i][3];
        geodesy::UTMPoint utm_pt(geo_pt);
        gpsUTMContainer.vect[i][1]=utm_pt.northing;
        gpsUTMContainer.vect[i][2]=utm_pt.easting;
        gpsUTMContainer.vect[i][3]=- utm_pt.altitude; //UTM-NED

        if (walkingMode&&i>0){   //cap velocity to human walking velocity with some flexibility
            double elapsedT=gpsUTMContainer.vect[i][0]-gpsUTMContainer.vect[i-1][0];
            double maxDisp=elapsedT*humanSpeed;
            if (    (gpsUTMContainer.vect[i][1]-gpsUTMContainer.vect[i-1][1])>maxDisp   ){gpsUTMContainer.vect[i][1]=gpsUTMContainer.vect[i-1][1]+maxDisp;}
            if (    (gpsUTMContainer.vect[i][1]-gpsUTMContainer.vect[i-1][1])<-maxDisp   ){gpsUTMContainer.vect[i][1]=gpsUTMContainer.vect[i-1][1]-maxDisp;}

            if (    (gpsUTMContainer.vect[i][2]-gpsUTMContainer.vect[i-1][2])>maxDisp   ){gpsUTMContainer.vect[i][2]=gpsUTMContainer.vect[i-1][2]+maxDisp;}
            if (    (gpsUTMContainer.vect[i][2]-gpsUTMContainer.vect[i-1][2])<-maxDisp   ){gpsUTMContainer.vect[i][2]=gpsUTMContainer.vect[i-1][2]-maxDisp;}

            if (    (gpsUTMContainer.vect[i][3]-gpsUTMContainer.vect[i-1][3])>maxDisp/2   ){gpsUTMContainer.vect[i][3]=gpsUTMContainer.vect[i-1][3]+maxDisp/2;}
            if (    (gpsUTMContainer.vect[i][3]-gpsUTMContainer.vect[i-1][3])<-maxDisp/2   ){gpsUTMContainer.vect[i][3]=gpsUTMContainer.vect[i-1][3]-maxDisp/2;}

        }
        if (mapNumber==0)
        {

            if (i==0){
                std::vector<double> tempVec {gpsUTMContainer.vect[i][1],gpsUTMContainer.vect[i][2],0,0,0};

                travelledDistance.push_back(tempVec);
            }
            else
            {
                double distTemp=sqrt(pow(gpsUTMContainer.vect[i][1]-gpsUTMContainer.vect[i-1][1],2)+pow(gpsUTMContainer.vect[i][2]-gpsUTMContainer.vect[i-1][2],2) ); //+ pow(gpsUTMContainer.vect[i][3]-gpsUTMContainer.vect[i-1][3],2));

                double distTemp0=sqrt(pow(gpsUTMContainer.vect[i][1]-gpsUTMContainer.vect[0][1],2)+pow(gpsUTMContainer.vect[i][2]-gpsUTMContainer.vect[0][2],2)  );  //+ pow(gpsUTMContainer.vect[i][3]-gpsUTMContainer.vect[0][3],2));
                double cTime=gpsUTMContainer.vect[i][0];
                std::vector<double> tempVec {gpsUTMContainer.vect[i][1],gpsUTMContainer.vect[i][2], distTemp, distTemp0, cTime};
                travelledDistance.push_back(tempVec);

            }
        }

    }
    double totalDist=0;

    int localCounter=0;
    int tempCounter=0;
    bool debug=false;
    double debugAt=1700;
    //generate list of lat lon with distances
    //then iterate on that list while changing file names
    if (autoGenerateMaps){
        bool firstLoopFlag=false;
        if (closeLoopFlag){
            firstLoopFlag=true;

        }

        double elapsedTemp=999999;
        std::string selection;
        double prevStamp=0;
        for (int i=0; i<travelledDistance.size(); i++){
            totalDist=travelledDistance[i][2]+totalDist;

            if (debug){

                if (totalDist<debugAt){continue;}
            }

            if (!selection.empty()){
                if (selection=="exit"){

                    continue;
                }

            }
            //  if (tempCounter>4) {continue;}
            elapsedTemp=elapsedTemp+travelledDistance[i][2];

            if ( elapsedTemp>autoDist   ) {   // && travelledDistance[i][3] > 10//query at least 10 meters from the starting point

                elapsedTemp=0;
            }
            else
            {
                //  if (travelledDistance[i][3] < autoDist && totalDist>700){break;}
                continue;
            }

            std::vector<double> UTM_ref {travelledDistance[i][0], travelledDistance[i][1]};
            double timeRef=travelledDistance[i][4];

            //-----------------------APPLYING LOCAL CONSTRAINTS----------------------------
            std::cout <<"Applying local maps..."<<std::endl;


            unsigned int loop_count=0;
            while(true){

                std::vector<std::vector<double>> timeRange;

                timeRange=localDataFilterAuto(gpsUTMContainer, UTM_ref, radius, prevStamp);

                if (firstLoopFlag&&timeRange.size()>1){
                    if (timeRange[1].size()>50)  //5 seconds
                    {timeRange[0]=timeRange[1];}

                }
                else{
                    if (mapNumber==0){
                        double lowestTimeDiff=99999;
                        unsigned int chosenIdx=0;
                        for (unsigned int j=0; j<timeRange.size();j++){

                            double timeDiff=abs(timeRef-timeRange[j][0]);

                            if (timeDiff<lowestTimeDiff){
                                lowestTimeDiff=timeDiff;
                                chosenIdx=j;


                            }

                        }

                        timeRange[0]=timeRange[chosenIdx];
                    }
                    else
                    {
                        std::cout<<"timeRange size is "<<timeRange.size()<<std::endl;
                        std::cout.precision(20);
                        std::cout<<std::setprecision(20);
                        for  (int mm=0; mm<timeRange.size(); mm++){
                            std::cout<<"timeRange["<<mm<<"] size is "<<timeRange[mm].size()<<std::endl;
                            std::cout<<"timeRange["<<mm<<"] starts at "<<timeRange[mm][0]<<std::endl;
                            std::cout<<"timeRange["<<mm<<"] ends at "<<timeRange[mm].back()<<std::endl;
                        }


                        /*          if  (totalDist>200 && timeRange.size()>1){


                            if (timeRange[1].size()>30)   //at least 3 seconds of recording to create a map (50 elements at 10 hz)
                            {
                                timeRange[0]=timeRange[1];
                            }

                        }
*/
                    }
                    std::cout<<"counter is "<<tempCounter<<std::endl;
                    std::cout<<std::endl;
                    tempCounter++;
                    prevStamp=timeRange[0][0];

                }
           //     break;

                Datacontainer rtkUTMContainer;
                // Datacontainer rtkLocalUTMContainer;
                if (!topics[3].empty()){
                    rtkUTMContainer=rtkContainer;
                    for (unsigned int k=0; k< rtkUTMContainer.vect.size(); k++){
                        geographic_msgs::GeoPoint geo_pt;
                        geo_pt.latitude = rtkUTMContainer.vect[k][1];
                        geo_pt.longitude= rtkUTMContainer.vect[k][2];
                        geo_pt.altitude = rtkUTMContainer.vect[k][3];
                        geodesy::UTMPoint utm_pt(geo_pt);
                        rtkUTMContainer.vect[k][1]=utm_pt.northing;
                        rtkUTMContainer.vect[k][2]=utm_pt.easting;
                        rtkUTMContainer.vect[k][3]=- utm_pt.altitude; //UTM-NED
                    }
                    // timeDataFilter(rtkUTMContainer, timeRange, rtkLocalUTMContainer);
                }
                Datacontainer imuLocalContainer, gpsLocalUTMContainer, rtkLocalUTMContainer;
                pCloudcontainer pcLocalContainer;

                timeDataFilter(imuContainer, timeRange, imuLocalContainer);
                timeDataFilter(pcContainer, timeRange, pcLocalContainer);
                timeDataFilter(gpsUTMContainer, timeRange, gpsLocalUTMContainer);
                if (!topics[3].empty()){
                    timeDataFilter(rtkUTMContainer, timeRange, rtkLocalUTMContainer);
                }


                //--------------------------------INTERPOLATION------------------------------------------
                std::cout <<"Data synchronization and Interpolation..."<<std::endl;

                Datacontainer imuLocalContainerInterp, gpsLocalUTMContainerInterp, gpsUTMContainerInterp, imuContainerInterp, rtkLocalUTMContainerInterp, rtkUTMContainerInterp;
                pCloudcontainer pcLocalContainerInterp, pcContainerInterp;

                interpolateRefRange(imuLocalContainer ,gpsLocalUTMContainer, gpsLocalUTMContainerInterp);    //refList (Datacontainer Type), Input, Output
                interpolateRefRange(imuContainer ,gpsUTMContainer, gpsUTMContainerInterp);

                std::cout<<gpsLocalUTMContainerInterp.vect.size()<< " "<<gpsUTMContainerInterp.vect.size()<< " "<< imuLocalContainer.vect.size()<< " "<<imuContainer.vect.size()<< std::endl;

                // std::cout<<gpsLocalUTMContainerInterp.vect<< " "<<gpsUTMContainerInterp.vect.size()<< " "<< imuLocalContainer.vect<< " "<<imuContainer.vect<< std::endl;

                //  interpolateRefRange(imuLocalContainer ,pcLocalContainer, pcLocalContainerInterp);    //refList (Datacontainer Type), Input, Output
                // interpolateRefRange(imuContainer ,pcContainer, pcContainerInterp);



                // interpolateRefRange(imuLocalContainer ,imuLocalContainer, imuLocalContainerInterp);    //refList (Datacontainer Type), Input, Output  IMU inerpolation
                // interpolateRefRange(imuContainer ,imuContainer, imuContainerInterp);


                if (!topics[3].empty()){


                    interpolateRefRange(imuLocalContainer ,rtkLocalUTMContainer, rtkLocalUTMContainerInterp);    //refList (Datacontainer Type), Input, Output
                    std::cout<<std::endl;
                    interpolateRefRange(imuContainer ,rtkUTMContainer, rtkUTMContainerInterp);

                }

                //  for (unsigned int i=0; i< rtkUTMContainerInterp.vect.size() ; i++ )
                //  {
                //      for (unsigned int j=0; j< rtkUTMContainerInterp.vect[i].size() ; j++ )
                //     {


                //  std::cout<<rtkUTMContainerInterp.vect[i][j]<< " ";

                //   }
                //  std::cout<<std::endl;
                // }
                //std::cout<<"-----size match start------"<<std::endl;
                //   std::cout<<rtkLocalUTMContainerInterp.vect.size()<<" "<<rtkUTMContainerInterp.vect.size()<<" "<< imuLocalContainer.vect.size()<<" "<<imuContainer.vect.size()<< std::endl;

                // std::cout<<rtkLocalUTMContainer.vect.size()<<" "<<rtkUTMContainer.vect.size()<<std::endl;
                //std::cout<<"-----size match end ------"<<std::endl;

                //---------------------------------------------MATCHING------------------------------------------------
                Datacontainer imuContainerMatched, gpsUTMContainerMatched, rtkUTMContainerMatched;
                Datacontainer imuLocalContainerMatched, gpsLocalUTMContainerMatched, rtkLocalUTMContainerMatched;

                if (!topics[3].empty()){

                    matchDataToPointCloudRate(pcContainer.timestamp , rtkUTMContainerInterp, rtkUTMContainerMatched);
                    matchDataToPointCloudRate(pcLocalContainer.timestamp , rtkLocalUTMContainerInterp, rtkLocalUTMContainerMatched);
                }

                matchDataToPointCloudRate(pcContainer.timestamp , gpsUTMContainerInterp, gpsUTMContainerMatched);
                matchDataToPointCloudRate(pcContainer.timestamp , imuContainer, imuContainerMatched);


                matchDataToPointCloudRate(pcLocalContainer.timestamp , gpsLocalUTMContainerInterp, gpsLocalUTMContainerMatched);
                matchDataToPointCloudRate(pcLocalContainer.timestamp , imuLocalContainer, imuLocalContainerMatched);



                std::cout<<"-----MATCHING SECTION------"<<std::endl;
                std::cout<<pcContainer.timestamp.size()<<" "<<rtkUTMContainerMatched.vect.size()<<" "<< imuContainerMatched.vect.size()<<" "<<gpsUTMContainerMatched.vect.size()<< std::endl;

                std::cout<<pcLocalContainer.timestamp.size()<<" "<<rtkLocalUTMContainerMatched.vect.size()<<" "<< imuLocalContainerMatched.vect.size()<<" "<<gpsLocalUTMContainerMatched.vect.size()<< std::endl;
                std::cout<<"-----MATCHING SECTION ENDS------"<<std::endl;

                //---------------------------------------------REGISTRATION-----------------------------------------------
                // lat,lon, radius TODO filter coordinates

                /* for (unsigned int i=0; i<imuContainerMatched.vect.size(); i++)
            for (unsigned int j=0; j<imuContainerMatched.vect[i].size(); j++)
            {
                {
                    std::cout<<imuContainerMatched.vect[i][j]<<" ";

                }
                std::cout<<std::endl;
            }
    */
                if (rMethod=="gps")
                {

                    GPS* Gps =new GPS();

                    std::cout<< "What map do you want to save?  Maps will be created for every survey and stored in the path specified in config.yaml"<<std::endl;

                    std::cout<<std::endl;

                    std::cout<< "THIS IS SURVEY #"<<mapNumber<<std::endl;

                    std::cout<<std::endl;

                    std::cout<<"choose from: gps_local rtk_local icp_local semantic_icp all exit"<<std::endl;

                    std::cout<<std::endl;

                    std::cout<<"REMARK: When finished, type exit to process next survey"<<std::endl;


                    while (selection.empty() ||( selection!="exit" && selection !="semantic_icp") ){
                        std::cout<<"Selection:"<<std::endl;
                        std::cin >> selection;
                    }


                    std::string exportName=selection + "_" + std::to_string(localCounter) ;
                    if(firstLoopFlag){exportName=selection + "_lc"; localCounter--;}
                    localCounter++;
                    std::cout<<std::endl;

                    if (selection=="gps_local" || selection=="all" ){
                        Gps->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, gpsLocalUTMContainerMatched, exportName, leafSize );
                    }
                    if (selection=="rtk_local" || selection=="all" ){
                        Gps->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, rtkLocalUTMContainerMatched, exportName, leafSize );
                    }
                    if (selection=="icp_local" || selection=="all" ){
                        ICP* Icp =new ICP();
                        Icp->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, gpsLocalUTMContainerMatched, exportName, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, false);
                        delete Icp;
                    }
                    if (selection=="semantic_icp" || selection=="all" ){
                        ICP* Icp =new ICP();
                        Icp->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, gpsLocalUTMContainerMatched, exportName, leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, semantics);
                        delete Icp;

                    }
                    if  (selection=="exit" || selection=="all" ){break;}




                    delete Gps;

                }
                if (loop_count==1 || firstLoopFlag==false){break;}
                firstLoopFlag=false;
                loop_count++;
            }

        }

    }



    else
    {

        geographic_msgs::GeoPoint geo_pt2;
        geo_pt2.latitude =lat; geo_pt2.longitude= lon; //geo_pt2.altitude = ;
        geodesy::UTMPoint utm_pt2(geo_pt2);
        std::vector<double> UTM_ref {utm_pt2.northing, utm_pt2.easting};
        //   std::cout<<UTM_ref[0]<<" "<<UTM_ref[1]<<std::endl;

        //-----------------------APPLYING LOCAL CONSTRAINTS----------------------------
        std::cout <<"Applying local maps..."<<std::endl;

        std::vector<std::vector<double>> timeRange;
        timeRange=localDataFilter(gpsUTMContainer, UTM_ref, radius);
        Datacontainer rtkUTMContainer;
        // Datacontainer rtkLocalUTMContainer;
        if (!topics[3].empty()){
            rtkUTMContainer=rtkContainer;
            for (unsigned int i=0; i< rtkUTMContainer.vect.size(); i++){
                geographic_msgs::GeoPoint geo_pt;
                geo_pt.latitude = rtkUTMContainer.vect[i][1];
                geo_pt.longitude= rtkUTMContainer.vect[i][2];
                geo_pt.altitude = rtkUTMContainer.vect[i][3];
                geodesy::UTMPoint utm_pt(geo_pt);
                rtkUTMContainer.vect[i][1]=utm_pt.northing;
                rtkUTMContainer.vect[i][2]=utm_pt.easting;
                rtkUTMContainer.vect[i][3]=- utm_pt.altitude; //UTM-NED
            }
            // timeDataFilter(rtkUTMContainer, timeRange, rtkLocalUTMContainer);
        }
        Datacontainer imuLocalContainer, gpsLocalUTMContainer, rtkLocalUTMContainer;
        pCloudcontainer pcLocalContainer;

        timeDataFilter(imuContainer, timeRange, imuLocalContainer);
        timeDataFilter(pcContainer, timeRange, pcLocalContainer);
        timeDataFilter(gpsUTMContainer, timeRange, gpsLocalUTMContainer);
        if (!topics[3].empty()){
            timeDataFilter(rtkUTMContainer, timeRange, rtkLocalUTMContainer);
        }


        //--------------------------------INTERPOLATION------------------------------------------
        std::cout <<"Data synchronization and Interpolation..."<<std::endl;

        Datacontainer imuLocalContainerInterp, gpsLocalUTMContainerInterp, gpsUTMContainerInterp, imuContainerInterp, rtkLocalUTMContainerInterp, rtkUTMContainerInterp;
        pCloudcontainer pcLocalContainerInterp, pcContainerInterp;

        interpolateRefRange(imuLocalContainer ,gpsLocalUTMContainer, gpsLocalUTMContainerInterp);    //refList (Datacontainer Type), Input, Output
        interpolateRefRange(imuContainer ,gpsUTMContainer, gpsUTMContainerInterp);

        std::cout<<gpsLocalUTMContainerInterp.vect.size()<< " "<<gpsUTMContainerInterp.vect.size()<< " "<< imuLocalContainer.vect.size()<< " "<<imuContainer.vect.size()<< std::endl;

        // std::cout<<gpsLocalUTMContainerInterp.vect<< " "<<gpsUTMContainerInterp.vect.size()<< " "<< imuLocalContainer.vect<< " "<<imuContainer.vect<< std::endl;

        //  interpolateRefRange(imuLocalContainer ,pcLocalContainer, pcLocalContainerInterp);    //refList (Datacontainer Type), Input, Output
        // interpolateRefRange(imuContainer ,pcContainer, pcContainerInterp);



        // interpolateRefRange(imuLocalContainer ,imuLocalContainer, imuLocalContainerInterp);    //refList (Datacontainer Type), Input, Output  IMU inerpolation
        // interpolateRefRange(imuContainer ,imuContainer, imuContainerInterp);


        if (!topics[3].empty()){


            interpolateRefRange(imuLocalContainer ,rtkLocalUTMContainer, rtkLocalUTMContainerInterp);    //refList (Datacontainer Type), Input, Output
            std::cout<<std::endl;
            interpolateRefRange(imuContainer ,rtkUTMContainer, rtkUTMContainerInterp);

        }

        //  for (unsigned int i=0; i< rtkUTMContainerInterp.vect.size() ; i++ )
        //  {
        //      for (unsigned int j=0; j< rtkUTMContainerInterp.vect[i].size() ; j++ )
        //     {


        //  std::cout<<rtkUTMContainerInterp.vect[i][j]<< " ";

        //   }
        //  std::cout<<std::endl;
        // }
        //std::cout<<"-----size match start------"<<std::endl;
        //   std::cout<<rtkLocalUTMContainerInterp.vect.size()<<" "<<rtkUTMContainerInterp.vect.size()<<" "<< imuLocalContainer.vect.size()<<" "<<imuContainer.vect.size()<< std::endl;

        // std::cout<<rtkLocalUTMContainer.vect.size()<<" "<<rtkUTMContainer.vect.size()<<std::endl;
        //std::cout<<"-----size match end ------"<<std::endl;

        //---------------------------------------------MATCHING------------------------------------------------
        Datacontainer imuContainerMatched, gpsUTMContainerMatched, rtkUTMContainerMatched;
        Datacontainer imuLocalContainerMatched, gpsLocalUTMContainerMatched, rtkLocalUTMContainerMatched;

        if (!topics[3].empty()){

            matchDataToPointCloudRate(pcContainer.timestamp , rtkUTMContainerInterp, rtkUTMContainerMatched);
            matchDataToPointCloudRate(pcLocalContainer.timestamp , rtkLocalUTMContainerInterp, rtkLocalUTMContainerMatched);
        }

        matchDataToPointCloudRate(pcContainer.timestamp , gpsUTMContainerInterp, gpsUTMContainerMatched);
        matchDataToPointCloudRate(pcContainer.timestamp , imuContainer, imuContainerMatched);


        matchDataToPointCloudRate(pcLocalContainer.timestamp , gpsLocalUTMContainerInterp, gpsLocalUTMContainerMatched);
        matchDataToPointCloudRate(pcLocalContainer.timestamp , imuLocalContainer, imuLocalContainerMatched);



        std::cout<<"-----MATCHING SECTION------"<<std::endl;
        std::cout<<pcContainer.timestamp.size()<<" "<<rtkUTMContainerMatched.vect.size()<<" "<< imuContainerMatched.vect.size()<<" "<<gpsUTMContainerMatched.vect.size()<< std::endl;

        std::cout<<pcLocalContainer.timestamp.size()<<" "<<rtkLocalUTMContainerMatched.vect.size()<<" "<< imuLocalContainerMatched.vect.size()<<" "<<gpsLocalUTMContainerMatched.vect.size()<< std::endl;
        std::cout<<"-----MATCHING SECTION ENDS------"<<std::endl;

        //---------------------------------------------REGISTRATION-----------------------------------------------
        // lat,lon, radius TODO filter coordinates

        /* for (unsigned int i=0; i<imuContainerMatched.vect.size(); i++)
            for (unsigned int j=0; j<imuContainerMatched.vect[i].size(); j++)
            {
                {
                    std::cout<<imuContainerMatched.vect[i][j]<<" ";

                }
                std::cout<<std::endl;
            }
    */
        if (rMethod=="gps")
        {

            GPS* Gps =new GPS();

            while (true)
            {

                std::string selection;
                std::cout<< "What map do you want to save?  Maps will be created for every survey and stored in the path specified in config.yaml"<<std::endl;

                std::cout<<std::endl;

                std::cout<< "THIS IS SURVEY #"<<mapNumber<<std::endl;

                std::cout<<std::endl;

                std::cout<<"choose from: gps_local gps_global rtk_local rtk_global icp_local semantic_icp all exit"<<std::endl;

                std::cout<<std::endl;

                std::cout<<"REMARK: When finished, type exit to process next survey"<<std::endl;

                std::cout<<"Selection:"<<std::endl;
                std::cin >> selection;

                std::cout<<std::endl;

                if  (selection=="gps_global" || selection=="all" ){
                    Gps->createMap(currentPath, imu2base, pc2base, gps2base, imuContainerMatched, pcContainer, gpsUTMContainerMatched, "gps_global", 2*leafSize );
                }

                if (selection=="gps_local" || selection=="all" ){
                    Gps->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, gpsLocalUTMContainerMatched, "gps_local", leafSize );
                }

                if  (selection=="rtk_global" || selection=="all" ){
                    Gps->createMap(currentPath, imu2base, pc2base, gps2base, imuContainerMatched, pcContainer, rtkUTMContainerMatched, "rtk_global", 2*leafSize );
                }

                if (selection=="rtk_local" || selection=="all" ){
                    Gps->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, rtkLocalUTMContainerMatched, "rtk_local", leafSize );
                }

                if (selection=="icp_local" || selection=="all" ){
                    ICP* Icp =new ICP();
                    Icp->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, gpsLocalUTMContainerMatched, "icp_local", leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, false);
                    delete Icp;
                }



                if (selection=="semantic_icp" || selection=="all" ){

                    ICP* Icp =new ICP();
                    Icp->createMap(currentPath, imu2base, pc2base, gps2base, imuLocalContainerMatched, pcLocalContainer, gpsLocalUTMContainerMatched, "icp_local_semantics", leafSize, icpConfigFilePath, inputFiltersConfigFilePath, mapPostFiltersConfigFilePath, computeProbDynamic, semantics);
                    delete Icp;

                }
                if  (selection=="exit" || selection=="all" ){break;}


            }

            delete Gps;

        }



    }
}
}
