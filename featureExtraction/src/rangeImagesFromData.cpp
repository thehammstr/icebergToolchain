#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include "CSVRow.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "NodeDefinitions.h"
// PCL stuff
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Other
#include "cloudManipulation.h"
// MACROS for command line flags
DEFINE_string(input, "", "Input File name");
DEFINE_string(imWidth, "", "image width in meters");
DEFINE_string(spacing, "", "Image spacing");
DEFINE_string(startPose, "","pose to start on");
DEFINE_string(endPose,"","pose to end on");
using namespace std;

int main(int argc, char** argv){

/* load data based on user input */

    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    if (FLAGS_input.empty()) {
	LOG(ERROR) << "Usage: ./ --input=DATA.CSV \n --imWidth=[width in meters (integers only, please)] \n --spacing=[percent overlap * 100 (again, integer)] \n --startPose=[default: 0] --endPose=[default: big number]";
    	return 1;
    }
    std::ifstream       file(FLAGS_input.c_str());
    CSVRow              row;
    Trajectory path;
    // Logging stuff
    int Nobs = 0;
    int Nposes = 0;
    int NposesToUse = 230000;
    int startingPose = 0;
    if (!FLAGS_endPose.empty()){
        NposesToUse = atoi(FLAGS_endPose.c_str());
    } 
    if (!FLAGS_startPose.empty()){
        startingPose = atoi(FLAGS_startPose.c_str());
        if (NposesToUse - startingPose < 0){
           std::cout<< "Try again...idiot."<<std::endl;
           return 1;
        }
    }
    // burn header
    file >> row;
	/*for (int jj=0;jj<row.size();++jj){
        std::cout << row[jj] << "\t";
	}
	cout << endl;*/
    while(file >> row)
    {	Nposes++;
        if (Nposes < NposesToUse){
	   PoseNode pose(row);
	   path.poses.push_back(pose);
	   //cout<< pose.inputs[0] << " " << pose.inputs[1] << endl;
	   if (pose.measurements.size() > 0){
	   	Nobs = pose.measurements.back().featureIndex;
	   }
        }
    }
    	std::cout << Nobs << " observations." << std::endl;
	std::cout << path.poses.size() << " poses." << std::endl;
   
/* parse map size arguments*/
   // width of map
   double mapWidth = 150.; // default 150 m map width
   if (!FLAGS_imWidth.empty()){
      mapWidth = (double)atoi(FLAGS_imWidth.c_str());
   }
   double halfWidth = mapWidth/2.;
   // calculate overlap percentage
   double overlap = .5; // default to 50% overlap
   if (!FLAGS_spacing.empty()){
      overlap = .01*(double)(atoi(FLAGS_spacing.c_str()));
      if (overlap > .75)
            overlap = .9;
      if (overlap < 0.)
            overlap = 0.;
   }
   std::cout<< "map width: " << mapWidth << ", spacing: "<< overlap*100. << "%"<<std::endl;

/* loop through data*/
   Eigen::Vector3d xLast(path.poses[startingPose].xEst(),path.poses[startingPose].yEst(),path.poses[startingPose].zEst());
   for (int iT = startingPose; iT<path.poses.size(); iT++){
      // current position
      Eigen::Vector3d xCurrent(path.poses[iT].xEst(),path.poses[iT].yEst(),path.poses[iT].zEst());
      // change since last reference point
      Eigen::Vector3d dX;
      dX = xCurrent - xLast;
      if (dX.norm() < mapWidth*(1. - overlap)){
         continue;
      }
      // Else, time to process a new submap
      // Guess at a timestamp map halfwidth based on speed, map size, and time
      int timeHalfWidth = 2.*mapWidth/path.poses[iT].DVL[0]*.33; // 3 Hz data
      int firstIndex = max(iT - timeHalfWidth,0);
      int lastIndex = min(iT + timeHalfWidth, (int)path.poses.size());
      // okay, now extract subcloud at those indices
      pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZ>);
      subcloud = path.ExtractSubcloud(firstIndex,lastIndex);
      // TODO: window the extracted subcloud spatially, instead of just temporally

      // calculate normals
      float scaleRadius = 1.;
      pcl::PointCloud<pcl::Normal>::Ptr subcloudnormals = getNormals(subcloud,scaleRadius);
      //pcl::PointCloud<pcl::PointNormal>::Ptr subcloudWithNormals = addNormalsToCloud(subcloud,subcloudnormals);
      // view direction
      pcl::Normal viewDir = avgNormal(subcloudnormals);
      cv::Mat rangeImage = imageFromCloudInDirection(subcloud,viewDir,.5,.02);
      cv::imshow("image1", rangeImage);
      cv::waitKey(50);
      //cv::blur(rangeImage,rangeImage,cv::Size(11,11));
      char filename[50];
      snprintf(filename, sizeof(filename),"../data/rangeImages/rangeImage_%d_%d.png",iT,(int)mapWidth);
      cv::imwrite(filename,rangeImage);
      xLast = xCurrent;
   }
}
