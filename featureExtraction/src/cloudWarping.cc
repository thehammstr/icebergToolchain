#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <math.h>

#define PI 3.14159265
#define VIEW_SPINNING_CLOUD 1
#define SAVE_IMAGES 0
//#include "../Generic/CSVRow.h"
#include "CSVRow.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "NodeDefinitions.h"
// PCL stuff
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/console/parse.h>
#include "cloudManipulation.h"
#include <pcl/keypoints/harris_3d.h>
// MACROS

DEFINE_string(input, "", "Input File name");


using namespace std;

int main(int argc, char** argv)
{
/*-------------------------------------------------*/
/*----------- Load in Data from CSV ---------------*/
/*-------------------------------------------------*/
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    if (FLAGS_input.empty()) {
	LOG(ERROR) << "Usage: ./extractFeatures... --input=DATA.CSV";
    	return 1;
    }
    std::ifstream       file(FLAGS_input.c_str());
    CSVRow              row;
    Trajectory path;
    path.plotDuration = 5.;
    //std::vector<PoseNode> poses;
    std::vector<FeatureNode> features;
    std::vector<int> featureIndices;
    // Logging stuff
    int Nobs = 0;
    file >> row;
	/*for (int jj=0;jj<row.size();++jj){
        std::cout << row[jj] << "\t";
	}
	cout << endl;*/
    int poseIdx = 0;
    while(file >> row)
    {	
	PoseNode pose(row);
        // tag each measurement with its parent pose
        for (int i1 = 0; i1<pose.measurements.size();i1++){
            pose.measurements[i1].poseIdx = poseIdx;
        }
        poseIdx++;
	path.poses.push_back(pose);
	if (pose.measurements.size() > 0){
		Nobs = pose.measurements.back().featureIndex;
	}
    }
    	std::cout << Nobs << " observations." << std::endl;
	std::cout << path.poses.size() << " poses." << std::endl;
// update path with initial guess of bias
	//path.updateWithConstBias(-.00012);
	//path.addConstBias(-.00002); // this is how you simulate that the ground is moving.
/*-------------------------------------------------*/
/*----------- Now do something with the data-------*/
/*-------------------------------------------------*/
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3;
   int idx1 = 2500;
   int idx2 = 4500;
   double drift = .001;
   cloud1 = path.ExtractSubcloud(idx1,idx2);
   path.addConstBias(-drift);
   cloud2 = path.ExtractSubcloud(idx1,idx2);
   path.addConstBias(2.*drift);
   cloud3 = path.ExtractSubcloud(idx1,idx2);

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud1 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud3 (new pcl::PointCloud<pcl::PointXYZRGB>);

   for (int ii = 0; ii<cloud1->points.size(); ii++){
     pcl::PointXYZRGB point1;
     point1.x = cloud1->points[ii].x;
     point1.y = cloud1->points[ii].y;
     point1.z = cloud1->points[ii].z;
     point1.r = 255;
     point1.g = 0;
     point1.b = 0;
     colorCloud1->points.push_back(point1);
     pcl::PointXYZRGB point2;
     point2.x = cloud2->points[ii].x;
     point2.y = cloud2->points[ii].y;
     point2.z = cloud2->points[ii].z;
     point2.r = 0;
     point2.g = 255;
     point2.b = 0;
     colorCloud2->points.push_back(point2);
     pcl::PointXYZRGB point3;
     point3.x = cloud3->points[ii].x;
     point3.y = cloud3->points[ii].y;
     point3.z = cloud3->points[ii].z;
     point3.r = 0;
     point3.g = 0;
     point3.b = 255;
     colorCloud3->points.push_back(point3);

   } 
  

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(colorCloud1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(colorCloud2);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3(colorCloud3);
  viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud1, rgb1,"subcloud1");
  viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud2, rgb2,"subcloud2");
  viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud3, rgb3,"subcloud3");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "subcloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "subcloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "subcloud3");
  viewer->addCoordinateSystem(10.0);
  viewer->setCameraPosition(-190.,200.,-42.,0.,-40.,0.,0.,0.,-1.);
  //viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
 
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //pcl::io::savePCDFileASCII("fpfh.pcl",*fpfhs);
    //pcl::io::savePCDFileASCII("subcloudWithNormals.pcd",*cloudWithNormal);
}

