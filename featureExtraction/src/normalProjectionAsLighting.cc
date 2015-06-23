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
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/console/parse.h>
#include "cloudManipulation.h"
#include <pcl/keypoints/harris_3d.h>
#include <pcl/range_image/range_image.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// MACROS

DEFINE_string(input, "", "Input File name");


using namespace std;
using namespace cv;
#if(1)


static int crossCheckMatching( Ptr<DescriptorMatcher>& descriptorMatcher,
                         const Mat& descriptors1, const Mat& descriptors2,
                         vector<DMatch>& filteredMatches12, int knn=1 )
{
    filteredMatches12.clear();
    vector<vector<DMatch> > matches12, matches21;
    descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
    descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );
    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    filteredMatches12.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
  return 1;
}






int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/secondPass.pcd", *cloud1) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

 std::cout << "Loaded "
            << cloud1->width * cloud1->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (int ii = 0; ii<cloud1->points.size(); ii++){
     cloud1->points[ii].z = - cloud1->points[ii].z;
  } 
 float scaleRadius = 1.;
  pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals(cloud1,scaleRadius);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals1 = addNormalsToCloud(cloud1,normals1);
        // view
        //normalsVis(cloud1,normals1); 
 cv::namedWindow("image1", CV_WINDOW_AUTOSIZE);
 pcl::Normal viewDir1 = avgNormal(normals1);
 Eigen::Vector3f nominalNormal1(viewDir1.normal_x,viewDir1.normal_y,viewDir1.normal_z);
  double maxDev1 = 0.;
  double pitchMax = 0.;
  double rollMax = 0.;
  pcl::Normal bestNorm1;
  float pitch[3] = {-.28, 0., .28};
  float roll[3] = {-.28, 0., .28};
  std::vector<cv::Mat> images;
  for (int ip = 0; ip < 3; ip++){ //float pitch = -.34; pitch < .34; pitch+= .32){
  //for (float pitch = -.15; pitch < .15; pitch+= .05){
    for (int ir = 0; ir<3 ; ir++){//float roll = -.34; roll < .34; roll += .32){
    //for (float roll = -.15; roll < .15; roll += .05){
       // rotate nominal normal
       Eigen::Matrix3f transform_1;
       transform_1 = Eigen::AngleAxisf(pitch[ip],Eigen::Vector3f::UnitX())
                                       * Eigen::AngleAxisf(roll[ir],Eigen::Vector3f::UnitY());
       Eigen::Vector3f newNormal1 =transform_1*nominalNormal1;
       pcl::Normal viewDirAug1(newNormal1(0),newNormal1(1),newNormal1(2));
       cv::Mat rangeImage1 = imageFromCloudInDirection(cloud1,viewDirAug1,.5,.02);
       cv::Scalar mean1,stddev1,mean2,stddev2; 
       cv::meanStdDev(rangeImage1,mean1,stddev1);
       if (stddev1[0]>maxDev1){
           maxDev1 = stddev1[0];
           pitchMax = pitch[ip];
           rollMax = roll[ir]; 
           bestNorm1 = viewDirAug1;
       } 
       string filename = "../data/image" + boost::to_string(3*ip + ir) + ".png";
       cv::imwrite(filename.c_str(),rangeImage1);
       images.push_back(rangeImage1);

    }
  }
 

  std::cout<<"pitchmax,rollmax,stddev: "<<180./M_PI*pitchMax<<" | "<<180./M_PI*rollMax<<" | "<<maxDev1<<std::endl;
  // now extract and display features on best image
  // detecting keypoints
  cv::Mat mask1;
  cv::Mat bestimage1 = imageFromCloudInDirection(cloud1,bestNorm1,.5,.01);
  mask1 = imageFromCloudInDirection(cloud1,bestNorm1,.5,.01,true);
  std::vector<cv::KeyPoint> keypoints1;
  cv::Mat blurredimage1;
  cv::blur(bestimage1,blurredimage1,cv::Size(11,11));
  // Parameters for shi tomasi detector
  int maxCorners = 25;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = .04;
  /// Draw corners detected
  int r = 4; 
  //cv::imwrite("image1.png",blurredimage1);


  return (0);
}

#else


#endif
