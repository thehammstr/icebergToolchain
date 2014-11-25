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
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
// MACROS

DEFINE_string(input, "", "Input File name");


using namespace std;

#if(1)

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("subcloudWithNormals.pcd", *cloud) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("AfirstPass1600.pcd", *cloud1) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("firstPass.pcd", *cloud1) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
   //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("AsecondPass11500.pcd", *cloud2) == -1) //* load the file
   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("secondPass.pcd", *cloud2) == -1) //* load the file
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
   for (int ii = 0; ii<cloud2->points.size(); ii++){
     cloud2->points[ii].z = - cloud2->points[ii].z;
  } 

 float scaleRadius = 1.;
  pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals(cloud1,scaleRadius);
  pcl::PointCloud<pcl::Normal>::Ptr normals2 = getNormals(cloud2,scaleRadius);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals1 = addNormalsToCloud(cloud1,normals1);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals2 = addNormalsToCloud(cloud2,normals2);
        // view
        normalsVis(cloud1,normals1); 
        normalsVis(cloud2,normals2); 
 cv::namedWindow("image1", CV_WINDOW_AUTOSIZE);
 cv::namedWindow("image2", CV_WINDOW_AUTOSIZE);
 pcl::Normal viewDir1 = avgNormal(normals1);
 pcl::Normal viewDir2 = avgNormal(normals2);
 Eigen::Vector3f nominalNormal1(viewDir1.normal_x,viewDir1.normal_y,viewDir1.normal_z);
 Eigen::Vector3f nominalNormal2(viewDir2.normal_x,viewDir2.normal_y,viewDir2.normal_z);
  double maxDev1 = 0.;
  double maxDev2 = 0.;
  double pitchMax = 0.;
  double rollMax = 0.;
  pcl::Normal bestNorm1;
  pcl::Normal bestNorm2; 
  for (float pitch = 0.; pitch < .1; pitch+= .5){
  //for (float pitch = -.15; pitch < .15; pitch+= .05){
    for (float roll = 0.; roll < .1; roll += .5){
    //for (float roll = -.15; roll < .15; roll += .05){
       // rotate nominal normal
       Eigen::Matrix3f transform_1;
       transform_1 = Eigen::AngleAxisf(pitch,Eigen::Vector3f::UnitX())
                                       * Eigen::AngleAxisf(roll,Eigen::Vector3f::UnitY());
       Eigen::Vector3f newNormal1 =transform_1*nominalNormal1;
       Eigen::Vector3f newNormal2 =transform_1*nominalNormal2;
       pcl::Normal viewDirAug1(newNormal1(0),newNormal1(1),newNormal1(2));
       pcl::Normal viewDirAug2(newNormal2(0),newNormal2(1),newNormal2(2));
       cv::Mat rangeImage1 = imageFromCloudInDirection(cloud1,viewDirAug1,.5,.02);
       cv::Mat rangeImage2 = imageFromCloudInDirection(cloud2,viewDirAug2,.5,.1);
       cv::Scalar mean1,stddev1,mean2,stddev2; 
       cv::meanStdDev(rangeImage1,mean1,stddev1);
       cv::meanStdDev(rangeImage2,mean2,stddev2);
       if (stddev1[0]>maxDev1){
           maxDev1 = stddev1[0];
           pitchMax = pitch;
           rollMax = roll; 
           bestNorm1 = viewDirAug1;
       } 
       if (stddev2[0]>maxDev2){
           maxDev2 = stddev2[0];
           bestNorm2 = viewDirAug2;
       }
       cv::imshow("image1", rangeImage1);
       cv::imshow("image2", rangeImage2);
       std::cout<<"pitch,roll,mean,stddev: "<<180./M_PI*pitch<<" "<<180./M_PI*roll<<" | "<<mean1[0]<<" "<<stddev1[0]<<std::endl;
       cv::waitKey(5000);

    }
  }
  std::cout<<"pitchmax,rollmax,stddev: "<<180./M_PI*pitchMax<<" | "<<180./M_PI*rollMax<<" | "<<maxDev1<<std::endl;
  // now extract and display features on best image
  // detecting keypoints
  cv::Mat mask1;
  cv::Mat mask2;
  cv::Mat bestimage1 = imageFromCloudInDirection(cloud1,bestNorm1,.5,.01);
  cv::Mat bestimage2 = imageFromCloudInDirection(cloud2,bestNorm2,.5,.1);
  int minHessian = 500; // 400 by default
  //cv::SurfFeatureDetector detector(minHessian);
  cv::ORB detector;
  std::vector<cv::KeyPoint> keypoints1;
  std::vector<cv::KeyPoint> keypoints2;
  cv::Mat blurredimage1;
  cv::Mat blurredimage2;
  cv::blur(bestimage1,blurredimage1,cv::Size(11,11));
  cv::blur(bestimage2,blurredimage2,cv::Size(11,11));
  //detector.detect(blurredimage1, keypoints1);
  //detector.detect(blurredimage2, keypoints2);
  // computing descriptors
  //cv::SurfDescriptorExtractor extractor;
  cv::Mat descriptors1;
  cv::Mat descriptors2;
  detector(blurredimage1,cv::Mat::ones(blurredimage1.rows,blurredimage1.cols,CV_8U),keypoints1,descriptors1);
  detector(blurredimage2,cv::Mat::ones(blurredimage2.rows,blurredimage2.cols,CV_8U),keypoints2,descriptors2);
  //extractor.compute(bestimage1, keypoints1, descriptors1);
  //extractor.compute(bestimage2, keypoints2, descriptors2);
  //cv::DescriptorMatcher matcher;
  cv::BFMatcher matcher;
  std::vector< cv::DMatch > matches; 
  //matcher.create("BruteForce-Hamming");
  matcher.match(descriptors1,descriptors2,matches);
  cv::Mat featureMatchImage;
  //cv::drawKeypoints(bestimage,keypoints1,featureImage);
  cv::drawMatches(bestimage1,keypoints1,bestimage2,keypoints2,matches,featureMatchImage);
  cv::namedWindow("image3", CV_WINDOW_AUTOSIZE);
  cv::imshow("image3",featureMatchImage);
  // edge detection
  int edgeThresh = 1;
  int lowThreshold = 80;
  int const max_lowThreshold = 100;
  int ratio = 3;
  int kernel_size = 5;
  cv::Mat canny1,canny2;
  cv::blur(bestimage1,canny1,cv::Size(7,7));
  cv::blur(bestimage2,canny2,cv::Size(7,7));
  //cv::Canny(canny1,canny1,lowThreshold,lowThreshold*ratio,kernel_size);
  //cv::Canny(canny2,canny2,lowThreshold,lowThreshold*ratio,kernel_size);
  //cv::blur(canny1,canny1,cv::Size(3,3));
  //cv::blur(canny2,canny2,cv::Size(3,3));
  cv::imshow("image1", canny1);
  cv::imshow("image2", canny2);
  // template matching
  cv::Mat corrSurf;
  cv::matchTemplate(canny1,canny2,corrSurf,CV_TM_CCORR_NORMED);
  //cv::matchTemplate(canny1,canny2,corrSurf,CV_TM_CCORR);
  //cv::matchTemplate(canny1,canny2,corrSurf,CV_SQDIFF);
  cv::normalize(corrSurf,corrSurf,0,1,cv::NORM_MINMAX, -1, cv::Mat());
  double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
  cv::Point matchLoc;
  cv::minMaxLoc( corrSurf, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
  matchLoc=maxLoc;
  cv::circle(corrSurf,matchLoc,5,.5);
  cv::namedWindow("image4",CV_WINDOW_AUTOSIZE);
  cv::imshow("image4",corrSurf);
  cv::waitKey(0000);
  //buildNormalImageFromCloud(cloudWithNormals);
  /*boost::shared_ptr<pcl::RangeImage> image = buildRangeImageFromCloud(cloudWithNormals);
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (*image);
  while (!range_image_widget.wasStopped ())
  {
    range_image_widget.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  */

  return (0);
}

#else


#endif
