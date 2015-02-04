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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("subcloudWithNormals.pcd", *cloud) == -1) //* load the file
  //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("AfirstPass1600.pcd", *cloud1) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/firstPass.pcd", *cloud1) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
   //if (pcl::io::loadPCDFile<pcl::PointXYZ> ("AsecondPass11500.pcd", *cloud2) == -1) //* load the file
   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../data/secondPass.pcd", *cloud2) == -1) //* load the file
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
  mask1 = imageFromCloudInDirection(cloud1,bestNorm1,.5,.01,true);
  cv::Mat bestimage2 = imageFromCloudInDirection(cloud2,bestNorm2,.5,.1);
  mask2 = imageFromCloudInDirection(cloud2,bestNorm2,.5,.1,true);
  std::vector<cv::KeyPoint> keypoints1;
  std::vector<cv::KeyPoint> keypoints2;
  cv::Mat blurredimage1;
  cv::Mat blurredimage2;
  cv::blur(bestimage1,blurredimage1,cv::Size(11,11));
  cv::blur(bestimage2,blurredimage2,cv::Size(11,11));
  // Parameters for shi tomasi detector
  int maxCorners = 25;
  double qualityLevel = 0.01;
  double minDistance = 10;
  int blockSize = 3;
  bool useHarrisDetector = false;
  double k = .04;
  /// Draw corners detected
  int r = 4; 
  cv::imwrite("image1.png",blurredimage1);
  cv::imwrite("image2.png",blurredimage2);
  /***************************************************
  
      Feature extraction, detection, and matching 

  ****************************************************/
  //****************************************************
  // Create objects
  //****************************************************
  cout << "< Creating detector, descriptor extractor and descriptor matcher ..." << endl;
  cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( "GFTT" ); // shi-tomasi
  std::string descriptorType = "ORB";
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::DescriptorExtractor::create( descriptorType );
  cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create( "BruteForce-Hamming" );
  //****************************************************
  // Run it 
  //****************************************************
    // image 1
    std::cout << std::endl << "< Extracting keypoints from first image..." << std::endl;
    detector->detect( blurredimage1, keypoints1 , mask1 );
    cout << keypoints1.size() << " points" << endl << ">" << endl;
    cout << "< Computing descriptors for keypoints from first image..." << endl;
    cv::Mat descriptors1;
    descriptorExtractor->compute( blurredimage1, keypoints1, descriptors1 );
    cout << ">" << endl;
    // image 2
    cout << endl << "< Extracting keypoints from second image..." << endl;
    detector->detect( blurredimage2, keypoints2, mask2 );
    cout << keypoints2.size() << " points" << endl << ">" << endl;
    cout << "< Computing descriptors for keypoints from second image..." << endl;
    cv::Mat descriptors2;
    descriptorExtractor->compute( blurredimage2, keypoints2, descriptors2 );
    cout << ">" << endl;
    // match
    vector<DMatch> filteredMatches;
    cv::Mat featureMatchImage;
    crossCheckMatching( descriptorMatcher,descriptors1, descriptors2,filteredMatches);
    // RANSAC
    Mat H12;
    double ransacReprojThreshold = 3.;
    vector<int> queryIdxs( filteredMatches.size() ), trainIdxs( filteredMatches.size() );
    for( size_t i = 0; i < filteredMatches.size(); i++ )
    {
        queryIdxs[i] = filteredMatches[i].queryIdx;
        trainIdxs[i] = filteredMatches[i].trainIdx;
    }
    if( true) //!H12.empty() ) // filter outliers
    {
        vector<char> matchesMask( filteredMatches.size(), 0 );
        vector<Point2f> points1; KeyPoint::convert(keypoints1, points1, queryIdxs);
        vector<Point2f> points2; KeyPoint::convert(keypoints2, points2, trainIdxs);
        H12 = findHomography( Mat(points1), Mat(points2), 0, ransacReprojThreshold ); 
        cout << "homography matrix: \n"<<H12<<endl;
        Mat points1t; perspectiveTransform(Mat(points1), points1t, H12);

        double maxInlierDist = ransacReprojThreshold < 0 ? 3 : ransacReprojThreshold;
        for( size_t i1 = 0; i1 < points1.size(); i1++ )
        {
            if( norm(points2[i1] - points1t.at<Point2f>((int)i1,0)) <= maxInlierDist ) // inlier
                matchesMask[i1] = 1;
        }
        // draw inliers
        //drawMatches( blurredimage1, keypoints1, blurredimage2, keypoints2, filteredMatches, 
          //           featureMatchImage, Scalar(0, 255, 0), Scalar(255, 0, 0), matchesMask);
        drawMatches( mask1, keypoints1, mask2, keypoints2, filteredMatches, 
                     featureMatchImage, Scalar(0, 255, 0), Scalar(255, 0, 0), matchesMask);

#if 0
        // draw outliers
        for( size_t i1 = 0; i1 < matchesMask.size(); i1++ )
            matchesMask[i1] = !matchesMask[i1];
        drawMatches( img1, keypoints1, img2, keypoints2, filteredMatches, drawImg, Scalar(255, 0, 0), Scalar(0, 0, 255), matchesMask,
                     DrawMatchesFlags::DRAW_OVER_OUTIMG | DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
#endif

        cout << "Number of inliers: " << countNonZero(matchesMask) << endl;
    }
    else
        drawMatches( blurredimage1, keypoints1, blurredimage2, keypoints2, filteredMatches, featureMatchImage );






  //cv::drawKeypoints(bestimage,keypoints1,featureImage);
  //cv::drawMatches(bestimage1,keypoints1,bestimage2,keypoints2,matches,featureMatchImage);
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
