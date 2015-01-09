#ifndef CLOUDMANIP
#define CLOUDMANIP
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/range_image/range_image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr,float);

pcl::Normal avgNormal(pcl::PointCloud<pcl::Normal>::Ptr);

pcl::PointCloud<pcl::PointNormal>::Ptr addNormalsToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr);

void normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr);
void normalsVisHeight (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr);

pcl::PointCloud<pcl::PointXYZ>::Ptr 
                    findHarrisCorners(pcl::PointCloud<pcl::PointXYZ>::Ptr ,
                                      pcl::PointCloud<pcl::Normal>::Ptr ,
                                      float , float );


pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
                    calculateFPFHDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr ,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr , 
                                             pcl::PointCloud<pcl::Normal>::Ptr ,
                                             float );


Eigen::Matrix4f matchFeaturesRANSAC(pcl::PointCloud<pcl::PointNormal>::Ptr source_cloud,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_desc,
                                    pcl::PointCloud<pcl::PointNormal>::Ptr target_cloud,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_desc, double*);



//pcl::PointCloud<pcl::PointXYZ>::Ptr 
//   buildNormalImageFromCloud(pcl::PointCloud< pcl::PointNormal >::Ptr );
boost::shared_ptr<pcl::RangeImage>
   buildRangeImageFromCloud(pcl::PointCloud< pcl::PointNormal >::Ptr);

void buildNormalImageFromCloud(pcl::PointCloud< pcl::PointNormal >::Ptr);

cv::Mat  imageFromCloudInDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::Normal,float,float,bool maskMask = false);

cv::Mat fillInRangeImageGaps(pcl::PointCloud<pcl::PointXYZI>::ConstPtr, bool maskMask = false);


#endif
