#ifndef CLOUDMANIP
#define CLOUDMANIP
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>



pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr,float);

pcl::PointCloud<pcl::PointNormal>::Ptr addNormalsToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr);

void normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr);

pcl::PointCloud<pcl::PointXYZ>::Ptr 
                    findHarrisCorners(pcl::PointCloud<pcl::PointXYZ>::Ptr ,
                                      pcl::PointCloud<pcl::Normal>::Ptr ,
                                      float , float );


pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
                    calculateFPFHDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr ,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr , 
                                             pcl::PointCloud<pcl::Normal>::Ptr ,
                                             float );


Eigen::Matrix4f matchFeaturesRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_desc,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_desc, double*);



#endif
