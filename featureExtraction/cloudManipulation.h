#ifndef CLOUDMANIP
#define CLOUDMANIP
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>



pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr,float);

pcl::PointCloud<pcl::PointNormal>::Ptr addNormalsToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::Normal>::Ptr);

void normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr);










#endif
