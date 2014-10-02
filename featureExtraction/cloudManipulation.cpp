#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/harris_3d.h>
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float radius)
{

  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (radius);

  // Compute the features
  ne.compute (*cloud_normals);

  return cloud_normals;
}


pcl::PointCloud<pcl::PointNormal>::Ptr addNormalsToCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{

pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);

pcl::concatenateFields(*cloud,*normals,*cloudWithNormals);

return cloudWithNormals;

}


void normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  // Add color to subcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int ii = 0; ii < cloud->points.size(); ii++){
     pcl::PointXYZRGB point;
     point.x = cloud->points[ii].x;
     point.y = cloud->points[ii].y;
     point.z = cloud->points[ii].z;
     uint8_t r = (uint8_t)(std::min(3000.*normals->points[ii].curvature,255.));
     uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(255-r));
     //point.rgb = *reinterpret_cast<float*>(&rgb); 
     point.rgba = rgb; 
     point.r = r; 
     point.g = 0;
     point.b = 255-r;
     colorCloud->points.push_back(point); 
 }
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,"subcloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "subcloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (colorCloud, normals, 15, 1., "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return;
}


