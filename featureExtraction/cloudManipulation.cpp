#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/harris_3d.h>
#include "cloudManipulation.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

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
  //viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr 
                    findHarrisCorners(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::PointCloud<pcl::Normal>::Ptr normals,
                                      float radius, float threshold = .001)
{
    pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI,pcl::Normal> detector;
    detector.setNonMaxSupression (true);
    detector.setRadius (radius);
    detector.setInputCloud(cloud);
    detector.setNormals(normals);
    detector.setRefine(false);
    detector.setThreshold(threshold);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    detector.setSearchMethod (tree);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
    detector.compute(*keypoints);
    
    std::cout << "keypoints detected: " << keypoints->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ tmp;
    double max = 0,min=1.;

    for(pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i!= keypoints->end(); i++){
        tmp = pcl::PointXYZ((*i).x,(*i).y,(*i).z);
        if ((*i).intensity>max ){
            max = (*i).intensity;
        }
        if ((*i).intensity<min){
            min = (*i).intensity;
        }
        keypoints3D->push_back(tmp);
    }

return keypoints3D;

}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr 
                    calculateFPFHDescriptors(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr searchSurface, 
                                             pcl::PointCloud<pcl::Normal>::Ptr normals,
                                             float radius)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  FPFH stuff
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (keypoints);
    fpfh.setInputNormals (normals);
    fpfh.setSearchSurface(searchSurface);
    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);

    fpfh.setSearchMethod (tree1);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (radius);

    // Compute the features
    fpfh.compute (*fpfhs);

    /*
        pcl::visualization::PCLHistogramVisualizer hist;
        hist.setBackgroundColor(0.,0.,0.);
        hist.addFeatureHistogram(*fpfhs,sizeof(fpfhs->points[0].histogram)/sizeof(fpfhs->points[0].histogram[0])-1,"cloud");
        hist.spinOnce(100);
    for (int ii = 0; ii < fpfhs->points.size(); ii++){
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh (new pcl::PointCloud<pcl::FPFHSignature33> ());
        fpfh->push_back(fpfhs->points[ii]);
        const std::string id="cloud";
        const std::string field= "fpfh"; 
        //hist.addFeatureHistogram(*fpfhs,sizeof(fpfhs->points[0].histogram)/sizeof(fpfhs->points[0].histogram[0]),id);
        hist.updateFeatureHistogram(*fpfh,sizeof(fpfhs->points[0].histogram)/sizeof(fpfhs->points[0].histogram[0])-1,id);
        hist.spinOnce(100);
    }
    */

return fpfhs;




}

/*
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(subcloud, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpcolor(keypoints3D, 255, 0, 0);
    viewer.addPointCloud(subcloud,pccolor,"testimg.png");
    viewer.addPointCloud(keypoints3D,kpcolor,"keypoints.png");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints.png"); 
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce();
        pcl_sleep (0.01);
    } 

*/

Eigen::Matrix4f matchFeaturesRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr source_points,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr target_points,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_descriptors, double * error)

{
   /*
   pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ,pcl::FPFHSignature33> sac;
   pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   //Provide a pointer to the input point cloud and features
   sac.setInputSource(source_points);
   sac.setSourceFeatures (source_descriptors);
   // Provide a pointer to the target point cloud and features
   sac.setInputTarget(target_points);
   sac.setTargetFeatures (target_descriptors);
   sac.setMinSampleDistance(10.);
   sac.setNumberOfSamples(4);
   sac.setMaxCorrespondenceDistance(100.);
   sac.setCorrespondenceRandomness(3);
   sac.setRANSACIterations(50);
   // Align input to target to obtain
   sac.align (*aligned_cloud);
   std::cout << "ransac iterations: "<<sac.getRANSACIterations()<<std::endl;
   std::cout << "has converged? : "<< sac.hasConverged()<<std::endl;
   
   double max_range = 5.;

   */ 
   
   pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
   icp.setInputSource(source_points);
   icp.setInputTarget(target_points);
   pcl::PointCloud<pcl::PointXYZ> Final;
   icp.align(Final);
   std::cout << "has converged:" << icp.hasConverged() << " score: " <<
   icp.getFitnessScore() << std::endl;
   std::cout << icp.getFinalTransformation() << std::endl;

   *error = icp.getFitnessScore();
   
   //*error = sac.getFitnessScore();
   //Eigen::Matrix4f transformation = sac.getFinalTransformation();
   Eigen::Matrix4f transformation = icp.getFinalTransformation();
   return transformation;


}

            
