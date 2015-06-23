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
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
//#include <pcl/registration/correspondence_rejection_boundary.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/features/boundary.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>

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

pcl::Normal avgNormal(pcl::PointCloud<pcl::Normal>::Ptr normals){

   Eigen::Vector3f normalAcc(0.,0.,0.);
   float pointCounter = 0.;
   for (int ii = 0; ii < normals->points.size(); ii++){
            // check for valid normal
      if ((isnan(normals->points[ii].normal[0]) || isnan(normals->points[ii].normal[1]) || isnan(normals->points[ii].normal[2])) ) {
         // do nothing 
      } else {
         normalAcc(0) =  (pointCounter/(pointCounter+1.))*normalAcc(0) + (1./(pointCounter+1.))*normals->points[ii].normal[0];
         normalAcc(1) =  (pointCounter/(pointCounter+1.))*normalAcc(1) + (1./(pointCounter+1.))*normals->points[ii].normal[1];
         normalAcc(2) =  (pointCounter/(pointCounter+1.))*normalAcc(2) + (1./(pointCounter+1.))*normals->points[ii].normal[2];
         pointCounter += 1.;
      }

   }
   normalAcc.normalize();
   std::cout<<"average normal: "<<normalAcc<<std::endl;
   pcl::Normal normOut(normalAcc(0),normalAcc(1),normalAcc(2));
   return normOut;
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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  //**********************************************************
  //**********************************************************
  // color code boundary info to test boundary estimation
  //**********************************************************
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
  est.setInputCloud (cloud);
  est.setInputNormals (normals);
  est.setRadiusSearch (2.);   // 2cm radius
  est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  est.compute (boundaries);
  //**********************************************************
  //**********************************************************

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
     if (boundaries.points[ii].boundary_point || isnan((float)boundaries.points[ii].boundary_point) ){
        point.g = 255;
     }
     colorCloud->points.push_back(point); 
 }
  // plot origin and axes
  pcl::PointXYZRGB origin;
  origin.x = 0.;
  origin.y = 0.;
  origin.z = 0.;
  origin.r = 255;
  origin.b = 255;
  origin.g = 255;
  originCloud->points.push_back(origin);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(originCloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,"subcloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (originCloud, rgb2,"subcloudorigin");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "subcloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "subcloudorigin");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (colorCloud, normals, 15, 1., "normals");
  viewer->addCoordinateSystem(10.0);
  viewer->setCameraPosition(-190.,200.,-42.,0.,-40.,0.,0.,0.,-1.);
  //viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return;
}

void normalsVisHeight (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  // Add color to subcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr originCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  //**********************************************************
  //**********************************************************
  // color code boundary info to test boundary estimation
  //**********************************************************
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
  est.setInputCloud (cloud);
  est.setInputNormals (normals);
  est.setRadiusSearch (2.);   // 2cm radius
  est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  est.compute (boundaries);
  //**********************************************************
  //**********************************************************
  float minZ = 1000000.;
  float maxZ = -1000000.;
  for (int ii = 0; ii < cloud->points.size(); ii++){
     minZ = std::min(minZ,cloud->points[ii].z);
     maxZ = std::max(maxZ,cloud->points[ii].z);
  }

  for (int ii = 0; ii < cloud->points.size(); ii++){
     pcl::PointXYZRGB point;
     point.x = cloud->points[ii].x;
     point.y = cloud->points[ii].y;
     point.z = cloud->points[ii].z-minZ;
     //uint8_t r = (uint8_t)(std::max(std::min(5.*cloud->points[ii].z,255.),0.));
     uint8_t r = (uint8_t)(255.*(cloud->points[ii].z - minZ)/(maxZ-minZ));
     uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(255-r));
     //point.rgb = *reinterpret_cast<float*>(&rgb); 
     point.rgba = rgb; 
     point.r = r; 
     point.g = 0;
     point.b = 255-r;
     if (boundaries.points[ii].boundary_point || isnan((float)boundaries.points[ii].boundary_point) ){
        point.g = 255;
     }
     colorCloud->points.push_back(point); 
 }
pcl::PointXYZRGB origin;
  origin.x = 0.;
  origin.y = 0.;
  origin.z = 0.;
  origin.r = 255;
  origin.b = 255;
  origin.g = 255;
  originCloud->points.push_back(origin);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(originCloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,"subcloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (originCloud, rgb2,"subcloudorigin");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "subcloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "subcloudorigin");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (colorCloud, normals, 15, 1., "normals");
  viewer->addCoordinateSystem(10.0);
  viewer->setCameraPosition(-190.,200.,-42.,0.,-40.,0.,0.,0.,-1.);
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

    // weed out spurious boundary features
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
    est.setInputCloud (cloud);
    est.setInputNormals (normals);
    est.setRadiusSearch (2.);   // 2cm radius
    est.setSearchMethod (typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute (boundaries);   
 
    std::cout << "keypoints detected: " << keypoints->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ tmp;
    double max = 0,min=1.;
    unsigned int counter = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i!= keypoints->end(); i++){
        tmp = pcl::PointXYZ((*i).x,(*i).y,(*i).z);
      // only add keypoint if not on a boundary
      if( !boundaries.points[counter].boundary_point){
        if ((*i).intensity>max ){
            max = (*i).intensity;
        }
        if ((*i).intensity<min){
            min = (*i).intensity;
        }
        keypoints3D->push_back(tmp);
      }
      counter++;
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

Eigen::Matrix4f matchFeaturesRANSAC(pcl::PointCloud<pcl::PointNormal>::Ptr source_points,
                                    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_descriptors,
                                    pcl::PointCloud<pcl::PointNormal>::Ptr target_points,
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
   
   pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
   //pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>::Ptr trans_lls (new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>);
   pcl::registration::TransformationEstimation2D<pcl::PointNormal, pcl::PointNormal>::Ptr trans_2D (new pcl::registration::TransformationEstimation2D<pcl::PointNormal, pcl::PointNormal>);
   icp.setTransformationEstimation (trans_2D);
   pcl::registration::CorrespondenceRejectorOneToOne::Ptr one2one(new pcl::registration::CorrespondenceRejectorOneToOne);
   //pcl::registration::CorrespondenceRejectorBoundary::Ptr boundaryRejector(new pcl::registration::CorrespondenceRejectorBoundary);
   //pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr boundaryRejector(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
   //boundaryRejector->initializeDataContainer<pcl::PointNormal,pcl::Normal>();
   //boundaryRejector->setInputSource<pcl::PointNormal>(source_points);
   //boundaryRejector->setInputTarget<pcl::PointNormal>(target_points);
   //pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr ransaq (new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);
   icp.setUseReciprocalCorrespondences(false); // setting this to true resulted in slower and worse results. boo. why?
   icp.setMaximumIterations(300);
   icp.setMaxCorrespondenceDistance(50.);
   std::cout << "max correspondence distance: "<< icp.getMaxCorrespondenceDistance() << std::endl;
   icp.addCorrespondenceRejector(one2one);
   //icp.addCorrespondenceRejector(boundaryRejector);
   icp.setInputSource(source_points);
   icp.setInputTarget(target_points);
   pcl::PointCloud<pcl::PointNormal> Final;
   icp.align(Final);
   std::cout << "round 1 has converged:" << icp.hasConverged() << " score: " <<
   icp.getFitnessScore(4.) << std::endl;
   std::cout << icp.getFinalTransformation() << std::endl;

   *error = icp.getFitnessScore();
   
   //*error = sac.getFitnessScore();
   //Eigen::Matrix4f transformation = sac.getFinalTransformation();
   Eigen::Matrix4f transformation = icp.getFinalTransformation();
 
   return transformation;


}


 boost::shared_ptr<pcl::RangeImage> buildRangeImageFromCloud(pcl::PointCloud< pcl::PointNormal >::Ptr cloud){

   
   // calculate average normal direction (weight by curvature?)
   double pointCounter = 0.;
   Eigen::Vector3f normalAcc(0.,0.,0.);
   Eigen::Vector3f centroid(0.,0.,0.);
   pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   for (int ii = 0; ii < cloud->points.size(); ii++){
      pcl::PointXYZ foo(cloud->points[ii].x,cloud->points[ii].y,cloud->points[ii].z);
      simple_cloud->points.push_back(foo);
      // calculate centroid
      centroid(0) = ((float)ii/((float)ii + 1.))*centroid(0) + (1./((float)ii+1.))*cloud->points[ii].x;
      centroid(1) = ((float)ii/((float)ii + 1.))*centroid(1) + (1./((float)ii+1.))*cloud->points[ii].y;
      centroid(2) = ((float)ii/((float)ii + 1.))*centroid(2) + (1./((float)ii+1.))*cloud->points[ii].z;
      // check for valid normal
      if ((isnan(cloud->points[ii].normal[0]) || isnan(cloud->points[ii].normal[1]) || isnan(cloud->points[ii].normal[2])) ) {
         // do nothing 
      } else {
         normalAcc(0) =  (pointCounter/(pointCounter+1.))*normalAcc(0) + (1./(pointCounter+1.))*cloud->points[ii].normal[0];
         normalAcc(1) =  (pointCounter/(pointCounter+1.))*normalAcc(1) + (1./(pointCounter+1.))*cloud->points[ii].normal[1];
         normalAcc(2) =  (pointCounter/(pointCounter+1.))*normalAcc(2) + (1./(pointCounter+1.))*cloud->points[ii].normal[2];
         pointCounter += 1.;
      }
      
   }
   // visualize average normal

   normalAcc.normalize();
   std::cout<<"normal: \n"<<normalAcc<<std::endl;
   std::cout<<"centroid: \n"<<centroid<<std::endl;
   pcl::PointCloud<pcl::Normal>::Ptr average_normals (new pcl::PointCloud<pcl::Normal>);
   pcl::Normal norm(normalAcc(0),normalAcc(1),normalAcc(2));
   for (int ii = 0; ii < cloud->points.size(); ii++){
      average_normals->points.push_back(norm);   
   }
   normalsVis(simple_cloud,average_normals); 
 

   // rotate point cloud so z aligns with normal
   Eigen::Vector3f nz(0.,0.,1.);
   Eigen::Vector3f axis = normalAcc.cross(-nz);
   float sintheta = axis.norm();
   std::cout<<"axis: "<<axis<<std::endl;
   Eigen::Affine3f xform = Eigen::Affine3f::Identity();
   //xform.translation()<< -centroid(0),-centroid(1),-centroid(2);
   xform.rotate(Eigen::AngleAxisf(asin(sintheta),(1./sintheta)*axis));
   pcl::PointCloud<pcl::PointXYZ>::Ptr flattenedCloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::transformPointCloud(*simple_cloud,*flattenedCloud,xform);
   pcl::PointCloud<pcl::Normal>::Ptr flattenedNormals = getNormals(flattenedCloud,1.);
   normalsVisHeight(flattenedCloud,flattenedNormals);
   // calc new avg normal
   pointCounter = 0.;
   normalAcc(0) = 0.;
   normalAcc(1) = 0.;
   normalAcc(2) = 0.;
   for (int ii = 0; ii < flattenedCloud->points.size(); ii++){
      // check for valid normal
      if ((isnan(flattenedNormals->points[ii].normal[0]) || isnan(flattenedNormals->points[ii].normal[1]) || isnan(flattenedNormals->points[ii].normal[2])) ) {
         // do nothing 
      } else {
         normalAcc(0) =  (pointCounter/(pointCounter+1.))*normalAcc(0) + (1./(pointCounter+1.))*flattenedNormals->points[ii].normal[0];
         normalAcc(1) =  (pointCounter/(pointCounter+1.))*normalAcc(1) + (1./(pointCounter+1.))*flattenedNormals->points[ii].normal[1];
         normalAcc(2) =  (pointCounter/(pointCounter+1.))*normalAcc(2) + (1./(pointCounter+1.))*flattenedNormals->points[ii].normal[2];
         pointCounter += 1.;
      }
   }   
  normalAcc.normalize();
  std::cout<<"flattened average normal: "<<normalAcc(0)<<" "<<normalAcc(1)<<" "<<normalAcc(2)<<std::endl;
  flattenedCloud->width = (uint32_t) flattenedCloud->points.size();
  flattenedCloud->height = 1;
  
  
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  .5f * (M_PI/180.0f));  //   .5 degree in radians
  float maxAngleWidth     = (float) (120.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (120.0f * (M_PI/180.0f));  // 180.0 degree in radians
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.01;
  float minRange = 0.0f;
  int borderSize = 1;
 
 // create rectangular bounding box in x-y (crop to fit)
   float maxX = -1e23;
   float maxY = -1e23;
   float minX = 1e23;
   float minY = 1e23;
   float maxZ = -1e23;
   float minZ = 1e23;
   int imwidth = 800;
   int imheight = 600;
   for (int jj = 0; jj<flattenedCloud->points.size(); jj++){
      maxX = std::max(maxX, flattenedCloud->points[jj].x);
      maxY = std::max(maxY, flattenedCloud->points[jj].y);
      maxZ = std::max(maxZ, flattenedCloud->points[jj].z);
      minX = std::min(minX, flattenedCloud->points[jj].x);
      minY = std::min(minY, flattenedCloud->points[jj].y);
      minZ = std::min(minZ, flattenedCloud->points[jj].z);

   }
  std::cout<< "maxX "<<maxX <<"\nminX "<<minX<<"\nmaxY "<<maxY<<"\nminY "<<minY<<"\nmaxZ "<<maxZ<<"\nminZ "<<minZ<<std::endl;
   Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f((maxX+minX)/2.,(maxY+minY)/2.,-5.*minZ);
  Eigen::Vector3f nx(1.,0.,0.);
  //sensorPose.rotate(Eigen::AngleAxisf(M_PI,nx)); 
  //pcl::RangeImage rangeImage;
  boost::shared_ptr<pcl::RangeImage> rangeImage (new pcl::RangeImage);
  rangeImage->createFromPointCloud(*flattenedCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  /*range_image.createFromPointCloudWithFixedSize(*flattenedCloud,
                                                  imwidth,
                                                  imheight,
                                                  (maxX+minX)/2.,
                                                  (maxY+minY)/2.,
                                                  .1, // focal length x
                                                  .1, // focal length y
                                                  sensorPose,
                                                  coordinate_frame,
                                                  noiseLevel,
                                                  minRange);*/
                                                  
  std::cout << *rangeImage << "\n";
 return rangeImage; 
}

 void buildNormalImageFromCloud(pcl::PointCloud< pcl::PointNormal >::Ptr cloud){

   // calculate average normal direction (weight by curvature?)
   double pointCounter = 0.;
   Eigen::Vector3f normalAcc(0.,0.,0.);
   Eigen::Vector3f centroid(0.,0.,0.);
   pcl::PointCloud<pcl::PointXYZ>::Ptr simple_cloud (new pcl::PointCloud<pcl::PointXYZ>);
   for (int ii = 0; ii < cloud->points.size(); ii++){
      pcl::PointXYZ foo(cloud->points[ii].x,cloud->points[ii].y,cloud->points[ii].z);
      simple_cloud->points.push_back(foo);
      // calculate centroid
      centroid(0) = ((float)ii/((float)ii + 1.))*centroid(0) + (1./((float)ii+1.))*cloud->points[ii].x;
      centroid(1) = ((float)ii/((float)ii + 1.))*centroid(1) + (1./((float)ii+1.))*cloud->points[ii].y;
      centroid(2) = ((float)ii/((float)ii + 1.))*centroid(2) + (1./((float)ii+1.))*cloud->points[ii].z;
      // check for valid normal
      if ((isnan(cloud->points[ii].normal[0]) || isnan(cloud->points[ii].normal[1]) || isnan(cloud->points[ii].normal[2])) ) {
         // do nothing 
      } else {
         normalAcc(0) =  (pointCounter/(pointCounter+1.))*normalAcc(0) + (1./(pointCounter+1.))*cloud->points[ii].normal[0];
         normalAcc(1) =  (pointCounter/(pointCounter+1.))*normalAcc(1) + (1./(pointCounter+1.))*cloud->points[ii].normal[1];
         normalAcc(2) =  (pointCounter/(pointCounter+1.))*normalAcc(2) + (1./(pointCounter+1.))*cloud->points[ii].normal[2];
         pointCounter += 1.;
      }
      
   }
   // visualize average normal

   normalAcc.normalize();
   std::cout<<"normal: \n"<<normalAcc<<std::endl;
   std::cout<<"centroid: \n"<<centroid<<std::endl;
   pcl::PointCloud<pcl::Normal>::Ptr average_normals (new pcl::PointCloud<pcl::Normal>);
   //pcl::Normal norm(normalAcc(0),normalAcc(1),normalAcc(2));
   pcl::Normal norm(0.,0.,-1.);
   for (int ii = 0; ii < cloud->points.size(); ii++){
      average_normals->points.push_back(norm);   
   }
   normalsVis(simple_cloud,average_normals); 
  
     cv::namedWindow("image2", CV_WINDOW_AUTOSIZE);
  for (float theta = asin(normalAcc(1))-.2; theta <= asin(normalAcc(1))+.2; theta+=.1){
     pcl::Normal norma(0.,sin(theta),-cos(theta));
     cv::Mat poop = imageFromCloudInDirection(simple_cloud,norma,.3,.1);
     cv::imshow("image2", poop);
     cv::waitKey(10.); 

  }
     // sample rectified rectangular cloud at regular intervals, interpolating
   // record hole locations

   // fill in holes

   // visualize

   // scale z? linearly? nonlinearly? 

   // return organized point cloud

   

   return;
}


cv::Mat imageFromCloudInDirection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::Normal norm,float resolution, float marginpct, bool maskMask){
  
// rotate point cloud so z aligns with normal
   Eigen::Vector3f normalAcc(norm.normal_x,norm.normal_y,norm.normal_z);
   Eigen::Vector3f nz(0.,0.,1.);
   Eigen::Vector3f axis = normalAcc.cross(-nz);
   float sintheta = axis.norm();
   Eigen::Affine3f xform = Eigen::Affine3f::Identity();
   //xform.translation()<< -centroid(0),-centroid(1),-centroid(2);
   if (sintheta != 0.)
      xform.rotate(Eigen::AngleAxisf(asin(sintheta),(1./sintheta)*axis));
   pcl::PointCloud<pcl::PointXYZ>::Ptr flattenedCloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::transformPointCloud(*cloud,*flattenedCloud,xform);
   pcl::PointCloud<pcl::Normal>::Ptr flattenedNormals = getNormals(flattenedCloud,1.);
   //normalsVisHeight(flattenedCloud,flattenedNormals);

// create bounding box
   float maxX = -1e23;
   float maxY = -1e23;
   float minX = 1e23;
   float minY = 1e23;
   float maxZ = -1e23;
   float minZ = 1e23;
   for (int jj = 0; jj<cloud->points.size(); jj++){
      maxX = std::max(maxX,flattenedCloud->points[jj].x);
      maxY = std::max(maxY,flattenedCloud->points[jj].y);
      maxZ = std::max(maxZ,flattenedCloud->points[jj].z);
      minX = std::min(minX,flattenedCloud->points[jj].x);
      minY = std::min(minY,flattenedCloud->points[jj].y);
      minZ = std::min(minZ,flattenedCloud->points[jj].z);
   }
   // imWidth without margin
   int imWidth  = (int)ceil((maxX-minX)/resolution);
   int imHeight = (int)ceil((maxY-minY)/resolution);
   float dX = (maxX-minX)/imWidth;
   float dY = (maxY-minY)/imHeight;
   int idx1 = 0;
   int idx2 = 0;

  // create grid
  pcl::PointCloud<pcl::PointXYZI>::Ptr grid (new pcl::PointCloud<pcl::PointXYZI>);
  //populate grid
  //float marginpct = .15; // percent
  float xmargin = (maxX-minX)*marginpct;
  float ymargin = (maxY-minY)*marginpct;

  for (float x = minX+xmargin+.5*dX; x<maxX-xmargin; x+=dX){
      idx2=0;
      for( float y = minY+ymargin+.5*dY; y<maxY-ymargin; y+=dY){
         pcl::PointXYZI pt;
         pt.x = x;
         pt.y = y;
         pt.z = 0.;
         pt.intensity = -17.;
         grid->points.push_back(pt);
         idx2++;
      }
      idx1++;
   }
   //std::cout<< idx1 <<","<<idx2<<" of " <<imWidth<<","<<imHeight<<std::endl;
   grid->height = idx2;
   grid->width = idx1;
   // Big ol' hack to make range image
   // abusing point clouds for search
   pcl::PointCloud<pcl::PointXYZI>::Ptr buffer (new pcl::PointCloud<pcl::PointXYZI>);

   for (int idx = 0; idx<flattenedCloud->points.size(); idx++){
      pcl::PointXYZI proj;
      proj.x = flattenedCloud->points[idx].x;
      proj.y = flattenedCloud->points[idx].y;
      proj.z = 0.;
      proj.intensity = flattenedCloud->points[idx].z - minZ;
      buffer->points.push_back(proj);
   }
/* ///////////////////////////////////////////////////////////////////////
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("buffer Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(buffer,"intensity");
  viewer->addPointCloud<pcl::PointXYZI> (buffer,intensity,"flattened");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "flattened");
  viewer->addCoordinateSystem(10.0);
  viewer->setCameraPosition(-190.,200.,-42.,0.,-40.,0.,0.,0.,-1.);
  //viewer->initCameraParameters ();

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

 //////////////////////////////////////////////////////////////////////// */
      // sample rectified rectangular cloud at regular intervals, interpolating
   pcl::KdTreeFLANN<pcl::PointXYZI> kdtree; 
   kdtree.setInputCloud(buffer);
   for (int ii = 0; ii<grid->points.size(); ii++){
      // get all points within resolution radius
      pcl::PointXYZI searchPoint = grid->points[ii];
      int K = 4;
      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);   
      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {   float pointCount = 0.;
          for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
            // average points
            if (pointNKNSquaredDistance[i] <= resolution){
                grid->points[ii].intensity = (pointCount/(pointCount+1.))*grid->points[ii].intensity + (1./(pointCount+1.))*buffer->points[pointIdxNKNSearch[i] ].intensity;
                pointCount += 1.;
            }
          } 
      }

   }
/////////////////////////////////////////////////////////////////////////
   // visualize organized cloud
/*
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("buffer Viewer"));
  viewer2->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity2(grid,"intensity");
  viewer2->addPointCloud<pcl::PointXYZI> (grid,intensity2,"flattened");
  viewer2->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "flattened");
  viewer2->addCoordinateSystem(10.0);
  viewer2->setCameraPosition(5.,-42.,-160.,0.,-40.,0.,0.,-1.,0.);
  //viewer->initCameraParameters ();

  while (!viewer2->wasStopped ())
  {
    viewer2->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

*/
/////////////////////////////////////////////////////////////////////////
  
   // record hole locations

   // fill in holes

   // visualize

   // scale z? linearly? nonlinearly? 


  cv::Mat output = fillInRangeImageGaps(grid,maskMask);
  //if (maskMask)
    // return output;
  cv::Mat equalizedImage;
  cv::equalizeHist(output,equalizedImage);

#if(0)
  cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
  cv::imshow("image",output);
  cv::waitKey();
  cv::imshow("image", equalizedImage);
  cv::waitKey(); 
  //std::cout<<output<<std::endl;
#endif
  return equalizedImage;
  //return output;
}


cv::Mat fillInRangeImageGaps(pcl::PointCloud<pcl::PointXYZI>::ConstPtr input, bool maskMask){

   // create output image
   cv::Mat filledImage = cv::Mat::zeros(input->height,input->width,CV_32F);
   boost::shared_ptr< cv::Mat > bitMaskAdd (new cv::Mat(input->height,input->width,CV_8U));
   cv::Mat bitMask = *bitMaskAdd;
   bitMask = cv::Mat::zeros(input->height,input->width,CV_8U);
   cv::Mat notMask = cv::Mat::ones(input->height,input->width,CV_8U);
   int pclCounter = 0;
   for (int jj = 0; jj < filledImage.cols; jj++){
      for (int ii = 0; ii < filledImage.rows; ii++){
        filledImage.at<float>(ii,jj) = (float)input->points[pclCounter].intensity;
        if (input->points[pclCounter].intensity > 0.){
            bitMask.at<unsigned char>(ii,jj) = 1;
            notMask.at<unsigned char>(ii,jj) = 0;
        }
        pclCounter++;
     }
   }
   
   double tolerance = 1e-8;
   cv::Size kernSize = cv::Size(5,5);
   int maxLoop = 10000;
   cv::Mat inputImage = filledImage.clone();
   cv::namedWindow("progress",CV_WINDOW_AUTOSIZE);
   cv::imshow("progress",inputImage);
   double lastResid = 1000.;
   int loopTracker = 0;
   for( int iLoop = 0; iLoop < maxLoop; iLoop++){
      // convolve working image with kernel
      cv::Mat workingImage = cv::Mat::zeros(input->height,input->width,CV_32F);
      cv::GaussianBlur(filledImage,workingImage,kernSize,20.);
      // rebuild filledImage
      cv::Mat accumulator = cv::Mat::zeros(input->height,input->width,CV_32F);
      cv::accumulate(inputImage,accumulator,bitMask);
      cv::accumulate(workingImage,accumulator,notMask);
     double resid = cv::norm(accumulator,filledImage);
     //if (resid < 1. || pow((lastResid - resid),2.)/(resid*resid+1e-12) < tolerance){
     if (pow((lastResid - resid),2.)/(resid*resid+1e-12) < tolerance){
          lastResid = resid;
          break;
     }
     filledImage = accumulator.clone();
     lastResid = resid;
     loopTracker++;
   }
  std::cout<<"residual: "<<lastResid<<" # iterations: "<<loopTracker<<std::endl;
  double min;
  double max;
  cv::minMaxIdx(filledImage,&min,&max);
  cv::Mat scaledMap;
  cv::convertScaleAbs(filledImage,scaledMap,255/max);
  cv::Mat outputImage;
  scaledMap.convertTo(outputImage,CV_8U);
  if (maskMask)
     return bitMask;
  else
     return outputImage;



}
/*
cv::Mat displayMultipleImages(int nHeight, int nWidth, std::vector<cv::Mat> images){

  if (images.size() > nHeight*nWidth)
     std::cout<< "Too many images. truncating"<<std::endl;

  cv::Mat megaImage;
  int szx = 1000;
  int szy = 1000;
  megaImage.create(szs,szy,CV_8UC1);
  int m = 0;
  int n = 0;
    for (int ii = 0; ii<nHeight; ii++, m+=szx/nHeight) {
       for (int jj=0; jj<nWidth; jj++, n+= szy/nWidth) {
          int x = images[nWidth*ii + jj].cols;
          int y = images[nWidth*ii + jj].rows;
          int max = (x > y)? x: y;
          float scale = (float)( (float) max/size)
          cv::Rect roi 
          cv::SetImageROI(megaImage, cv::Rect(n, m, (int)( x/scale ), (int)( y/scale )));
          

       }
    }
}
*/
