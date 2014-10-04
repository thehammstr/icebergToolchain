#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <math.h>

#define PI 3.14159265

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
	path.addConstBias(-.00012);
/*-------------------------------------------------*/
/*----------- Now do something with the data-------*/
/*-------------------------------------------------*/
    // Visualize initial guess of trajectory
    int uniqueCounter = 0;
    /* Initialize feature locations */
    for (int ii = 0; ii<path.poses.size(); ii++){
	/* Push features into cloud */
	for (int jj = 0; jj<path.poses[ii].measurements.size(); jj++){
		featureIndices.push_back(uniqueCounter);
		if (path.poses[ii].measurements[jj].featureIndex > uniqueCounter){
			FeatureNode feature_j(path.poses[ii],path.poses[ii].measurements[jj]);
			features.push_back(feature_j);
			//uniqueCounter = path.poses[ii].measurements[jj].featureIndex;
			uniqueCounter++;
			//cout<< "***";
			path.poses[ii].measurements[jj].featureIndex = uniqueCounter;
		}
		//cout<<path.poses[ii].measurements[jj].featureIndex;
		//cout<<endl;
	}
    }
    std::cout<<uniqueCounter+1<<" unique features"<<std::endl;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //path.PlotTrajectory();	
    int HalfWidth = 100;

    //for (int iOut = 1000 + HalfWidth; iOut < 3000-HalfWidth; iOut+=2*HalfWidth){
    for (int iOut = 2000 + HalfWidth; iOut < 3000-HalfWidth; iOut+=HalfWidth){
        // Extract reference pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud1;
        subcloud1 = path.ExtractSubcloud(iOut - HalfWidth,iOut+HalfWidth);
        // calculate normals
        float scaleRadius = 1.;
        float harrisThresh = .003;
        pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals(subcloud1,scaleRadius);
        // view
        normalsVis(subcloud1,normals1); 
        // Extract Features from said pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr Harris1 = findHarrisCorners(subcloud1,normals1,scaleRadius*3.,harrisThresh);
        // build descriptors of keypoints
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors1 = calculateFPFHDescriptors(Harris1,subcloud1,normals1,scaleRadius*3);
        //for (int jOut = iOut+2*HalfWidth; jOut < 3000-HalfWidth;jOut+=2*HalfWidth){
        int jOut = iOut+HalfWidth;
            // extract comparison cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud2;
        subcloud2 = path.ExtractSubcloud(jOut - HalfWidth,jOut+HalfWidth);
        // normals
        pcl::PointCloud<pcl::Normal>::Ptr normals2 = getNormals(subcloud2,scaleRadius);
            // extract features
        pcl::PointCloud<pcl::PointXYZ>::Ptr Harris2 = findHarrisCorners(subcloud2,normals2,scaleRadius*3.,harrisThresh);
            
            pcl::visualization::PCLVisualizer viewer ("3D Viewer");
            viewer.setSize(1100,1100);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(subcloud1, 255, 0, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolorFeat(Harris1, 0, 255, 255);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpcolor(subcloud2, 255, 255, 0);
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpcolorFeat(Harris2, 255, 255, 0);
            viewer.addPointCloud(subcloud1,pccolor,"Cloud1.png");
            viewer.addPointCloud(Harris1,pccolorFeat,"Cloud1Features.png");
            //viewer.addPointCloud(subcloud2,kpcolor,"Cloud2.png");
            //viewer.addPointCloud(Harris2,kpcolorFeat,"Cloud2Features.png");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Cloud1Features.png"); 
            //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud2Features.png"); 
            
            Eigen::Vector4f initViewPoint(-240.,0.,50.,1.);
            Eigen::Vector4f focalPoint;
            pcl::compute3DCentroid(*subcloud1,focalPoint);
            focalPoint(2) += 30; // it was a little low
            Eigen::Matrix4f rotmat;
            float viewAngle = 0;
            int labelIdx = 0;
            std::string baseName ("gifs/cloudView");
            while (!viewer.wasStopped () && viewAngle < 6.28)
            {   // make rotmat
                rotmat << cos(viewAngle),sin(viewAngle),0.,focalPoint(0),
                            -sin(viewAngle),cos(viewAngle),0.,focalPoint(1),
                              0.,0.,1., focalPoint(2),
                              0.,0.,0.,1.;
                viewAngle+=.03;
                Eigen::Vector4f viewPoint = rotmat*(initViewPoint-focalPoint) ;
                viewer.setCameraPosition(viewPoint(0),viewPoint(1),viewPoint(2),focalPoint(0),focalPoint(1),focalPoint(2),0.,0.,1.);
                viewer.spinOnce();
                viewer.saveScreenshot(baseName + boost::lexical_cast<std::string>(labelIdx) + ".png");
                pcl_sleep (.4);
                std::cout<<iOut<<std::endl;
                labelIdx++;
            } 
           /* 

   
        }*/

    }
    
    //pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormal = addNormalsToCloud(subcloud1, normals1);

    //normalsVis(subcloud,normals);
    //show point cloud


    
/*
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  FPFH stuff
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (keypoints3D);
    fpfh.setInputNormals (normals);
    fpfh.setSearchSurface(subcloud);
    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);

    fpfh.setSearchMethod (tree1);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (2.*scaleRadius);

    // Compute the features
    fpfh.compute (*fpfhs);
    std::cout<<fpfhs->points.size()<<std::endl;
    pcl::visualization::PCLHistogramVisualizer hist;
    const std::string id="cloud";
    const std::string field= "fpfh"; 
    hist.setBackgroundColor(0.,0.,0.);
    //hist.addFeatureHistogram(*fpfhs,sizeof(fpfhs->points[0].histogram)/sizeof(fpfhs->points[0].histogram[0]),id);
    hist.addFeatureHistogram(*fpfhs,field,1,id);
    hist.spin();
    for (int ii = 0; ii < fpfhs->points.size(); ii++){
        hist.updateFeatureHistogram(*fpfhs,field,ii,id);
        hist.spinOnce(1000);
    } 
*/
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //pcl::io::savePCDFileASCII("fpfh.pcl",*fpfhs);
    //pcl::io::savePCDFileASCII("subcloudWithNormals.pcd",*cloudWithNormal);
}
