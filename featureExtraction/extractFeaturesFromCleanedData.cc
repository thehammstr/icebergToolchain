#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
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
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud;
    subcloud = path.ExtractSubcloud(2300,2580);
    float scaleRadius = 1.;
    pcl::PointCloud<pcl::Normal>::Ptr normals = getNormals(subcloud,scaleRadius);
    
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormal = addNormalsToCloud(subcloud, normals);

    normalsVis(subcloud,normals);
    /*
        HARRIS CORNERS
    */
    
    pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI,pcl::Normal> detector;
    detector.setNonMaxSupression (true);
    detector.setRadius (2.*scaleRadius);
    detector.setInputCloud(subcloud);
    detector.setNormals(normals);
    detector.setRefine(false);
    detector.setThreshold(.001);
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
            std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
            max = (*i).intensity;
        }
        if ((*i).intensity<min){
            min = (*i).intensity;
        }
        keypoints3D->push_back(tmp);
    }

    std::cout << "maximal response: "<< max << " min response:  "<< min<<std::endl;

    //show point cloud
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



    

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*  FPFH stuff
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (subcloud);
    fpfh.setInputNormals (normals);
    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (2.*scaleRadius);

    // Compute the features
    fpfh.compute (*fpfhs);
    normalsVis(subcloud,normals);*/
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::io::savePCDFileASCII("subcloudWithNormals.pcd",*cloudWithNormal);
}
