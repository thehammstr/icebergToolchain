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
	path.addConstBias(-.00002);
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
    int HalfWidth = 200;

    //for (int iOut = 1000 + HalfWidth; iOut < 3000-HalfWidth; iOut+=2*HalfWidth){
    std::vector<Eigen::Vector3d> matchMetric;
    ofstream matchgoodness;
    matchgoodness.open("matches.csv");
    matchgoodness << "iOut, jOut, error" <<std::endl;
    for (int iOut = 1500 + HalfWidth; iOut < 20000-HalfWidth; iOut+=2*HalfWidth){
        // Extract reference pointcloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud1;
        subcloud1 = path.ExtractSubcloud(iOut - HalfWidth,iOut+HalfWidth);
        // calculate normals
        float scaleRadius = 1.0;
        float harrisRadius = 2.;
        float harrisThresh = .0008;
        pcl::PointCloud<pcl::Normal>::Ptr normals1 = getNormals(subcloud1,scaleRadius);
        pcl::PointCloud<pcl::PointNormal>::Ptr subcloud1WithNormals = addNormalsToCloud(subcloud1,normals1);
        // view
            for (int jOut = iOut+2*HalfWidth+9500; jOut < iOut+2*HalfWidth+10200;jOut+=HalfWidth){
             std::cout<<"iout, jout: "<<iOut<<" | "<<jOut<<std::endl; 
           //int jOut = iOut+HalfWidth;
               // extract comparison cloud
             pcl::PointCloud<pcl::PointXYZ>::Ptr subcloud2;
             subcloud2 = path.ExtractSubcloud(jOut - HalfWidth,jOut+HalfWidth);
             // normals
             pcl::PointCloud<pcl::Normal>::Ptr normals2 = getNormals(subcloud2,scaleRadius);
             pcl::PointCloud<pcl::PointNormal>::Ptr subcloud2WithNormals = addNormalsToCloud(subcloud2,normals2);
             if (VIEW_SPINNING_CLOUD){
               path.PlotProposedMatches(iOut,jOut,5.0);
               
               pcl::visualization::PCLVisualizer viewer ("3D Viewer");
               viewer.setSize(1100,1100);
               pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(subcloud1, 90, 0, 0);
               pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> kpcolor(subcloud2, 90, 80, 0);
               viewer.addPointCloud(subcloud1,pccolor,"Cloud1.png");
               viewer.addPointCloud(subcloud2,kpcolor,"Cloud2.png");
               viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Cloud1.png"); 
               viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Cloud2.png"); 
               Eigen::Vector4f initViewPoint(-240.,0.,50.,1.);
               Eigen::Vector4f focalPoint;
               pcl::compute3DCentroid(*subcloud1,focalPoint);
               focalPoint(2) += 30; // it was a little low
               Eigen::Matrix4f rotmat;
               float viewAngle = 0;
               int labelIdx = 0;
               std::string baseName ("gifs/cloudView");
               pcl::io::savePCDFileASCII("firstPass.pcd",*subcloud1);
               pcl::io::savePCDFileASCII("secondPass.pcd",*subcloud2);
               while (!viewer.wasStopped () )
               {   // make rotmat
                if (viewAngle < 6.28){
                   rotmat << cos(viewAngle),sin(viewAngle),0.,focalPoint(0),
                               -sin(viewAngle),cos(viewAngle),0.,focalPoint(1),
                                 0.,0.,1., focalPoint(2),
                                 0.,0.,0.,1.;
                   viewAngle+=.03;
                   //Eigen::Vector4f viewPoint = rotmat*(initViewPoint-focalPoint) ;
                   Eigen::Vector4f viewPoint = rotmat*(initViewPoint) ;
                   viewer.setCameraPosition(viewPoint(0),viewPoint(1),viewPoint(2),focalPoint(0),focalPoint(1),focalPoint(2),0.,0.,1.);
                   }
                   viewer.spinOnce();
                   if (SAVE_IMAGES){
                       viewer.saveScreenshot(baseName + boost::lexical_cast<std::string>(labelIdx) + ".png");
                   } else {
                   pcl_sleep (.4);}
                   labelIdx++;
               } 
               viewer.close();
              /*
              pcl::Normal viewDir1 = avgNormal(normals1);
              pcl::Normal viewDir2 = avgNormal(normals2);
              cv::Mat rangeImage1 = imageFromCloudInDirection(subcloud1,viewDir1,.5,.01);
              cv::Mat rangeImage2 = imageFromCloudInDirection(subcloud2,viewDir2,.5,.01);
              cv::namedWindow("image1", CV_WINDOW_AUTOSIZE);
              cv::imshow("image1", rangeImage1);
              cv::namedWindow("image2", CV_WINDOW_AUTOSIZE);
              cv::imshow("image2", rangeImage2);
              cv::waitKey(5.); 
              */
              pcl_sleep (.3); 
             }
         }
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //pcl::io::savePCDFileASCII("fpfh.pcl",*fpfhs);
    //pcl::io::savePCDFileASCII("subcloudWithNormals.pcd",*cloudWithNormal);
   }
}
