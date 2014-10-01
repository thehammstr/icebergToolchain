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
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
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
    subcloud = path.ExtractSubcloud(2400,2580);


}
