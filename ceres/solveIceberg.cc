#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include "../Generic/CSVRow.h"
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "NodeDefinitions.h"
#include "ResidualBlockDefinitions.h"
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
    Eigen::Matrix3f m;
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    if (FLAGS_input.empty()) {
	LOG(ERROR) << "Usage: ./solveIceberg --input=DATA.CSV";
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
    while(file >> row)
    {	
	PoseNode pose(row);
	path.poses.push_back(pose);
	cout<< pose.inputs[0] << " " << pose.inputs[1] << endl;
	if (pose.measurements.size() > 0){
		Nobs = pose.measurements.back().featureIndex;
	}
    }
    	std::cout << Nobs << " observations." << std::endl;
	std::cout << path.poses.size() << " poses." << std::endl;
// update path with initial guess of bias
	//path.updateWithConstBias(-.00012);
	path.addConstBias(-.0012);
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
    path.PlotTrajectory();	
    // initialize problem
    ceres::Problem constMotionProblem;
    ceres::Problem problem;
    // Anchor the origin
    // For each time t
    for (int ii = 1; ii<path.poses.size(); ii++){
	ceres::CostFunction* odometry_cost_function = 
		OdometryError::Create(path.poses[ii-1],path.poses[ii]);
	// each time has odometry link
	ceres::LossFunction* loss_fxn = new ceres::HuberLoss(10.0);
	problem.AddResidualBlock(odometry_cost_function,
					NULL,
					path.poses[ii-1].state,
					path.poses[ii].state);
	// for each recorded measurement in the pose (usually zero)
	for (int jj = 0; jj<path.poses[ii].measurements.size(); jj++){
		int k = path.poses[ii].measurements[jj].featureIndex;
		ceres::CostFunction* meas_cost_function = 
		MeasurementError::Create(path.poses[ii],features[k],jj);
		ceres::LossFunction* loss2 = new ceres::HuberLoss(1.0);
		problem.AddResidualBlock(meas_cost_function,
					loss2,
					path.poses[ii].state,
					features[k].state);
	}
    }
    /* hold everything but bias constant in first block*/
    std::vector<int> const_subset;
    for (int isub = 0; isub<(_B_); isub++){ 
	const_subset.push_back(isub);}
	cout<< "subset " << const_subset.size()<<endl;
    ceres::SubsetParameterization* subset_parameterization = new ceres::SubsetParameterization(STATE_SIZE,const_subset);
    problem.SetParameterization(path.poses[0].state,subset_parameterization);
    //problem.SetParameterBlockConstant(path.poses[0].state); //TODO: can we free up bias[0] to be nonzero?
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR; //SPARSE_SCHUR;
    options.max_solver_time_in_seconds = 12800.;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 200;
    //options.max_num_iterations = 200;
    int actualMaxIter = 500;
    ceres::Solver::Summary summary;
    //for (int jj = 0; jj < actualMaxIter/options.max_num_iterations; jj++){
    	ceres::Solve(options, &problem, &summary);
    	path.PlotTrajectory();
//	if (summary.termination_type == ceres::CONVERGENCE){break;}
  //  }
    std::cout << summary.FullReport() << "\n";
    ofstream outfile;
    outfile.open("output.csv");	 
    //outfile << "x,y,z,u,v,w,phi,theta,psi,bias,"<<endl;
    for (int ii = 0 ; ii<path.poses.size();ii++){
	for (int jj = 0; jj<STATE_SIZE; jj++){
		outfile << path.poses[ii].state[jj] << ',';
	}
	for (int kk = 0; kk<2; kk++){
		outfile << path.poses[ii].inputs[kk] << ',';
	}

		outfile << std::endl;
		//cout << path.poses[ii].state[8]<< ' ' << path.poses[ii].state[9] <<endl;
	
    } 

}
