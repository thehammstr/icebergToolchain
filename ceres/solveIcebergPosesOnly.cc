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

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

DEFINE_string(input, "", "Input File name");
DEFINE_string(links, "", "Link File name");

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
	LOG(ERROR) << "Usage: ./solveIceberg --input=DATA.CSV --links=LinkFile.csv";
    	return 1;
    }
    std::ifstream       file(FLAGS_input.c_str());
    std::ifstream       linkfile(FLAGS_links.c_str());
    CSVRow              row;
    CSVRow              linkrow;
    Trajectory path;
    std::vector<PoseLink> Links;
    path.plotDuration = 10.;
    //std::vector<PoseNode> poses;
    std::vector<FeatureNode> features;
    std::vector<int> featureIndices;
    // Logging stuff
    int Nobs = 0;
    int Nposes = 0;
    int NposesToUse = 230000;
    file >> row;
	/*for (int jj=0;jj<row.size();++jj){
        std::cout << row[jj] << "\t";
	}
	cout << endl;*/
    while(file >> row)
    {	Nposes++;
        if (Nposes < NposesToUse){
	   PoseNode pose(row);
	   path.poses.push_back(pose);
	   //cout<< pose.inputs[0] << " " << pose.inputs[1] << endl;
	   if (pose.measurements.size() > 0){
	   	Nobs = pose.measurements.back().featureIndex;
	   }
        }
    }
    	std::cout << Nobs << " observations." << std::endl;
	std::cout << path.poses.size() << " poses." << std::endl;
    // burn header
    linkfile >> linkrow;
    while(linkfile>>linkrow)
    {
      PoseLink tempLink;
      tempLink.loadLink(linkrow);
      if (tempLink.idx1 < NposesToUse && tempLink.idx2 < NposesToUse){
         tempLink.t1 = path.poses[tempLink.idx1].time;
         tempLink.t2 = path.poses[tempLink.idx2].time;
         Links.push_back(tempLink);
      }
      std::cout<< "idx1: "<<tempLink.idx1<<", idx2: "<<tempLink.idx2;
      std::cout<<std::endl;

    }
// update path with initial guess of bias
	//path.updateWithConstBias(-.00012);
    path.PlotTrajectory(Links);	
	path.addConstBias(.0000);
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
    path.PlotTrajectory(Links);	
    
  double bestAvgVel = 0.;
  double bestCost = 1e12;
  double biasguess = 1.;

for (int jSweep = -80; jSweep < -90; jSweep ++ ){
    float iSweep = float(jSweep)*0.00001;
    // initialize problem
    ceres::Problem problem;
    double avgMotion = 0.;
    path.updateWithConstBias(iSweep); 
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
    }
    
    
    // add icp links
    for (int jj = 0; jj<Links.size(); jj++){
        double relativeWeight = 10.;
        ceres::CostFunction* icp_cost_function = 
                RegistrationError::Create(Links[jj],relativeWeight);
        ceres::LossFunction* loss_fxn = new ceres::HuberLoss(10.);
        problem.AddResidualBlock(icp_cost_function,
                                     loss_fxn,
                                     path.poses[Links[jj].idx1].state,
                                     path.poses[Links[jj].idx2].state);

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
    options.max_num_iterations = 50;
    ceres::Solver::Summary summary;
    //for (int jj = 0; jj < actualMaxIter/options.max_num_iterations; jj++){
    	ceres::Solve(options, &problem, &summary);
    	//path.PlotTrajectory();
        if (summary.final_cost < bestCost){
           bestCost = summary.final_cost;
           bestAvgVel = iSweep;
        }
        std::cout << summary.FullReport() << "\n";
        //std::cout<< " average motion: " << avgMotion <<std::endl;

  }
   std::cout<< "best initial cost: "<< bestCost<< "estimated avg speed: "<<bestAvgVel<<std::endl;
//	if (summary.termination_type == ceres::CONVERGENCE){break;}
  //  }
    //std::cout << summary.FullReport() << "\n";


// Now actually run ceres

    path.updateWithConstBias(bestAvgVel); 
        path.PlotTrajectory(Links);	
    // Anchor the origin
    // For each time t    
        ceres::Problem problem;

    for (int ii = 1; ii<path.poses.size(); ii++){
        
	ceres::CostFunction* odometry_cost_function = 
		OdometryError::Create(path.poses[ii-1],path.poses[ii]);
	// each time has odometry link
	ceres::LossFunction* loss_fxn = new ceres::HuberLoss(10.0);
	problem.AddResidualBlock(odometry_cost_function,
					loss_fxn,
					path.poses[ii-1].state,
					path.poses[ii].state);
    }
    
    // add icp links
    for (int jj = 0; jj<Links.size(); jj++){
        double relativeWeight = 1.;
        ceres::CostFunction* icp_cost_function = 
                RegistrationError::Create(Links[jj],relativeWeight);
        ceres::LossFunction* loss_fxn = new ceres::HuberLoss(10.);
        problem.AddResidualBlock(icp_cost_function,
                                     loss_fxn,
                                     path.poses[Links[jj].idx1].state,
                                     path.poses[Links[jj].idx2].state);

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
    options.max_num_iterations = 500;
    ceres::Solver::Summary summary;
    // SOLVE

    std::cout<< "link pose 1: "<< path.poses[Links[0].idx1].state[0];
    std::cout<< " " << path.poses[Links[0].idx1].state[1];
    std::cout<< " " << path.poses[Links[0].idx1].state[2];
    std::cout<< " " << path.poses[Links[0].idx1].state[3];
    std::cout<< std::endl;
    std::cout<< "link pose 2: "<< path.poses[Links[0].idx2].state[0];
    std::cout<< " " << path.poses[Links[0].idx2].state[1];
    std::cout<< " " << path.poses[Links[0].idx2].state[2];
    std::cout<< " " << path.poses[Links[0].idx2].state[3];

    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    path.plotDuration = -1.; // keep plot up indefintely


    std::cout<< "link pose 1: "<< path.poses[Links[0].idx1].state[0];
    std::cout<< " " << path.poses[Links[0].idx1].state[1];
    std::cout<< " " << path.poses[Links[0].idx1].state[2];
    std::cout<< " " << path.poses[Links[0].idx1].state[3];
    std::cout<< std::endl;
    std::cout<< "link pose 2: "<< path.poses[Links[0].idx2].state[0];
    std::cout<< " " << path.poses[Links[0].idx2].state[1];
    std::cout<< " " << path.poses[Links[0].idx2].state[2];
    std::cout<< " " << path.poses[Links[0].idx2].state[3];

    std::cout<< std::endl<< "xform: "; 
    for (int pp = 0; pp<16; pp++)
         std::cout<<Links[0].Transform[pp]<<" ";
    std::cout<<std::endl;

    path.PlotTrajectory(Links);
  
    ofstream outfile;
    outfile.open("output.csv");	 
    //outfile << "t,x,y,psi,bias,input1,input2"<<endl;
    for (int ii = 0 ; ii<path.poses.size();ii++){
                outfile << path.poses[ii].time << ',';
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
