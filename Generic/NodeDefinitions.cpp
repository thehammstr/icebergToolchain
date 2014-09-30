// NodeDefinitions.cpp
#include "NodeDefinitions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <time.h>
ResonMeasurement::ResonMeasurement(int idx, double x,double y, double z, double n){
	state[0] = x;
	state[1] = y;
	state[2] = z;
	state[3] = n;
	featureIndex = idx;
}
double ResonMeasurement::mx(){ return state[0];}
double ResonMeasurement::my(){ return state[1];}
double ResonMeasurement::mz(){ return state[2];}
double ResonMeasurement::ni(){ return state[3];}

PoseNode::PoseNode(CSVRow row){
//time - x - y - z - phi - theta - psi - commanded speed - omega - dvl flag - dvl - measurements (idx, poseidx, 4dof)
// min row length is 13. Any larger and it must be measurements
	stateSize = STATE_SIZE;
	time = atof(row[0].c_str());
	if (atoi(row[9].c_str()) == 1){
	  DVLflag = true;
	}else { 
          DVLflag = false;}
	DVL[0]    = atof(row[10].c_str());
	DVL[1]    = atof(row[11].c_str());
	DVL[2]    = atof(row[12].c_str());
	/* Build state vector*/
	state[0] = atof(row[1].c_str());
	state[1] = atof(row[2].c_str());
	state[2] = atof(row[6].c_str());
	state[3] = 0.;
	// inputs  [omega_z, DVL, z, pitch, roll]
	inputs[0] = atof(row[8].c_str()); // w_z
	inputs[1] = DVL[0]; 
	inputs[2] = DVL[1]; 
	inputs[3] = DVL[2]; 
	inputs[4] = atof(row[3].c_str()); //z	
	inputs[5] = atof(row[4].c_str()); // phi	
	inputs[6] = atof(row[5].c_str()); // theta	
	for (int jj = 13; jj<row.size();jj = jj+5){
		
		ResonMeasurement meas(atoi(row[jj].c_str()),
                                      atof(row[jj+1].c_str()),
                                      atof(row[jj+2].c_str()),
                                      atof(row[jj+3].c_str()),
                                      atof(row[jj+4].c_str()));
                meas.poseIdx = -1; 
		measurements.push_back(meas);
	} 

}

double PoseNode::xEst() const {return state[0];}
double PoseNode::yEst() const {return state[1];}
double PoseNode::zEst() const {return inputs[4];}
double PoseNode::uEst() const {return DVL[0];}
double PoseNode::vEst() const {return DVL[1];}
double PoseNode::wEst() const {return DVL[2];}
double PoseNode::phiEst() const {return inputs[5];}
double PoseNode::thetaEst() const {return inputs[6];}
double PoseNode::psiEst() const {return state[2];}
double PoseNode::biasEst() const {return state[3];}

Trajectory::Trajectory(void){
	return;

}

Trajectory::Trajectory(const std::vector<PoseNode> poses_in){
	poses = poses_in;
	plotDuration = 1e6;

}

bool Trajectory::PlotTrajectory(void){

    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr featureCloud (new pcl::PointCloud<pcl::PointXYZ>);

   for (int ii = 0; ii<poses.size(); ii++){
	pcl::PointXYZ basic_point;
	basic_point.x = poses[ii].yEst();
	basic_point.y = poses[ii].xEst();
	basic_point.z = -poses[ii].zEst();	
	basic_cloud_ptr->points.push_back(basic_point);
	for (int jj = 0; jj<poses[ii].measurements.size(); jj++){
		FeatureNode feature_j(poses[ii],poses[ii].measurements[jj]);
		pcl::PointXYZ featurePoint;
		featurePoint.x = feature_j.state[1];
		featurePoint.y = feature_j.state[0];
		featurePoint.z = -feature_j.state[2];
		featureCloud->points.push_back(featurePoint);
		

	}


    }
  basic_cloud_ptr->width = 1;
  basic_cloud_ptr->height = basic_cloud_ptr->points.size();
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr, "trajectory");
  viewer->addPointCloud<pcl::PointXYZ> (featureCloud, "features");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "trajectory");
  viewer->addCoordinateSystem (1.0);
  viewer->setCameraPosition(1312.78,565.839,1102.13,943.747,725.283,265.458, -0.828261,0.355559,0.433083);
  viewer->setCameraFieldOfView(0.8575);
  viewer->setCameraClipDistances(86.7214, 3962.83);
  viewer->setPosition(66,52);
  viewer->setSize(1280,512); 
  time_t startTime;
  startTime = time(NULL);
  //while (!viewer->wasStopped () && time(NULL) - startTime < plotDuration)
  while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100));
     }
  //viewer->close();

  std::cout << "width: " << basic_cloud_ptr->width << ", height:  "<< basic_cloud_ptr->height<<std::endl;
  pcl::io::savePCDFileASCII("output_traj.pcd",*basic_cloud_ptr);

  return true;
}

void Trajectory::updateWithConstBias(double bias){
// update path with initial guess of bias
	for (int i = 1; i<poses.size(); i++){
		double R[3][3];
		double worldVel[3];
		double AngleAxis[3];
		double dT = poses[i].time - poses[i-1].time;
		poses[i-1].inputs[0] += bias;
		poses[i-1].state[_B_] = bias;
		poses[i].state[_PSI_] = poses[i-1].psiEst() + (poses[i-1].inputs[0] - poses[i-1].biasEst())*dT;
		//poses[i].state[_PSI_] = poses[i-1].psiEst() + (poses[i-1].inputs[0] )*dT;
		AngleAxis[0] = 0.;
		AngleAxis[1] = 0.;
		AngleAxis[2] = poses[i-1].psiEst();
		double bodyVel[3];
		bodyVel[0] = poses[i-1].uEst();
		bodyVel[1] = poses[i-1].vEst();
		bodyVel[2] = poses[i-1].wEst();
		ceres::AngleAxisRotatePoint(AngleAxis,bodyVel,worldVel);
		poses[i].state[_X_] = poses[i-1].xEst() + worldVel[0]*dT;
		poses[i].state[_Y_] = poses[i-1].yEst() + worldVel[1]*dT;
		//poses[i].state[_Z_] = poses[i-1].state[_Z_] + worldVel[2]*dT;
		
	}
	return;
}

void Trajectory::addConstBias(double bias){
// update path with initial guess of bias
	for (int i = 1; i<poses.size(); i++){
		double R[3][3];
		double worldVel[3];
		double AngleAxis[3];
		double dT = poses[i].time - poses[i-1].time;
		poses[i-1].inputs[0] += bias;
		poses[i].state[_PSI_] = poses[i-1].psiEst() + (poses[i-1].inputs[0] - poses[i-1].biasEst())*dT;
		//poses[i].state[_PSI_] = poses[i-1].psiEst() + (poses[i-1].inputs[0] )*dT;
		AngleAxis[0] = 0.;
		AngleAxis[1] = 0.;
		AngleAxis[2] = poses[i-1].psiEst();
		double bodyVel[3];
		bodyVel[0] = poses[i-1].uEst();
		bodyVel[1] = poses[i-1].vEst();
		bodyVel[2] = poses[i-1].wEst();
		ceres::AngleAxisRotatePoint(AngleAxis,bodyVel,worldVel);
		poses[i].state[_X_] = poses[i-1].xEst() + worldVel[0]*dT;
		poses[i].state[_Y_] = poses[i-1].yEst() + worldVel[1]*dT;
		//poses[i].state[_Z_] = poses[i-1].state[_Z_] + worldVel[2]*dT;
		
	}
	return;
}

FeatureNode::FeatureNode(PoseNode Pose, ResonMeasurement Meas){
	/* Take position estimate and body frame vector measurement and initialize
	a map feature */
	double measWF[3];
	double AngleAxis[3];	
	AngleAxis[0] = 0.;
	AngleAxis[1] = 0.;
	AngleAxis[2] = Pose.psiEst();
	/* Rotate the measurement by the heading estimate */
	ceres::AngleAxisRotatePoint(AngleAxis,Meas.state,measWF);
	state[0] = Pose.xEst() + measWF[0];
	state[1] = Pose.yEst() + measWF[1];
	state[2] = Pose.zEst() + measWF[2];
	/* Record normal inclination */
	state[3] = Meas.state[3];
	/* Which feature? */
	featureIndex = Meas.featureIndex;

}


