// NodeDefinitions.cpp
#include "NodeDefinitions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <time.h>
#include <math.h>

void
PoseLink::loadLink(CSVRow row)
{
  idx1 = atoi(row[0].c_str());
  idx2 = atoi(row[1].c_str());
  for (int ii = 0; ii<16;ii++){
    Transform[ii] = atof(row[ii+2].c_str());
  }
  t1 = 0.;
  t2 = 10000.;
}


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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr featureCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr headingCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

     unsigned char color = 0;
     unsigned char brightness = 100;
     unsigned long colorstep = ceil(poses.size()/brightness) + 1;
   for (int ii = 0; ii<poses.size(); ii++){
	pcl::PointXYZRGB basic_point;
	basic_point.x = poses[ii].xEst();
	basic_point.y = poses[ii].yEst();
	basic_point.z = poses[ii].zEst();
	basic_point.r = 200;
	basic_point.g = 00;
	basic_point.b = 200;
	basic_cloud_ptr->points.push_back(basic_point);
        // show heading vector
        pcl::PointXYZRGB bodyX;
        bodyX.x = cos(poses[ii].state[_PSI_]);
        bodyX.y = sin(poses[ii].state[_PSI_]);
        bodyX.z = 0.;
        for (int poo = 0; poo<10;poo++){
           pcl::PointXYZRGB dot;
           dot.x = basic_point.x + double(poo)*.1*bodyX.x;
           dot.y = basic_point.y + double(poo)*.1*bodyX.y;
           dot.z = basic_point.z;
           dot.r = 155;
           dot.g = 0;
           dot.b = 0;
           headingCloud->points.push_back(dot);
        }
	for (int jj = 0; jj<poses[ii].measurements.size(); jj+=5){
		FeatureNode feature_j(poses[ii],poses[ii].measurements[jj]);
		pcl::PointXYZRGB featurePoint;
		featurePoint.x = feature_j.state[0];
		featurePoint.y = feature_j.state[1];
		featurePoint.z = feature_j.state[2];
                featurePoint.r = color;
                featurePoint.g = brightness-(color%(brightness/2));
                featurePoint.b = brightness-color;
		featureCloud->points.push_back(featurePoint);
		

	}
       if( !(ii%colorstep))
            color++;

    }
  basic_cloud_ptr->width = 1;
  basic_cloud_ptr->height = basic_cloud_ptr->points.size();
  featureCloud->width = 1;
  featureCloud->height = featureCloud->points.size();  
  headingCloud->width = 1;
  headingCloud->height = headingCloud->points.size(); 

  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
  //viewer->setBackgroundColor (0, 0, 0);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(basic_cloud_ptr);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> featureColors(featureCloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> headingColors(headingCloud);

  viewer->addPointCloud<pcl::PointXYZRGB> (basic_cloud_ptr, rgb, "trajectory");
  viewer->addPointCloud<pcl::PointXYZRGB> (featureCloud, featureColors, "features");
  viewer->addPointCloud<pcl::PointXYZRGB> (headingCloud, headingColors, "headings");

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "trajectory");
  viewer->addCoordinateSystem (1.0);
  viewer->resetCamera();
  viewer->setCameraPosition(241.928, -249.692, 470.148,-0.646306, 0.489756, 0.585173,0); // oblique view on sim berg
  //viewer->setCameraPosition(1312.78,565.839,1102.13,943.747,725.283,265.458, -0.828261,0.355559,0.433083);
  //viewer->setCameraFieldOfView(0.8575);
  //viewer->setCameraClipDistances(86.7214, 3962.83);
  //viewer->setPosition(66,52);
  //viewer->setSize(1280,512); 
  time_t startTime;
  startTime = time(NULL);
  while ( (!viewer->wasStopped () && time(NULL) - startTime < plotDuration) || 
                  (!viewer->wasStopped () && plotDuration < 0.)  )
  //while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100));
     }
  //viewer->close();

  std::cout << "width: " << featureCloud->width << ", height:  "<< featureCloud->height<<std::endl;
  pcl::io::savePCDFileASCII("output_traj.pcd",*basic_cloud_ptr);
  pcl::io::savePCDFileASCII("output_cloud.pcd",*featureCloud);

  return true;
}



bool Trajectory::PlotTrajectory(std::vector<PoseLink> links){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr featureCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr headingCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr linkCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

     unsigned char color = 0;
     unsigned char brightness = 100;
     unsigned long colorstep = ceil(poses.size()/brightness) + 1;
   for (int ii = 0; ii<poses.size(); ii++){
	pcl::PointXYZRGB basic_point;
	basic_point.x = poses[ii].xEst();
	basic_point.y = poses[ii].yEst();
	basic_point.z = poses[ii].zEst();
	basic_point.r = 200;
	basic_point.g = 00;
	basic_point.b = 200;
	basic_cloud_ptr->points.push_back(basic_point);
        // show heading vector
        pcl::PointXYZRGB bodyX;
        bodyX.x = cos(poses[ii].state[_PSI_]);
        bodyX.y = sin(poses[ii].state[_PSI_]);
        bodyX.z = 0.;
        for (int poo = 0; poo<10;poo++){
           pcl::PointXYZRGB dot;
           dot.x = basic_point.x + double(poo)*.1*bodyX.x;
           dot.y = basic_point.y + double(poo)*.1*bodyX.y;
           dot.z = basic_point.z;
           dot.r = 155;
           dot.g = 0;
           dot.b = 0;
           headingCloud->points.push_back(dot);
        }
	for (int jj = 0; jj<poses[ii].measurements.size(); jj+=5){
		FeatureNode feature_j(poses[ii],poses[ii].measurements[jj]);
		pcl::PointXYZRGB featurePoint;
		featurePoint.x = feature_j.state[0];
		featurePoint.y = feature_j.state[1];
		featurePoint.z = feature_j.state[2];
                featurePoint.r = color;
                featurePoint.g = brightness-(color%(brightness/2));
                featurePoint.b = brightness-color;
		featureCloud->points.push_back(featurePoint);
		

	}
       if( !(ii%colorstep))
            color++;

    }
  int LLine = 80;
  for (int nn=0; nn<links.size(); nn++){
     Eigen::Matrix3f m;
     m = Eigen::AngleAxisf(poses[links[nn].idx1].psiEst(), Eigen::Vector3f::UnitZ());

     Eigen::Vector3f bodyoffset;
     Eigen::Vector3f worldoffset;
     bodyoffset << links[nn].Transform[3] , links[nn].Transform[7] , poses[links[nn].idx2].zEst() - poses[links[nn].idx1].zEst();
     //std::cout << "body offset: "<< bodyoffset;
     worldoffset = m*bodyoffset;
     //std::cout <<  ", world offset: " << worldoffset;
     for (int qline=0; qline<LLine; qline++){
        pcl::PointXYZRGB point;
        pcl::PointXYZRGB pointErr;
        point.x = poses[links[nn].idx1].xEst() + ((double)qline/(double)LLine)*(worldoffset(0)) ;
	point.y = poses[links[nn].idx1].yEst() + ((double)qline/(double)LLine)*(worldoffset(1)) ;
	point.z = poses[links[nn].idx1].zEst() + ((double)qline/(double)LLine)*(worldoffset(2)) ;
        pointErr.x = poses[links[nn].idx2].xEst() + ((double)qline/(double)LLine)*
                                                    (poses[links[nn].idx1].xEst() + worldoffset(0) - poses[links[nn].idx2].xEst()) ;
	pointErr.y = poses[links[nn].idx2].yEst() + ((double)qline/(double)LLine)*
                                                    (poses[links[nn].idx1].yEst() + worldoffset(1) - poses[links[nn].idx2].yEst()) ;
	pointErr.z = poses[links[nn].idx2].zEst() + ((double)qline/(double)LLine)*
                                                    (poses[links[nn].idx1].zEst() + worldoffset(2) - poses[links[nn].idx2].zEst()) ;
	point.r = 00;
	point.g = 250;
	point.b = 00;
	pointErr.r = 250;
	pointErr.g = 0;
	pointErr.b = 00;
	linkCloud->points.push_back(point);
        linkCloud->points.push_back(pointErr);
     }
     pcl::PointXYZRGB target; 
     target.x = poses[links[nn].idx2].xEst();
     target.y = poses[links[nn].idx2].yEst();
     target.z = poses[links[nn].idx2].zEst();
     linkCloud->points.push_back(target);
  }

  basic_cloud_ptr->width = 1;
  basic_cloud_ptr->height = basic_cloud_ptr->points.size();
  featureCloud->width = 1;
  featureCloud->height = featureCloud->points.size();  
  headingCloud->width = 1;
  headingCloud->height = headingCloud->points.size(); 
  linkCloud->width = 1;
  linkCloud->height = linkCloud->points.size();
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
  //viewer->setBackgroundColor (0, 0, 0);
pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(basic_cloud_ptr);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> featureColors(featureCloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> headingColors(headingCloud);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> linkColors(linkCloud);

  viewer->addPointCloud<pcl::PointXYZRGB> (basic_cloud_ptr, rgb, "trajectory");
  viewer->addPointCloud<pcl::PointXYZRGB> (featureCloud, featureColors, "features");
  viewer->addPointCloud<pcl::PointXYZRGB> (headingCloud, headingColors, "headings");
  viewer->addPointCloud<pcl::PointXYZRGB> (linkCloud, linkColors, "links");

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "trajectory");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "links");
  viewer->addCoordinateSystem (1.0);
  viewer->resetCamera();
  viewer->setCameraPosition(241.928, -249.692, 470.148,-0.646306, 0.489756, 0.585173,0); // oblique view on sim berg
  //viewer->setCameraPosition(211.61, -1478.52, -2093.59,-0.98675, -0.1545, -0.0561001,0);
  //viewer->setCameraPosition(1312.78,565.839,1102.13,943.747,725.283,265.458, -0.828261,0.355559,0.433083);
  //viewer->setCameraFieldOfView(0.8575);
  //viewer->setCameraClipDistances(86.7214, 3962.83);
  //viewer->setPosition(66,52);
  //viewer->setSize(1280,512); 
  time_t startTime;
  startTime = time(NULL);
  while ( (!viewer->wasStopped () && time(NULL) - startTime < plotDuration) || 
                  (!viewer->wasStopped () && plotDuration < 0.)  )
  //while (!viewer->wasStopped ())
     {
       viewer->spinOnce (100);
       boost::this_thread::sleep (boost::posix_time::microseconds (100));
     }
  //viewer->close();

  std::cout << "width: " << featureCloud->width << ", height:  "<< featureCloud->height<<std::endl;
  //pcl::io::savePCDFileASCII("output_traj.pcd",*basic_cloud_ptr);
  pcl::io::savePCDFileASCII("output_cloud.pcd",*featureCloud);

  return true;
}






void
Trajectory::serialize(std::string filename)
{
   std::cout << "saving trajectory to " << filename << std::endl; 
   std::ofstream file(filename.c_str());
   file << "Format: time - x - y - z - phi - theta - psi - commanded speed - omega - dvl flag - dvl - measurements (scan number 4dof)\n";
   for (int ii = 0; ii<poses.size(); ++ii){
      file << poses[ii].time       << "," <<
              poses[ii].xEst()     << "," <<
              poses[ii].yEst()     << "," <<
              poses[ii].zEst()     << "," <<
              poses[ii].phiEst()   << "," <<
              poses[ii].thetaEst() << "," <<
              poses[ii].psiEst()   << "," <<
              "1.5"                << "," <<  // commanded speed (not used)
              poses[ii].inputs[0] - poses[ii].biasEst()  << "," <<  // omega z - bias
              poses[ii].DVLflag    << "," <<  // dvl flag
              poses[ii].DVL[0]     << "," <<
              poses[ii].DVL[1]     << "," <<
              poses[ii].DVL[2];
              //std::cout << ii << " " << poses[ii].measurements.size() <<std::endl;
      for (int jj = 0; jj<poses[ii].measurements.size(); ++jj){
          file <<  "," << poses[ii].measurements[jj].featureIndex <<
                   "," << poses[ii].measurements[jj].mx()         <<
                   "," << poses[ii].measurements[jj].my()         <<
                   "," << poses[ii].measurements[jj].mz()         <<
                   "," << poses[ii].measurements[jj].ni();
      }
      file << std::endl;
   }
   file.close();
  /*try {
    pcl::io::savePCDFileASCII("output_traj.pcd",*basic_cloud_ptr);
    pcl::io::savePCDFileASCII("output_cloud.pcd",*featureCloud);
  } 
  catch {
    std::cout<< "couldn't save pcd files\n" <<std::endl;
  }*/

}


bool Trajectory::PlotProposedMatches(int idx1, int idx2, float disp_sec){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr line (new pcl::PointCloud<pcl::PointXYZRGB>);

   for (int ii = 0; ii<poses.size(); ii++){
	pcl::PointXYZRGB basic_point;
	basic_point.x = poses[ii].yEst();
	basic_point.y = poses[ii].xEst();
	basic_point.z = -poses[ii].zEst();
        if (ii == idx1 || ii == idx2){
           basic_point.r = 255;
           basic_point.g = 255;
           basic_point.b = 0;
        } else {	
           basic_point.r = 0;
           basic_point.g = 0;
           basic_point.b = 255;
	}
        basic_cloud_ptr->points.push_back(basic_point);
	

    }
  // build line
  for (float id = 0.; id<=1.; id += .01){
     pcl::PointXYZRGB linepoint;
     linepoint.x = poses[idx1].yEst() + id*(poses[idx2].yEst()-poses[idx1].yEst());
     linepoint.y = poses[idx1].xEst() + id*(poses[idx2].xEst()-poses[idx1].xEst());
     linepoint.z = -poses[idx1].zEst() - id*(poses[idx2].zEst()-poses[idx1].zEst());
     linepoint.r = 0;
     linepoint.g = 255;
     linepoint.b = 255;
     line->points.push_back(linepoint);
  }

  basic_cloud_ptr->width = 1;
  basic_cloud_ptr->height = basic_cloud_ptr->points.size();
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(basic_cloud_ptr);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> linecolor(line);
  viewer->addPointCloud<pcl::PointXYZRGB> (basic_cloud_ptr, rgb,"trajectory");
  viewer->addPointCloud<pcl::PointXYZRGB> (line, linecolor,"line");

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "trajectory");
  viewer->addCoordinateSystem (1.0);
  //viewer->setCameraPosition(1312.78,565.839,1102.13,943.747,725.283,265.458, -0.828261,0.355559,0.433083);
  //viewer->setCameraFieldOfView(0.8575);
  //viewer->setCameraClipDistances(86.7214, 3962.83);
  viewer->setPosition(66,52);
   time_t startTime;
  startTime = time(NULL);
  //while (!viewer->wasStopped () && time(NULL) - startTime < disp_sec)
  float timer = 0.;
  while (!viewer->wasStopped ()  && timer < disp_sec)
     {
       viewer->spinOnce (100);
       pcl_sleep (.1);
       timer+= .1;
     }
  viewer->close();

  //std::cout << "width: " << featureCloud->width << ", height:  "<< featureCloud->height<<std::endl;
  //pcl::io::savePCDFileASCII("output_traj.pcd",*basic_cloud_ptr);
  //pcl::io::savePCDFileASCII("output_cloud.pcd",*featureCloud);

  return true;
}




void Trajectory::updateWithConstBias(double bias){
// update path with initial guess of bias
	for (int i = 1; i<poses.size(); i++){
		double R[3][3];
		double worldVel[3];
		double AngleAxis[3];
		double dT = poses[i].time - poses[i-1].time;
		//poses[i-1].inputs[0] += bias;
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
// rebuild with bias
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

void Trajectory::addBias(void){
// rebuild with bias
// this is a hack way to do a non-constant bias. It was done in the 11th hour. Bias is hard-coded
        double EndTime = poses[poses.size()-1].time;
	for (int i = 1; i<poses.size(); i++){
		double R[3][3];
		double worldVel[3];
		double AngleAxis[3];
		double dT = poses[i].time - poses[i-1].time;
		poses[i-1].inputs[0] += -.00008788*sin((3.141592/EndTime)*poses[i-1].time); // maxes out at 18 deg/hr
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


pcl::PointCloud<pcl::PointXYZ>::Ptr Trajectory::ExtractSubcloudFixedWidth(int center, double mapWidth){
     // initialize output
     pcl::PointCloud<pcl::PointXYZ>::Ptr temp ;
     pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
     // calc indices
     int timeHalfWidth = 2.*mapWidth/poses[center].DVL[0]*.33; // 3 Hz data
     int firstIndex = std::max(center - timeHalfWidth,0);
     int lastIndex = std::min(center + timeHalfWidth, (int)poses.size()-1);
     // extract subcloud
     temp = ExtractSubcloud(firstIndex,lastIndex);
     // squared distance for comparison
     double refdistSq = mapWidth*mapWidth/4.;
     // go through and erase anything greater than refdist away in the horizontal
     for (int ii = 0; ii<temp->points.size(); ii++){
        double xx = temp->points[ii].x;
        double yy = temp->points[ii].y;
        if ((xx*xx) + (yy*yy) < refdistSq){
            // this is inefficient, but oh well.
            output->points.push_back(temp->points[ii]);
            
        } else {
            std::cout<< ii << " too far.\n";
        } 
              
     }

   return output;

}



pcl::PointCloud<pcl::PointXYZ>::Ptr Trajectory::ExtractSubcloud(int idx1, int idx2){



    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr featureCloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (idx2<=idx1 || idx1<0 || idx2<0 ||idx1>=poses.size() ||idx2>=poses.size()){
       std::cout<<"idx2 must be greater than idx1 and both must be within range of poses\n";
      return featureCloud;
    }


   for (int ii = idx1; ii<idx2; ii++){
	pcl::PointXYZ basic_point;
	basic_point.x = poses[ii].xEst();
	basic_point.y = poses[ii].yEst();
	basic_point.z = poses[ii].zEst();	
	basic_cloud_ptr->points.push_back(basic_point);
	for (int jj = 0; jj<poses[ii].measurements.size(); jj++){
		FeatureNode feature_j(poses[ii],poses[ii].measurements[jj]);
		pcl::PointXYZ featurePoint;
		featurePoint.x = feature_j.state[0];
		featurePoint.y = feature_j.state[1];
		featurePoint.z = feature_j.state[2];
		featureCloud->points.push_back(featurePoint);
		

	}


    }
  basic_cloud_ptr->width = 1;
  basic_cloud_ptr->height = basic_cloud_ptr->points.size();
  featureCloud->width = 1;
  featureCloud->height = featureCloud->points.size();


  std::cout << "width: " << basic_cloud_ptr->width << ", height:  "<< basic_cloud_ptr->height<<std::endl;
  // shift feature cloud so that its origin is the "middle" pose
  pcl::PointCloud<pcl::PointXYZ>::Ptr shiftedFeatureCloud (new pcl::PointCloud<pcl::PointXYZ>);
  // extract "middle" pose
  int refIdx = idx1 + (idx2-idx1)/2;
  // build transformation 
  Eigen::Affine3f xform2 = Eigen::Affine3f::Identity();
  // don't shift z
  xform2.translation() << -poses[refIdx].xEst(), -poses[refIdx].yEst(), -poses[refIdx].zEst();
  //std::cout<< poses[refIdx].yEst() << " " << poses[refIdx].xEst()<<" "<< -poses[refIdx].zEst()<<std::endl;
  // do it
  pcl::transformPointCloud(*featureCloud,*shiftedFeatureCloud,xform2);
  // and rotate
   // and rotate
  Eigen::Affine3f m;
  m = Eigen::AngleAxisf(-poses[refIdx].psiEst(), Eigen::Vector3f::UnitZ()); 
  pcl::transformPointCloud(*shiftedFeatureCloud,*shiftedFeatureCloud,m);
  // save it
  //pcl::io::savePCDFileASCII("subcloud.pcd",*shiftedFeatureCloud);

  return shiftedFeatureCloud;

}


pcl::PointCloud<pcl::PointXYZ>::Ptr Trajectory::ExtractSubcloudAtAlt(int idx1, int idx2){


    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr featureCloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (idx2<=idx1 || idx1<0 || idx2<0 ||idx1>=poses.size() ||idx2>=poses.size()){
       std::cout<<"idx2 must be greater than idx1 and both must be within range of poses\n";
      return featureCloud;
    }


   for (int ii = idx1; ii<idx2; ii++){
	pcl::PointXYZ basic_point;
	basic_point.x = poses[ii].xEst();
	basic_point.y = poses[ii].yEst();
	basic_point.z = poses[ii].zEst();	
	basic_cloud_ptr->points.push_back(basic_point);
	for (int jj = 0; jj<poses[ii].measurements.size(); jj++){
		FeatureNode feature_j(poses[ii],poses[ii].measurements[jj]);
		pcl::PointXYZ featurePoint;
		featurePoint.x = feature_j.state[0];
		featurePoint.y = feature_j.state[1];
		featurePoint.z = feature_j.state[2];
		featureCloud->points.push_back(featurePoint);
		

	}


    }
  basic_cloud_ptr->width = 1;
  basic_cloud_ptr->height = basic_cloud_ptr->points.size();
  featureCloud->width = 1;
  featureCloud->height = featureCloud->points.size();


  std::cout << "width: " << basic_cloud_ptr->width << ", height:  "<< basic_cloud_ptr->height<<std::endl;
  // shift feature cloud so that its origin is the "middle" pose
  pcl::PointCloud<pcl::PointXYZ>::Ptr shiftedFeatureCloud (new pcl::PointCloud<pcl::PointXYZ>);
  // extract "middle" pose
  int refIdx = idx1 + (idx2-idx1)/2;
  // build transformation 
  Eigen::Affine3f xform2 = Eigen::Affine3f::Identity();
  // don't shift z
  xform2.translation() << -poses[refIdx].xEst(), -poses[refIdx].yEst(), 0.;
  //std::cout<< poses[refIdx].yEst() << " " << poses[refIdx].xEst()<<" "<< -poses[refIdx].zEst()<<std::endl;
  // do it
  pcl::transformPointCloud(*featureCloud,*shiftedFeatureCloud,xform2);
  // and rotate
   // and rotate
  Eigen::Affine3f m;
  m = Eigen::AngleAxisf(-poses[refIdx].psiEst(), Eigen::Vector3f::UnitZ()); 
  pcl::transformPointCloud(*shiftedFeatureCloud,*shiftedFeatureCloud,m);
  // save it
  pcl::io::savePCDFileASCII("subcloud.pcd",*shiftedFeatureCloud);

  return shiftedFeatureCloud;
}

