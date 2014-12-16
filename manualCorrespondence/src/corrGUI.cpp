#include "corrGUI.h"
#include "../build/ui_corrGUI.h"
#include <cloudManipulation.h>

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  trajectory.reset (new PointCloudT);
  interestPoints.reset (new PointCloudT);
  proposedLinks.reset( new PointCloudT);
  recordedLinks.reset( new PointCloudT);
  subcloud1.reset( new PointCloudT);
  subcloud2.reset( new PointCloudT);

  // The number of points in the cloud
  cloud->points.resize (200);
  trajectory->points.resize(1000);
  interestPoints->points.resize(2);
  proposedLinks->points.resize(1000);
  // The default color
  red   = 128;
  green = 128;
  blue  = 128;
  idx1 = 500;
  idx2 = 10500;
  halfwidth = 100;

  // Fill the cloud with some points
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }

  // Set up the QVTK1 window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();
  // Set up the QVTK2 window
  viewer2.reset (new pcl::visualization::PCLVisualizer ("viewer2", false));
  ui->qvtkWidget_2->SetRenderWindow (viewer2->getRenderWindow ());
  viewer2->setupInteractor (ui->qvtkWidget_2->GetInteractor (), ui->qvtkWidget_2->GetRenderWindow ());
  ui->qvtkWidget_2->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_write,  SIGNAL (clicked ()), this, SLOT (writeButtonPressed ()));
  connect (ui->pushButton_ICP, SIGNAL (clicked ()), this, SLOT (icpButtonPressed ()));
  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer->addPointCloud (trajectory, "trajectory");
  viewer->addPointCloud (interestPoints,"interestPoints");
  viewer->addPointCloud (proposedLinks,"proposedLinks");
  viewer->addPointCloud (recordedLinks,"recordedLinks");
  viewer2->addPointCloud (subcloud1,"subcloud1");
  viewer2->addPointCloud (subcloud2,"subcloud2");
  viewer->resetCamera ();
  viewer2->resetCamera ();
  ui->qvtkWidget->update ();
}

void 
PCLViewer::loadTrajectory( char * filename )
{

   std::cout<<"loading file " << filename <<std::endl;
   std::ifstream file(filename);
   CSVRow row;
   path.plotDuration = 5.;
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
  path.addConstBias(.00001);
  // build trajectory
  updateTrajectory();

   viewer->updatePointCloud (cloud, "cloud");
   viewer->updatePointCloud (trajectory, "trajectory");
   updateLinkDisplay();
   viewer->resetCamera ();
   ui->qvtkWidget->update ();

   renderSubClouds();
   viewer2->resetCamera ();
   ui->qvtkWidget_2->update ();

   return;
}

void
PCLViewer::updateTrajectory()
{
  trajectory->points.resize(path.poses.size());
  cloud->clear();
  for (int iT = 0; iT<path.poses.size(); iT ++)
  {
     trajectory->points[iT].x = path.poses[iT].xEst();
     trajectory->points[iT].y = path.poses[iT].yEst();
     trajectory->points[iT].z = path.poses[iT].zEst();
     trajectory->points[iT].r = 200;
     trajectory->points[iT].g = 00;
     trajectory->points[iT].b = 00;

     for (int jT = 0; jT < path.poses[iT].measurements.size(); jT += 5)
     {
        FeatureNode feature_j (path.poses[iT],path.poses[iT].measurements[jT]);
        PointT featurePoint;
        featurePoint.x = feature_j.state[0];
        featurePoint.y = feature_j.state[1];
        featurePoint.z = feature_j.state[2];
        featurePoint.r = 0;
        featurePoint.g = 40;
        featurePoint.b = 40;
        cloud->points.push_back(featurePoint);

     }

  }

 return;
}

void
PCLViewer::updateLinkDisplay()
{
  PointT pt1 = trajectory->points[idx1];
  PointT pt2 = trajectory->points[idx2];
  float numPoints = proposedLinks->points.size();
  for (int iA = 0; iA<proposedLinks->points.size(); iA++)
  {
     proposedLinks->points[iA].x = pt1.x + (((float)iA)/numPoints)*(pt2.x - pt1.x);
     proposedLinks->points[iA].y = pt1.y + (((float)iA)/numPoints)*(pt2.y - pt1.y);
     proposedLinks->points[iA].z = pt1.z + (((float)iA)/numPoints)*(pt2.z - pt1.z);
     proposedLinks->points[iA].r = 255;
     proposedLinks->points[iA].g = 255;
     proposedLinks->points[iA].b = 255;
  }
  viewer->updatePointCloud (proposedLinks,"proposedLinks");
  ui->qvtkWidget->update ();

}

void
PCLViewer::renderSubClouds()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr subSimpleCloud1;
  subSimpleCloud1 = path.ExtractSubcloudAtAlt(idx1-halfwidth,idx1+halfwidth); 
  subcloud1->points.resize(subSimpleCloud1->points.size());
  for (int ii = 0; ii< subSimpleCloud1->points.size();ii++)
  {
    subcloud1->points[ii].x = subSimpleCloud1->points[ii].x;
    subcloud1->points[ii].y = subSimpleCloud1->points[ii].y;
    subcloud1->points[ii].z = subSimpleCloud1->points[ii].z;
    subcloud1->points[ii].r = 200;
    subcloud1->points[ii].g = 200;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr subSimpleCloud2;
  subSimpleCloud2 = path.ExtractSubcloudAtAlt(idx2-halfwidth,idx2+halfwidth); 
  subcloud2->points.resize(subSimpleCloud2->points.size());
  for (int ii = 0; ii< subSimpleCloud2->points.size();ii++)
  {
    subcloud2->points[ii].x = subSimpleCloud2->points[ii].x;
    subcloud2->points[ii].y = subSimpleCloud2->points[ii].y;
    subcloud2->points[ii].z = subSimpleCloud2->points[ii].z;
    subcloud2->points[ii].r = 200;
    subcloud2->points[ii].g = 00;
  }




  viewer2->updatePointCloud(subcloud1,"subcloud1");
  viewer2->updatePointCloud(subcloud2,"subcloud2");
  //viewer2->resetCamera();
  ui->qvtkWidget_2->update ();  
  return;
}

void
PCLViewer::writeButtonPressed ()
{
  printf ("write button was pressed\n");
  for (int ii = 0; ii<proposedLinks->points.size();ii++)
  {
     PointT goodpoint;
     goodpoint.x = proposedLinks->points[ii].x;
     goodpoint.y = proposedLinks->points[ii].y;
     goodpoint.z = proposedLinks->points[ii].z;
     goodpoint.r = 0;
     goodpoint.g = 255;
     goodpoint.b = 0;
     recordedLinks->points.push_back(goodpoint);
  }
  viewer->updatePointCloud (recordedLinks, "recordedLinks");
  ui->qvtkWidget->update ();
}

void
PCLViewer::icpButtonPressed ()
{
  printf ("ICP button was pressed\n");

  
  
  ui->qvtkWidget->update ();
}


void
PCLViewer::RGBsliderReleased ()
{
  // Set the new color
  /*for (size_t i = 0; i < cloud->size (); i++)
  {
    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }*/
  renderSubClouds();
  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::pSliderValueChanged (int value) 
{
  double bias = ((double)value)/10000.;
  //path.updateWithConstBias(bias);
  //updateTrajectory();
  viewer->updatePointCloud(cloud, "cloud");
  viewer->updatePointCloud(trajectory,"trajectory");
  ui->qvtkWidget->update ();
}

void
PCLViewer::redSliderValueChanged (int value)
{
  rawSlider1 = value;
  float frac = (float)value / 100000.;
  idx1 = (int)( frac*path.poses.size() );
  if (idx1 < halfwidth)
    idx1 = halfwidth+1;
  if (idx1 > idx2)
    idx1 = idx2;
  //printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
  updateLinkDisplay();
}

void
PCLViewer::greenSliderValueChanged (int value)
{
  rawSlider2 = value;
  float frac = (float)value / 100000.;
  idx2 = (int)( frac*path.poses.size() );
  if (idx2 < idx1)
     idx2 = idx1;
  if (idx2 >= path.poses.size() - halfwidth)
     idx2 = path.poses.size() - halfwidth -1;
  updateLinkDisplay();
}

void
PCLViewer::blueSliderValueChanged (int value)
{
  halfwidth = value;
  // make sure things still valid
  redSliderValueChanged(rawSlider1);
  greenSliderValueChanged(rawSlider2);
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
