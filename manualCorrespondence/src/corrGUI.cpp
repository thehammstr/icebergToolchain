#include "corrGUI.h"
#include "../build/ui_corrGUI.h"
#include <cloudManipulation.h>

#define STRIDE 5

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
  subcloud1a.reset( new PointCloudT);
  subcloud2a.reset( new PointCloudT);
  
  Xform = Eigen::Matrix4f::Identity();
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
  icpHasBeenRun = false;
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
  connect (ui->pushButton_delete, SIGNAL (clicked ()), this, SLOT (deleteButtonPressed ()));
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
  viewer->addPointCloud(subcloud1a,"subcloud1a");
  viewer->addPointCloud(subcloud2a,"subcloud2a");
  //showSubCloudExtents();
  viewer2->addPointCloud (subcloud1,"subcloud1");
  viewer2->addPointCloud (subcloud2,"subcloud2");
  //viewer->resetCamera ();
  viewer->setCameraPosition(2510.61, -1639.52, -3501.59,-0.825261, 0.0649858, -0.561001,0);
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
  path.addConstBias(.00004);
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
  unsigned char color = 0;
  unsigned char brightness = 100;
  unsigned long colorstep = ceil(path.poses.size()/brightness) + 1;
  for (int iT = 0; iT<path.poses.size(); iT ++)
  {
     trajectory->points[iT].x = path.poses[iT].xEst();
     trajectory->points[iT].y = path.poses[iT].yEst();
     trajectory->points[iT].z = path.poses[iT].zEst();
     trajectory->points[iT].r = 200;
     trajectory->points[iT].g = 00;
     trajectory->points[iT].b = 200;

     for (int jT = 0; jT < path.poses[iT].measurements.size(); jT += STRIDE)
     {
        FeatureNode feature_j (path.poses[iT],path.poses[iT].measurements[jT]);
        PointT featurePoint;
        featurePoint.x = feature_j.state[0];
        featurePoint.y = feature_j.state[1];
        featurePoint.z = feature_j.state[2];
        
        featurePoint.r = color;
        featurePoint.g = brightness-(color%(brightness/2));
        featurePoint.b = brightness-color;
        cloud->points.push_back(featurePoint);

     }
     if(!(iT%colorstep))
        color++;
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
PCLViewer::showSubCloudExtents()
{
  // color subclouds in window 1
  subcloud1a->clear();
  for (int iT = idx1-halfwidth; iT<idx1+halfwidth; iT ++)
  {
    for (int jT = 0; jT < path.poses[iT].measurements.size(); jT += 5*STRIDE)
     {
        FeatureNode feature_j (path.poses[iT],path.poses[iT].measurements[jT]);
        PointT featurePoint;
        featurePoint.x = feature_j.state[0];
        featurePoint.y = feature_j.state[1];
        featurePoint.z = feature_j.state[2];
        
        featurePoint.r = 200;
        featurePoint.g = 200;
        featurePoint.b = 00;
        subcloud1a->points.push_back(featurePoint);

     }

  }
  subcloud2a->clear();
  for (int iT = idx2-halfwidth; iT<idx2+halfwidth; iT ++)
  {
    for (int jT = 0; jT < path.poses[iT].measurements.size(); jT += 5*STRIDE)
     {
        FeatureNode feature_j (path.poses[iT],path.poses[iT].measurements[jT]);
        PointT featurePoint;
        featurePoint.x = feature_j.state[0];
        featurePoint.y = feature_j.state[1];
        featurePoint.z = feature_j.state[2];
        
        featurePoint.r = 200;
        featurePoint.g = 00;
        featurePoint.b = 00;
        subcloud2a->points.push_back(featurePoint);

     }

  }

  viewer->updatePointCloud(subcloud1a,"subcloud1a");
  viewer->updatePointCloud(subcloud2a,"subcloud2a");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "subcloud1a");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "subcloud2a");

}

void
PCLViewer::renderSubClouds()
{
  // create subclouds in window 2
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


  icpHasBeenRun = false;

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
  if (icpHasBeenRun){
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

  // RECORD Link INFO
  PoseLink link;
  link.idx1 = idx1;
  link.idx2 = idx2;
  int poo = 0;
  for (int iq = 0; iq < 4; iq++){
    for ( int jq = 0; jq < 4; jq++){
      link.Transform[poo] = Xform(iq,jq);
      poo++;
    }
  }
  validLinks.push_back(link);
  writeLinksToFile();
  } else {
    std::cout<<"ICP has not been run!\n";
  }
}

void
PCLViewer::writeLinksToFile()
{
  ofstream myfile;
  myfile.open("../../DATA/Soquel20121031/recordedLinks.csv");
  // print header
  myfile << " idx1, idx2, Transformationmatrix (4x4, row major)\n";
  // print data
  for (int ii = 0; ii<validLinks.size(); ii++){
    myfile << validLinks[ii].idx1 << "," << validLinks[ii].idx2 ;
    for (int jj = 0; jj < 16; jj++){
       myfile << ","<<validLinks[ii].Transform[jj];
    }
    myfile << std::endl;
  }
  myfile.close();
 
  return;
}
void
PCLViewer::deleteButtonPressed()
{
   printf ("delete button was pressed\n");

   if (validLinks.size()>0){
     // remove link depiction
     for (int ii = 0; ii<proposedLinks->points.size();ii++)
     {
       recordedLinks->points.pop_back();
     }
     viewer->updatePointCloud (recordedLinks, "recordedLinks");
     ui->qvtkWidget->update ();
     validLinks.pop_back();
     writeLinksToFile();
   } else {
     printf("nothing to delete\n");
   }
   return;
}

void
PCLViewer::runICP()
{
  pcl::IterativeClosestPointWithNormals<pcl::PointNormal,pcl::PointNormal> icp;
  //pcl::IterativeClosestPoint<PointT,PointT> icp;
  pcl::registration::TransformationEstimation2D<pcl::PointNormal, pcl::PointNormal>::Ptr 
        trans_2D (new pcl::registration::TransformationEstimation2D<pcl::PointNormal,pcl::PointNormal>);
  icp.setTransformationEstimation (trans_2D);
  pcl::registration::CorrespondenceRejectorOneToOne::Ptr one2one(new pcl::registration::CorrespondenceRejectorOneToOne);
  icp.setUseReciprocalCorrespondences(false); // setting this to true resulted in slower and worse results. boo. why?
  icp.setMaximumIterations(500);
  icp.setMaxCorrespondenceDistance(150.);
  std::cout << "max correspondence distance: "<< icp.getMaxCorrespondenceDistance() << std::endl;
  icp.addCorrespondenceRejector(one2one);

  // Need to make PointNormals out of XYZRGB
  pcl::PointCloud<pcl::PointNormal>::Ptr subcloud1n = addNorms(subcloud1,5);
  pcl::PointCloud<pcl::PointNormal>::Ptr subcloud2n = addNorms(subcloud2,5); 
  // run icp
  icp.setInputSource(subcloud2n);
  icp.setInputTarget(subcloud1n);
  pcl::PointCloud<pcl::PointNormal> Final;
  icp.align(Final);

  // put result back into submap
  pcl::transformPointCloud(*subcloud2,*subcloud2,Xform);

  std::cout << icp.getFinalTransformation() << std::endl;
  Xform = icp.getFinalTransformation();
  viewer2->updatePointCloud(subcloud1,"subcloud1");
  viewer2->updatePointCloud(subcloud2,"subcloud2");
  ui->qvtkWidget_2->update ();

  icpHasBeenRun = true; 
  return;
}

pcl::PointCloud<pcl::PointNormal>::Ptr 
PCLViewer::addNorms(pcl::PointCloud<PointT>::Ptr const subcloud,int sparsity)
{
  double radius = 1.5;
  pcl::NormalEstimation<PointT,pcl::Normal> ne;
  ne.setInputCloud(subcloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 
  ne.setRadiusSearch (radius);
  // Compute the features
  ne.compute (*cloud_normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal>);
  for (int ii = 0; ii<subcloud->points.size();ii+=sparsity)
  {
     pcl::PointNormal temp;
     temp.x = subcloud->points[ii].x;
     temp.y = subcloud->points[ii].y;
     temp.z = subcloud->points[ii].z;
     temp.normal_x = cloud_normals->points[ii].normal_x;
     temp.normal_y = cloud_normals->points[ii].normal_y;
     temp.normal_z = cloud_normals->points[ii].normal_z;
     output->points.push_back(temp);
  }

  return output;
}


void
PCLViewer::icpButtonPressed ()
{
  printf ("ICP button was pressed\n");
  runICP();
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
  showSubCloudExtents();
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
  showSubCloudExtents();
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
