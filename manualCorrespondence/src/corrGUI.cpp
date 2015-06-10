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
  idx1 = 100;
  idx2 = 201;
  halfwidth = 100;
  icpHasBeenRun = false;
  selectedLink = 0;
  renderTimer = new QTimer(this);
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

  // Connect buttons and the function
  connect (ui->pushButton_write,  SIGNAL (clicked ()), this, SLOT (writeButtonPressed ()));
  connect (ui->pushButton_delete, SIGNAL (clicked ()), this, SLOT (deleteButtonPressed ()));
  connect (ui->pushButton_ICP, SIGNAL (clicked ()), this, SLOT (icpButtonPressed ()));
  connect (ui->pushButton_delete_select, SIGNAL (clicked ()), this, SLOT (deleteSelected ()));
  connect (ui->pushButton_load_links, SIGNAL (clicked ()), this, SLOT (readLinksFromFile ()));
  connect (ui->pushButton_write_all, SIGNAL (clicked ()), this, SLOT (writeLinksToFile ()));

  // Connect text boxes
  connect (ui->plainTextEdit_linkfile, SIGNAL (textChanged ()), this, SLOT (fileNameChanged ()));
   
  // Connect sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
  // Connect spin box with function (select link)
  connect (ui->spinBox_link, SIGNAL (valueChanged (int)), this, SLOT (highlightLink (int) ));
  // This thing doesn't do anything right now.
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));
  // Connect rendering timer for clicking through indices
  connect (renderTimer,SIGNAL (timeout()), this, SLOT (renderSubClouds() ));
  // initialize pointclouds
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
  const char * linkFile_name = "recordedLinks.csv";
  std::cout<< "Using default link file name: recordedLinks.csv\n";
  loadTrajectory(filename, linkFile_name);

}

void 
PCLViewer::loadTrajectory( char * filename, const char * linkFileName  )
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

  // Link file stuff
  linkFile = linkFileName;
  readLinksFromFile();

  //path.addConstBias(.00004);
  // build trajectory
  updateTrajectory();
  //renderSubClouds();
  //showSubCloudExtents();
   viewer->updatePointCloud (cloud, "cloud");
   viewer->updatePointCloud (trajectory, "trajectory");
   viewer->resetCamera ();
   ui->qvtkWidget->update ();

   renderSubClouds();
   viewer2->resetCamera ();
   ui->qvtkWidget_2->update ();
   drawGoodLinks(); 
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
  std::cout<< "idx1: " << idx1 << ", idx2: "<<idx2<<std::endl;

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
  renderTimer->stop();
  // create subclouds in window 2
  pcl::PointCloud<pcl::PointXYZ>::Ptr subSimpleCloud1;
  subSimpleCloud1 = path.ExtractSubcloudAtAlt(idx1-halfwidth,idx1+halfwidth); 
  //subSimpleCloud1 = path.ExtractSubcloud(idx1-halfwidth,idx1+halfwidth); 
  subcloud1->points.resize(subSimpleCloud1->points.size());
  for (int ii = 0; ii< subSimpleCloud1->points.size();ii++)
  {
    subcloud1->points[ii].x = subSimpleCloud1->points[ii].x;
    subcloud1->points[ii].y = subSimpleCloud1->points[ii].y;
    subcloud1->points[ii].z = subSimpleCloud1->points[ii].z;
    subcloud1->points[ii].r = 200;
    subcloud1->points[ii].g = 200;
  }
  pcl::PointXYZRGBA origin;
  origin.x = 0.;
  origin.y = 0.;
  origin.z = 0.;
  origin.r = 0;
  origin.g = 0;
  origin.b = 255;
  subcloud1->points.push_back(origin);
  pcl::PointCloud<pcl::PointXYZ>::Ptr subSimpleCloud2;
  subSimpleCloud2 = path.ExtractSubcloudAtAlt(idx2-halfwidth,idx2+halfwidth); 
  //subSimpleCloud2 = path.ExtractSubcloud(idx2-halfwidth,idx2+halfwidth); 
  subcloud2->points.resize(subSimpleCloud2->points.size());
  for (int ii = 0; ii< subSimpleCloud2->points.size();ii++)
  {
    subcloud2->points[ii].x = subSimpleCloud2->points[ii].x;
    subcloud2->points[ii].y = subSimpleCloud2->points[ii].y;
    subcloud2->points[ii].z = subSimpleCloud2->points[ii].z;
    subcloud2->points[ii].r = 200;
    subcloud2->points[ii].g = 00;
  }

  origin.r = 255;
  subcloud2->points.push_back(origin);
  icpHasBeenRun = false;
  Xform = Eigen::Matrix4f::Identity();

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
    // add z offset
    link.Transform[11] = path.poses[idx2].zEst() - path.poses[idx1].zEst();
  }
  validLinks.push_back(link);
  writeLinksToFile();
  // NOW DISPLAY LINKS
  drawGoodLinks();
  } else {
    std::cout<<"ICP has not been run!\n";
  }
}

void
PCLViewer::drawGoodLinks()
{
  recordedLinks->points.clear();

  //std::cout<< "Drawing good links!"<<std::endl;
  //std::cout << "Valid Links: " << validLinks.size() << std::endl;

  for (int iGood = 0; iGood < validLinks.size(); ++iGood){
    PointT pt1 = trajectory->points[validLinks[iGood].idx1];
    PointT pt2 = trajectory->points[validLinks[iGood].idx2];
    float numPoints = 1000;
    for (int iA = 0; iA<numPoints; iA++)
    {
     PointT goodPoint;
     goodPoint.x = pt1.x + (((float)iA)/(float)numPoints)*(pt2.x - pt1.x);
     goodPoint.y = pt1.y + (((float)iA)/(float)numPoints)*(pt2.y - pt1.y);
     goodPoint.z = pt1.z + (((float)iA)/(float)numPoints)*(pt2.z - pt1.z);
     if (iGood == selectedLink){
     goodPoint.r = 255;
     goodPoint.g = 0;
     goodPoint.b = 0;
     } else {
     goodPoint.r = 0;
     goodPoint.g = 255;
     goodPoint.b = 0;
     }
     recordedLinks->points.push_back(goodPoint);
    }
  viewer->updatePointCloud (recordedLinks, "recordedLinks");
  ui->qvtkWidget->update ();
  } 
}

void 
PCLViewer::deleteSelected()
{
   printf ("delete button was pressed\n");

   if (!validLinks.empty()){
     validLinks.erase(validLinks.begin() + selectedLink);
     if (selectedLink >= validLinks.size() ){
        selectedLink = validLinks.size() - 1;
     }
     highlightLink(selectedLink);
     writeLinksToFile();
     drawGoodLinks();
   } else {
     printf("nothing to delete\n");
   }
   return;
}

void 
PCLViewer::highlightLink(int link)
{
  if (link < 0 || link >= validLinks.size() ){
    std::cout << "Warning: selected number not in range" << std::endl;
    std::cout << "There are only "<< validLinks.size() << " links."<<std::endl;
    ui->spinBox_link->setValue(selectedLink);
  } else {
    selectedLink = link;
    ui->spinBox_link->setValue(selectedLink);
    drawGoodLinks();
  }
}

void
PCLViewer::writeLinksToFile()
{
  ofstream myfile;
  //myfile.open("../../DATA/Soquel20121031/recordedLinks.csv");
  myfile.open(linkFile.c_str());
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
  std::cout<< validLinks.size()<< " links written to " << linkFile << std::endl; 
  return;
}

void
PCLViewer::readLinksFromFile()
{
 std::string line;
 ifstream myfile;
 myfile.open(linkFile.c_str());
 if (myfile.is_open())
 {
   // burn off header
   std::getline(myfile,line);
   int numInputs = 0;
   while (std::getline(myfile,line))
   {
      if (line.size()){
        // std::cout << line.size() << std::endl;
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> parsedInput;
        while (std::getline(ss,token,','))
        {
          parsedInput.push_back(token);
        }
        if (isValidInput(parsedInput)){
          // load it into PoseLink
          PoseLink inputLink;
          inputLink.idx1 = std::atoi(parsedInput[0].c_str());
          inputLink.idx2 = std::atoi(parsedInput[1].c_str());
          for (int iLink = 0; iLink < 16; ++iLink){
             inputLink.Transform[iLink] = std::atof(parsedInput[iLink+2].c_str());
          }
          std::cout << "Link!\n";
          // check for duplicate record
          // bool indicating that an input record is a duplicate
          bool doop = false;
          for (int icheck = 0; icheck < validLinks.size(); ++icheck){
             // if both indices match, then record is a duplicate
             if (inputLink.idx1 == validLinks[icheck].idx1 && inputLink.idx2 == validLinks[icheck].idx2)
               doop = true;
          }
          if (!doop){
            //only add unique links to record
            validLinks.push_back(inputLink);
            ++numInputs;
          }
        }
      }
   }

   std::cout<<"Loaded " << numInputs << " new links" << std::endl;
   myfile.close();

 } else {
   std::cout << "Could not locate file " << linkFile <<std::endl;
 }
 
 displayFileName(); 
 drawGoodLinks();
 return;
}

bool
PCLViewer::isValidInput(std::vector<std::string> parsed_input)
{
  // need 18 entries per line
  if (parsed_input.size() == 18 && std::atoi(parsed_input[0].c_str()) )
  {
    int idx1 = std::atoi(parsed_input[0].c_str());
    int idx2 = std::atoi(parsed_input[1].c_str());
    // indices must be in range and idx1 <= idx2
    if (idx1 >= 0 && idx1 <= idx2 && idx2 <= path.poses.size())
       return true;
  } 

  return false;
}


void
PCLViewer::displayFileName()
{
  QString outName = linkFile.c_str();
  //ui->plainTextEdit_linkfile->clear();
  ui->plainTextEdit_linkfile->setPlainText(outName);

}

void
PCLViewer::fileNameChanged()
{
  QString qs = ui->plainTextEdit_linkfile->toPlainText();
  // convert to string and set to linkfile
  linkFile.clear();
  linkFile = qs.toUtf8().constData();
  return;
}


void
PCLViewer::deleteButtonPressed()
{
   printf ("delete button was pressed\n");

   if (!validLinks.empty()){
     validLinks.pop_back();
     if (selectedLink >= validLinks.size() ){
        selectedLink=validLinks.size()-1;
        highlightLink(selectedLink);
     }
     writeLinksToFile();
     drawGoodLinks();
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
  icp.setMaximumIterations(150);
  icp.setMaxCorrespondenceDistance(50.);
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

  Eigen::Matrix4f Tform = icp.getFinalTransformation();

  // put result back into submap
  pcl::transformPointCloud(*subcloud2,*subcloud2,Tform);

  std::cout << icp.getFinalTransformation() << std::endl;
  Xform = Tform*Xform;
  viewer2->updatePointCloud(subcloud1,"subcloud1");
  viewer2->updatePointCloud(subcloud2,"subcloud2");
  ui->qvtkWidget_2->update ();

  icpHasBeenRun = true; 
  return;
}

void
PCLViewer::runGridSearch()
{
  double acceptableRad = 50.;
  double psiRange = .15;
  double xRange = 10.;
  double yRange = 10.;
  int nSamples_T = 11;
  int nSamples_R = 11;
  double xHat,yHat,pHat;
  double bestCost = 1e8;
  double bestX,bextY,bestPsi;

  // working pointcloud   
  PointCloudT::Ptr tempcloud;
  tempcloud.reset(new PointCloudT);
  std::vector<int> nn_i (1);
  std::vector<float> nn_d (1);
  std::vector<int> nn_indices(subcloud1->points.size(),-1);
  std::vector<int> nn_dists(subcloud1->points.size());

  for (int ip = 0; ip<nSamples_R; ip++){
    pHat = -psiRange + 2.*(psiRange*(double)ip/((double)nSamples_R));
    for (int iy = 0; iy< nSamples_T; iy++){
      yHat = -yRange + 2.*(yRange*(double)iy/((double)nSamples_T));
      for (int ix = 0; ix< nSamples_T; ix++){
        xHat = -xRange + 2.*(xRange*(double)ix/((double)nSamples_T));
        // build transform object
        Eigen::Matrix4f Tform;
        Tform << cos(pHat), sin(pHat), 0., xHat,
                -sin(pHat), cos(pHat), 0., yHat,
                    0.    ,      0.  , 1., 0.,
                    0.    ,      0.  , 0., 1.;
        std::cout<< "dx= "<<xHat<< " dy= "<<yHat<<" dpsi= "<< pHat<<std::endl;
        // transform subcloud 2
        pcl::transformPointCloud(*subcloud2,*tempcloud,Tform);
        //----------------
        pcl::KdTreeFLANN<PointT> tree;
        tree.setInputCloud(subcloud1);
        std::vector<int> nn_i (1);
        std::vector<float> nn_d (1);
        std::vector<int> nn_indices(subcloud1->points.size(),-1);
        std::vector<int> nn_dists(subcloud1->points.size());
        // find all matches
        for (int idx = 0; idx<tempcloud->points.size(); idx++){
          PointT query = tempcloud->points[idx];
          int match = tree.nearestKSearch(query,1,nn_i,nn_d);
          if (match == 1){
             // record match if it's better than another index
             if (nn_indices[nn_i[0]] == -1){
                // first time seen
                nn_indices[nn_i[0]] = idx;
                nn_dists[nn_i[0]] = nn_d[0];
             } else { // if this is closer than last one
                if (nn_d[0] < nn_dists[nn_i[0]] ){
                   nn_indices[nn_i[0]] = idx;
                   nn_dists[nn_i[0]] = nn_d[0];
                }
             }
          }
        }
        // calculate score
        
        // record score
      }
    } 
  }
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
  //runGridSearch();
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
  //RGBsliderReleased();
  
  //ui->qvtkWidget->update ();
}

void
PCLViewer::redSliderValueChanged (int value)
{
  // start "watchdog" render timer
  renderTimer->start(1000);
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
  ui->lcdNumber_R->display(idx1);
  //RGBsliderReleased();

}

void
PCLViewer::greenSliderValueChanged (int value)
{
  // start "watchdog" render timer
  renderTimer->start(1000);
  rawSlider2 = value;
  float frac = (float)value / 100000.;
  idx2 = (int)( frac*path.poses.size() );
  if (idx2 < idx1)
     idx2 = idx1;
  if (idx2 >= path.poses.size() - halfwidth)
     idx2 = path.poses.size() - halfwidth -1;
  updateLinkDisplay();
  showSubCloudExtents();
  ui->lcdNumber_G->display(idx2);
  //RGBsliderReleased();
}

void
PCLViewer::blueSliderValueChanged (int value)
{
  halfwidth = value;
  // make sure things still valid
  redSliderValueChanged(rawSlider1);
  greenSliderValueChanged(rawSlider2);
  //RGBsliderReleased();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
