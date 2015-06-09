#ifndef CORRGUI_H
#define CORRGUI_H

#include <iostream>
#include <fstream>
// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/kdtree/kdtree_flann.h>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Local stuff
#include "CSVRow.h"
#include "NodeDefinitions.h"
#include "cloudManipulation.h"
#include <string>
#include <iostream>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

  void 
  loadTrajectory( char *  ); 
  void
  loadTrajectory(char *,const char *); // if link file provided by user

public slots:
  void
  writeButtonPressed ();

  void
  deleteButtonPressed ();

  void
  icpButtonPressed ();

  void
  RGBsliderReleased ();

  void
  pSliderValueChanged (int value);

  void
  redSliderValueChanged (int value);

  void
  greenSliderValueChanged (int value);

  void
  blueSliderValueChanged (int value);

  void 
  deleteSelected();

  void
  highlightLink(int value);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  PointCloudT::Ptr cloud;
  PointCloudT::Ptr trajectory;
  PointCloudT::Ptr subcloud1;
  PointCloudT::Ptr subcloud2;
  PointCloudT::Ptr subcloud1a;
  PointCloudT::Ptr subcloud2a;
  PointCloudT::Ptr interestPoints;
  PointCloudT::Ptr proposedLinks;
  PointCloudT::Ptr recordedLinks;
  Eigen::Matrix4f Xform;
  std::vector<PoseLink> validLinks;
  Trajectory path;
  std::string linkFile;
  int red;
  int green;
  int blue;
  int idx1;
  int rawSlider1;
  int idx2;
  int rawSlider2;
  int halfwidth;
  int selectedLink;
  bool icpHasBeenRun;
  
  void updateLinkDisplay();
  void renderSubClouds();
  void updateTrajectory();
  void showSubCloudExtents();
  void runICP();
  void runGridSearch();
  pcl::PointCloud<pcl::PointNormal>::Ptr 
       addNorms(pcl::PointCloud<PointT>::Ptr,int);
  void writeLinksToFile();
  void readLinksFromFile();
  void drawGoodLinks(); 
private:
  Ui::PCLViewer *ui;
  bool isValidInput(std::vector<std::string>);

};

#endif // PCLVIEWER_H
