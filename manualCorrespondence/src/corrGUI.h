#ifndef CORRGUI_H
#define CORRGUI_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Local stuff
#include "CSVRow.h"
#include "NodeDefinitions.h"
#include "cloudManipulation.h"

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

public slots:
  void
  writeButtonPressed ();

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

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;
  PointCloudT::Ptr cloud;
  PointCloudT::Ptr trajectory;
  PointCloudT::Ptr subcloud1;
  PointCloudT::Ptr subcloud2;
  PointCloudT::Ptr interestPoints;
  PointCloudT::Ptr proposedLinks;
  PointCloudT::Ptr recordedLinks;
  Trajectory path;
  int red;
  int green;
  int blue;
  int idx1;
  int rawSlider1;
  int idx2;
  int rawSlider2;
  int halfwidth;
  void updateLinkDisplay();
  void renderSubClouds();
  void updateTrajectory();

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
