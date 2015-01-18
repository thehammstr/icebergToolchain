#ifndef NODEDEFS
#define NODEDEFS

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include <vector>
#include "CSVRow.h"
// PCL stuff
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <Eigen/StdVector>

#define STATE_SIZE 4
#define MAP_DIM 4
#define BIG_NUMBER 1E10
#define _X_ 0
#define _Y_ 1
#define _Z_ 2
#define _U_ 3
#define _V_ 4
#define _W_ 5
#define _PHI_ 6
#define _THE_ 7
#define _PSI_ 2
#define _B_ 3
#define _DVLX_ 10
#define _DVLY_ 11
#define _DVLZ_ 12

typedef struct PoseLink_{
  // x1 = Transform*x2;
  int idx1;
  int idx2;
  double Transform[16];

  public:
  void loadLink(CSVRow);

} PoseLink;

class ResonMeasurement {
  public:
	double state[MAP_DIM];
	/* these are all in the vehicle frame */
	double mx();
	double my();
	double mz();
	double ni();
	int featureIndex;
        int poseIdx;
	ResonMeasurement(int,double,double,double,double);
};


class PoseNode{
	public:
	// Constructor
		PoseNode(CSVRow);
	// member values
		double time;
		int stateSize;
		double state[STATE_SIZE];
		double inputs[7]; // [omega_z, DVL, z, pitch, roll]
		bool DVLflag;
		double DVL[3];

		double xEst() const ;
		double yEst() const ;
		double zEst() const ;
		double uEst() const ;
		double vEst() const ;
		double wEst() const ;
		double phiEst() const ;
		double thetaEst() const ;
		double psiEst() const ;
		double biasEst() const ;



		std::vector<ResonMeasurement> measurements;
	// member functions
		
};

class Trajectory{
	public:
		Trajectory(void);
		Trajectory(std::vector<PoseNode> poses_in);
		std::vector<PoseNode> poses;
		bool PlotTrajectory(void);
                bool PlotTrajectory(std::vector<PoseLink>);
                void saveTrajectoryToFile(char * filename); 
                bool PlotProposedMatches(int, int, float);
                pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractSubcloud(int, int);
                pcl::PointCloud<pcl::PointXYZ>::Ptr ExtractSubcloudAtAlt(int,int);
                void addConstBias(double);
		void updateWithConstBias(double);
		double plotDuration;
		

}; 

class FeatureNode {
	public:
	// Constructor
		FeatureNode(PoseNode,ResonMeasurement);
	// member values
		double state[MAP_DIM];
		int featureIndex;
};

#endif
