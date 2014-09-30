#include "NodeDefinitions.h"
#define BIG_NUMBER 1E10

// NEED TO DEFINE NOISE MATRICES 

struct OdometryError {
  OdometryError(PoseNode pose1, PoseNode pose2)
	: pose1(pose1), pose2(pose2) {}

  template <typename T>
  bool operator()(const T* const poseEst1, const T* const poseEst2, T* residual) const { 
	// Using the information in pose1 & pose 2 encode odometry error fxn
	double dT = pose2.time - pose1.time;
	T R[3][3];
	T worldVel1[3];
	T worldVel2[3];
	T AngleAxis[3];
	T Xres;
	T Yres;
	AngleAxis[0] = T(0.);
	AngleAxis[1] = T(0.);
	AngleAxis[2] = poseEst1[_PSI_];
	T bodyVel[3];
	bodyVel[0] = T(pose1.uEst());
	bodyVel[1] = T(pose1.vEst());
	bodyVel[2] = T(pose1.wEst());
	ceres::AngleAxisRotatePoint(AngleAxis,bodyVel,worldVel1);
	AngleAxis[2] = poseEst2[_PSI_];
	ceres::AngleAxisRotatePoint(AngleAxis,bodyVel,worldVel2);
	Xres = poseEst2[_X_] - (poseEst1[_X_] + (.5*worldVel1[0] + .5*worldVel2[0])*T(dT));
	Yres = poseEst2[_Y_] - (poseEst1[_Y_] + (.5*worldVel1[1] + .5*worldVel2[1])*T(dT));
	if (pose1.DVLflag){ // check for dvl lock
	residual[_X_] = 1.*Xres ;
	residual[_Y_] = 1.*Yres ;
	} else {
	residual[_X_] = .1*Xres ;
	residual[_Y_] = .1*Yres ;
	}
	residual[_PSI_] = 100.*(poseEst2[_PSI_] - (poseEst1[_PSI_] + (T(pose1.inputs[0]) - poseEst1[_B_] )*T(dT)  ));
	residual[_B_] = 100000.*(poseEst2[_B_] - poseEst1[_B_]);

	return true;
  }
   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const PoseNode pose1,
                                      const PoseNode pose2) {
     return (new ceres::AutoDiffCostFunction<OdometryError, STATE_SIZE, STATE_SIZE, STATE_SIZE>(
                 new OdometryError(pose1, pose2)));
   }

   PoseNode pose1;
   PoseNode pose2;
};


struct MeasurementError {
  MeasurementError(PoseNode pose1, FeatureNode feature1, int measIdx1)
	: pose(pose1), feature(feature1), measIdx(measIdx1) {}

  template <typename T>
  bool operator()(const T* const poseEst, const T* const featureEst, T* residual) const { 
	// Using the information in pose & feature to calc error fxn
        /* Build zhat */
	// extract heading
	T heading = poseEst[_PSI_];
	T r_feat_pose[3];
	T zHat_pos[3];
	T AngleAxis[3];
	AngleAxis[0] = T(0.);
	AngleAxis[1] = T(0.);
	AngleAxis[2] = -poseEst[_PSI_];
	// difference in world frame
	r_feat_pose[0] = featureEst[0] - poseEst[_X_];
	r_feat_pose[1] = featureEst[1] - poseEst[_Y_];
	r_feat_pose[2] = featureEst[2] - poseEst[_Z_];
	// rotate into vehicle frame
	ceres::AngleAxisRotatePoint(AngleAxis,r_feat_pose,zHat_pos);
	// add normal inclination info
	T zHat[4];
	zHat[0] = zHat_pos[0];	
	zHat[1] = zHat_pos[1];	
	zHat[2] = zHat_pos[2];	
	zHat[3] = featureEst[3];	
	/* END creation of zHat */
	/* Build z = pose.measurements[measIdx].state */
	T zMeas[4];
	zMeas[0] = T(pose.measurements[measIdx].state[0]);
	zMeas[1] = T(pose.measurements[measIdx].state[1]);
	zMeas[2] = T(pose.measurements[measIdx].state[2]);
	zMeas[3] = T(pose.measurements[measIdx].state[3]);


	/* Residual */

	residual[0] = .1*(zHat[0] - zMeas[0]); 
	residual[1] = .1*(zHat[1] - zMeas[1]); 
	residual[2] = .01*(zHat[2] - zMeas[2]); 
	residual[3] = .001*(zHat[3] - zMeas[3]); 
	/*
	residual[0] = .1*(zHat[0] - zMeas[0]); 
	residual[1] = .1*(zHat[1] - zMeas[1]); 
	residual[2] = .0*(zHat[2] - zMeas[2]); 
	residual[3] = .01*(zHat[3] - zMeas[3]);
	*/

	return true;
  }
   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const PoseNode pose1,
                                      const FeatureNode feat1,
				      const int jj) {
     return (new ceres::AutoDiffCostFunction<MeasurementError, MAP_DIM,STATE_SIZE, MAP_DIM>(
                 new MeasurementError(pose1, feat1,jj)));
   }

   PoseNode pose;
   FeatureNode feature;
   int measIdx;
};
// DVL!
struct DVLError {
  DVLError(PoseNode pose1)
	: pose(pose1) {}

  template <typename T>
  bool operator()(const T* const poseEst, T* residual) const { 

	/* Residual */
	if (pose.DVLflag){
	residual[0] = .1*(T(pose.uEst()) - poseEst[_U_]); // Assumes perfect DVL
	residual[1] = .1*(T(pose.vEst()) - poseEst[_V_]);
	residual[2] = .1*(T(pose.wEst()) - poseEst[_W_]);
	} else {
	residual[0] = T(0.); // Assumes perfect DVL
	residual[1] = T(0.);
	residual[2] = T(0.);
	}

	return true;
  }
   // Factory to hide the construction of the CostFunction object from
   // the client code.
   static ceres::CostFunction* Create(const PoseNode pose1) {
     return (new ceres::AutoDiffCostFunction<DVLError, 3,STATE_SIZE>(
                 new DVLError(pose1)));
   }

   PoseNode pose;
};
