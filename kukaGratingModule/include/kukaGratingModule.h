/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Lucia Pais
 * email:   lucia.pais@epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef kukaGratingModule_H_
#define kukaGratingModule_H_

#include "RobotLib/RobotInterface.h"
#include "RobotLib/WorldObject.h"
#include "RobotLib/KinematicChain.h"
#include "MathLib/IKGroupSolver.h"
#include "KUKARobotModel/LWRRobot.h"
#include "MotionGenerators/CDDynamics.h"
#include "GLTools/GLTools.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <CDSExecution.h>
#include "GMR.h"

// Execution Modes for the Real Time loop
#define MODE_NONE  0
#define MODE_HOME  1
#define MODE_REACH 2
#define MODE_PUSH  3
#define MODE_GRATE 4

// States in the FSM
#define TASK_START              -1
#define TASK_REACH_GRATER_TOP    0
#define TASK_REACH_GRATER_BOTTOM 1
#define TASK_REACH_TRASH         2
#define TASK_SEARCH_GRATER       3
#define	TASK_NONE		 4

// Segment IDs
#define	SEGM_REACH		1
#define SEGM_GRATE		2
#define SEGM_TRASH		3

// Dynamic types for the CDS
#define MODEL_DYNAMICS    1
#define LINEAR_DYNAMICS   2
#define NO_DYNAMICS	  3

// Translational Stiffness values
#define TRANS_STIFF_X	1200
#define TRANS_STIFF_Y	1200
#define TRANS_STIFF_Z	1200

// Rotational Stiffness values
#define ROT_STIFF_X	300
#define ROT_STIFF_Y	300
#define ROT_STIFF_Z	300

// Vision Stuff
// #define USE_VISION
#define USE_SYMMETRY
#define MAX_TRACKED_OBJS 10


class kukaGratingModule : public RobotInterface
{
protected:
    LWRRobot* mLWRRobot;
    Robot::ControlMode ctrlmode;

    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;

    int             mState;		// index variable for the RT loop
    int		    mCurrentTask;	// index variable for the FSM
    int		    mNextTask;		// index variable for the FSM

// ===================================================
// ========  Control Variables	      ================
// ===================================================

    CDSExecution	*cdsRun;		// CDS Controller for Cartesian Space
    GMR 		*mForceModel;		// Cartesian force basedon current position
    GMR 		*mStiffnessModel;	// Cartesian Stiffness based on current position


    Vector3		vDesiredStiffness;	// Translational stiffness
    Vector3		vStiffnessModulation;	// Stiffness Modulation (lambda factor)

    CDDynamics 		*genCart; 		// moving in cartesian space
    CDDynamics 		*genJoint;		// moving in joint space
    CDDynamics 		*mCDJointFilter; 	// smoother for joint space commands
    CDDynamics		*mCartFilter;		// smoother in Cart space

    Matrix4 		mHTARGETinABS;
    Vector3 		currentTarget;

    void		initializeController();
    double 		module_dt;
    double 		fri_dt;

    void 		initializeCDSController(int segmentID, int dynamicsType);
    void 		initializeForce(int segmentID);
    void 		initializeStiffness(int segmentID);

    Matrix4		mNextRobotEEPose;
    bool		bCDSinitialized;
    bool		bForceInitialized;
    bool		bStiffnessInitialized;
    bool		bForceBypass;

// ===================================================
// ========  Inverse Kinematics Stuff ================
// ===================================================

   	KinematicChain  mKinChain;
   	IKGroupSolver   mIKSolver;
   	IndicesVector   mJointMapping;
   	Vector		mJointVelLimits[2];
   	Vector          vCurrJoint;

	void		initializeRobot();	// initializes Robot and Control Mode
	void		solveIK(Matrix4& mNextEEPose);  // For simulation mode or for JPOS mode

// ===================================================
// ========  Target Object Variables  ================
// ===================================================

	WorldObject*    oTargetObj;             	// the object we're trying to reach
	Matrix4		mTargetFrame;			// the target reference frame
	Matrix4         mTargetRelativeFrame;   	// the frame relative to the target
	Vector 		vTarget;			// the target as it should be set for CDDynamics (x, y, z, wx, wy, wz)
	void		initializeTaskVariables();

// ===================================================
// ========  Robot Position Variables ================
// ===================================================
    	int nEndEffectorId;

	Vector3 mRobotEEAbsolutePos;			// Robot's position in absolute coordinates
	Matrix3 mRobotEEAbsoluteOrient;			// Robot's orientation in absolute coordinates
	Matrix4 mRobotEEAbsoluteFrame;
	Vector3 vRobotEEPosInTargetFrame;  		// Robot's position in the target frame
	Vector3 vRobotEEOrientInTargetFrame;		// Robot's orientation in the target frame
	Matrix4	mFullEEInTargetFrame;			// Robot's full EE position and orientation in target frame

	//Vector3 vPosInTargetFrame
	Vector3	vPositionError3;			// Position error
	Vector3 vPosErrMult;				// Multiplying factors for position error
	Vector3	vOrientationError3;			// Orientation error
	Vector3 vOrientErrMult;				// Multiplying factors for orientation error
	REALTYPE reachingThreshold;			// threshold for reaching the target

	void 		getCrtEEPose(Vector3& position, Matrix3& orientation);		// Returns current robot Cartesian coordinates
	Matrix4		getCrtEEFullFrame();

	bool    	bSync;				// True if the current robot joint state should be sinked in CDDynamics
	bool		bSyncCart;			// True if the current robot cartesian state should be sinked in CDDynamics

	void 		synchronizeJointMotion();
	void		synchronizeCartMotion();

	bool		bTargetReached;					// True if target was reached
	bool		checkTargetReached(Matrix4& mGoalFrame); 	// checks if target was reached
	bool		bTargetSet;
	double		computeReachingError(Matrix4& mNextEEPose);

// ===================================================
// ========  Task Variables           ================
// ===================================================

	Vector zzzStiff;

	int  nGratingPasses;
	int  nGratingTotalPasses;

	// The following methods update both
	// the target object and the attractor
	void setTargetToGrater();
	void setTargetToGraterTop();
	void setTargetToGraterBottom();
	void setTargetToBowl();
	void setTargetToSearchGrater();

	void advanceStateMachine();
	void initializeVariables();

	double shift;
	double des_force;
	double curr_force;

// ===================================================
// ==============    Various Utilities  ==============
// ===================================================

	// Saving to File
	vector<float>		nTaskId;
	vector<Vector> 		crt_cart_pos;
	vector<Vector> 		crt_cart_or;
	vector<Vector> 		crt_cart_for;
	vector<Vector>		crt_jnt_tqs;

	bool			bRecordingStarted;
	void    		saveCurrentRobotState();
	void			saveToFile(int fid);

	// Various Things
	void			checkCtrlMode();
	Vector			getHomePositionforSim();	// If running in simulator we start the execution from a home position

	// Drawing trajectory
	double 			timeIDX;
	Matrix			mRobTraj;
								// If running on the real robot the home position is set on the script
// ===================================================
// ==============    VISION Stuff  ===================
// ===================================================

	#ifdef USE_VISION
		ros::NodeHandle 	*node;
		tf::TransformListener *listener;
		btMatrix3x3            btmat2[MAX_TRACKED_OBJS];
		btVector3              btvec3[MAX_TRACKED_OBJS];
		const char*            object_names[MAX_TRACKED_OBJS];     // array containing object names in the vision system
		WorldObject*           object_handles[MAX_TRACKED_OBJS];   // array containing objects from the scene that are tracked
		int                    object_count;       		   // size of the two arrays above
		Matrix4                visionHMatrix[MAX_TRACKED_OBJS];
		tf::StampedTransform   transform[MAX_TRACKED_OBJS];
		bool                   tracking_vision;
	#endif
	void	initializeVision();
	void	updateVision();
// ===================================================


public:
            kukaGratingModule();
    virtual ~kukaGratingModule();

    virtual Status              RobotInit();
    virtual Status              RobotFree();

    virtual Status              RobotStart();
    virtual Status              RobotStop();

    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual void 		RobotDraw();
    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
};



#endif
