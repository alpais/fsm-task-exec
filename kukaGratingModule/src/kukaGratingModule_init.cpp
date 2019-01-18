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

#include "kukaGratingModule.h"

void kukaGratingModule::initializeRobot(){

	// ===================================================
	// ========  Initialize Robot and CTRL mode ==========
	// ===================================================
	if (mRobot->IsSimulationMode())
		mLWRRobot = new LWRRobot;
	else
		mLWRRobot = (LWRRobot*) mRobot;

	ctrlmode = Robot::CTRLMODE_POSITION;
	mSensorsGroup.SetSensorsList(mRobot->GetSensors());
	mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

	nEndEffectorId = mRobot->GetLinkIndex("TOOL");
	cout << "The end effector id is " << nEndEffectorId << endl;
	if (nEndEffectorId == -1)
		GetConsole()->Print("ERROR: End effector not found");

	// ===================================================
	// ========  Initialize Inverse Kinematics ===========
	// ===================================================
	mKinChain.SetRobot(mRobot);
	mKinChain.Create(0, 0, nEndEffectorId);
	mJointMapping = mKinChain.GetJointMapping();
	mIKSolver.SetSizes(mKinChain.GetDOFCount());
	mIKSolver.AddSolverItem(6); 					// One solver with 6 constraints (x,y,z, wx, wy, wz)
	mIKSolver.SetVerbose(false); 					// No comments
	mIKSolver.SetThresholds(0.0001, 0.0001); 		// Singularities thresholds
	mIKSolver.Enable(true, 0); 						// Enable first solver
	mIKSolver.SetDofsIndices(mJointMapping, 0); 	// Joint maps for first solver
	mJointVelLimits[0].Resize(mKinChain.GetDOFCount());
	mJointVelLimits[1].Resize(mKinChain.GetDOFCount());
	mJointVelLimits[0] = DEG2RAD(-60.0);
	mJointVelLimits[1] = DEG2RAD( 60.0);
	mIKSolver.SetLimits(mJointVelLimits[0], mJointVelLimits[1]);
	vCurrJoint.Resize(mRobot-> GetDOFCount());
}


void kukaGratingModule::initializeTaskVariables(){

	// ===================================================
	// ========  Initialize Stuff for grating  ===========
	// ===================================================
	shift 		= -0.03;		// On grater

	des_force 	= 0.0;	// Vertical force while grating
	curr_force 	= 0.0;	// Temporary variable for calculating force correction

	module_dt 	= 0.002;
	fri_dt 		= 0.002;

	reachingThreshold = 0.02;
	nGratingTotalPasses = 5;

	mTargetRelativeFrame = Matrix4::IDENTITY;

	mRobotEEAbsolutePos.Zero();
	mRobotEEAbsoluteOrient = Matrix3::IDENTITY;

	vRobotEEPosInTargetFrame.Zero();
	vRobotEEOrientInTargetFrame.Zero();
	mFullEEInTargetFrame = Matrix4::IDENTITY;

	mNextRobotEEPose.Identity();
	vDesiredStiffness.Set(Vector3(TRANS_STIFF_X, TRANS_STIFF_Y, TRANS_STIFF_Z));
	vStiffnessModulation.Set(Vector3(1,1,1));

	vPositionError3.Zero();
	vOrientationError3.Zero();

	vTarget.Resize(6);
	vTarget.Zero();

	bSync 		= false;
	bSyncCart 	= false;
	bTargetReached 	= false;
	bTargetSet 	= false;

	bCDSinitialized 	= false;
	bForceInitialized 	= false;
	bStiffnessInitialized 	= false;

	bForceBypass 	= false;

	// Variables used for saving in the file
	bRecordingStarted = false;
	crt_cart_pos.clear();
	crt_cart_or.clear();
	crt_cart_for.clear();
	crt_jnt_tqs.clear();
	nTaskId.clear();

	// Variables used for drawing the trajectory in the simulator
	timeIDX = 0;
	mRobTraj.Resize(12000, 3);
	Vector3 tpos; Matrix3 tmat; getCrtEEPose(tpos, tmat);
	mRobTraj(0,0) = tpos(0); mRobTraj(0,1) = tpos(1);  mRobTraj(0,2) = tpos(2);


}

void kukaGratingModule::initializeController(){

	// ===================================================
	// ========  Initialize CDDynamics         ===========
	// ===================================================

	// Cartesian space
	genCart = new CDDynamics(6, module_dt, 1); // In cartesian space x, y, z, wx, wy, wz
	Vector vel_lim_cart(6);
	vel_lim_cart = DEG2RAD(60);
	genCart->SetVelocityLimits(vel_lim_cart);

	mCartFilter = new CDDynamics(6, module_dt, 1);
	mCartFilter->SetVelocityLimits(vel_lim_cart);
	mCartFilter->SetWn(1);

	// Joint Space
	genJoint = new CDDynamics(mRobot->GetDOFCount(), module_dt, 1); // In joint space
	Vector vel_lim_joint(mRobot->GetDOFCount());
	vel_lim_joint = DEG2RAD(60);
	genJoint->SetVelocityLimits(vel_lim_joint);

	mCDJointFilter = new CDDynamics(mRobot->GetDOFCount(), fri_dt, 1); // filter in jointspace
	Vector vel_lim_joints(mRobot->GetDOFCount());
	vel_lim_joints = DEG2RAD(60);
	mCDJointFilter->SetVelocityLimits(vel_lim_joints);

}


void kukaGratingModule::initializeVision(){

	// ===================================================
	// ========  Initialize Vision             ===========
	// ===================================================

#ifdef USE_VISION
	int argc = 0;
	char** argv;
	ros::init(argc, argv, "kukaGratingTask", ros::init_options::NoSigintHandler);
	node = new ros::NodeHandle;
	// initialize the tracking system
	object_count = 2;
	object_names[0] = "/grater_root";
	object_names[1] = "/bowl_root";
	object_handles[0] = GetWorld()->Find("grater");
	object_handles[1] = GetWorld()->Find("bowl");
	listener = new tf::TransformListener;
	for (int i=0; i<object_count; i++) {
		visionHMatrix[i].Set(object_handles[i]->GetReferenceFrame().GetHMatrix());
	}
	tracking_vision = true;
#endif


}
