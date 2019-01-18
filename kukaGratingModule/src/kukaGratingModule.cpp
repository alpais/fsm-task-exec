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


kukaGratingModule::kukaGratingModule()
:RobotInterface(){
}
kukaGratingModule::~kukaGratingModule(){
}

RobotInterface::Status kukaGratingModule::RobotInit(){

	// ===================================================
	// ========  Add Console Commands ====================
	// ===================================================
	AddConsoleCommand("home");
	AddConsoleCommand("ctrl_Cart");
	AddConsoleCommand("force");

	AddConsoleCommand("go");
	AddConsoleCommand("go_rec");

	AddConsoleCommand("save");
	AddConsoleCommand("readPose");

	initializeRobot();
	initializeTaskVariables();
	initializeVision();
	initializeController();

    return STATUS_OK;
}
RobotInterface::Status kukaGratingModule::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status kukaGratingModule::RobotStart(){
	cout << "Robot EE POSE " << endl;

	Vector3 position; Matrix3 orientation;
	getCrtEEPose(position, orientation);
	position.Print();

    return STATUS_OK;
}    
RobotInterface::Status kukaGratingModule::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status kukaGratingModule::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status kukaGratingModule::RobotUpdateCore(){

	if (mRobot->GetControlMode() != ctrlmode) {
		mRobot->SetControlMode(ctrlmode);
		bSync = true;
	}

#ifdef USE_VISION
	updateVision();
#endif
	if (bSync == true) {
		synchronizeJointMotion(); timeIDX = 0;
		return STATUS_OK;
	}

	if (bSyncCart == true){
		synchronizeCartMotion(); timeIDX = 0;
		return STATUS_OK;
	}

	// ===================================================
	// ========  Reaction loop                 ===========
	// ===================================================
	switch (mState) {
	case MODE_NONE:
		break;
	case MODE_HOME:
	{
		Vector  vJointCommand;
		vJointCommand.Resize(mRobot->GetDOFCount());
		vJointCommand = getHomePositionforSim();

		genJoint->SetTarget(vJointCommand);
		genJoint->Update();
		genJoint->GetState(vJointCommand);

		mActuatorsGroup.SetJointAngles(vJointCommand);
		mActuatorsGroup.WriteActuators();

		break;
	}
	case MODE_REACH:
	{
// ------------------------------------------------------------------------------------------------------------------
		// Get object position and Compute attractor frame in abs. This represents the "Target"
		Matrix4 mHABSOBJinABS = oTargetObj->GetReferenceFrame().GetHMatrix();
		mHTARGETinABS.Identity();
		mHTARGETinABS.SetTranslation(mHABSOBJinABS.GetOrientation()*mTargetRelativeFrame.GetTranslation()
				+ mHABSOBJinABS.GetTranslation());

		mHTARGETinABS.SetOrientation(mHABSOBJinABS.GetOrientation()*mTargetRelativeFrame.GetOrientation());
		bTargetSet = true;

// ------------------------------------------------------------------------------------------------------------------
		// Querry the model for the next desired position
		cdsRun->setObjectFrame(mHABSOBJinABS);			// Object moves in real time
		cdsRun->setAttractorFrame(mTargetRelativeFrame);	// Attractor moves with it
		cdsRun->setCurrentEEPose(getCrtEEFullFrame());
		mNextRobotEEPose = cdsRun->getNextEEPose();
		// smooth out the trajectory
		vTarget.SetSubVector(0, mNextRobotEEPose.GetTranslation());
		vTarget.SetSubVector(3, mNextRobotEEPose.GetOrientation().GetExactRotationAxis());
		mCartFilter->SetTarget(vTarget);
		mCartFilter->Update();
		mCartFilter->GetState(vTarget);

		// Querry for the desired Stiffness
		// Compute crt position wrt the attractor 
		computeReachingError(mHTARGETinABS);
		mStiffnessModel->getGMROutput(vPositionError3.Array(), vStiffnessModulation.Array());
		vDesiredStiffness(0) = TRANS_STIFF_X * vStiffnessModulation(0);
		vDesiredStiffness(1) = TRANS_STIFF_Y * vStiffnessModulation(1);
		vDesiredStiffness(2) = TRANS_STIFF_Z * vStiffnessModulation(2);
/*		
		vTarget.SetSubVector(0, mHTARGETinABS.GetTranslation());
		vTarget.SetSubVector(3, mHTARGETinABS.GetOrientation().GetExactRotationAxis());
		genCart->SetTarget(vTarget);
		genCart->Update();
		genCart->GetState(vTarget);
*/
		mRobotEEAbsolutePos.Set(Vector3(vTarget[0], vTarget[1], vTarget[2]));
		mRobotEEAbsoluteOrient = Matrix3::SRotationV(Vector3(vTarget[3], vTarget[4], vTarget[5]));

// ------------------------------------------------------------------------------------------------------------------
		// for drawing the trajectory in the simulator
		mRobTraj(timeIDX, 0) = mRobotEEAbsolutePos(0);
		mRobTraj(timeIDX, 1) = mRobotEEAbsolutePos(1);
		mRobTraj(timeIDX, 2) = mRobotEEAbsolutePos(2);
		timeIDX++;


// ------------------------------------------------------------------------------------------------------------------
		// Going down until there is a contact with the grater
		if ((mCurrentTask == TASK_SEARCH_GRATER) && (ctrlmode == Robot::CTRLMODE_CARTIMPEDANCE))
		{
			Vector cartStiffness;
			cartStiffness.Resize(6);
			cartStiffness.SetSubVector(0, Vector3(TRANS_STIFF_X, TRANS_STIFF_Y, TRANS_STIFF_Z));
			cartStiffness.SetSubVector(3, Vector3(ROT_STIFF_X, ROT_STIFF_Y, ROT_STIFF_Z));
			mLWRRobot->SetCartStiffness(cartStiffness);

			Vector vCrtEEForce; vCrtEEForce.Resize(6); vCrtEEForce.Zero();
			vCrtEEForce = mLWRRobot->GetEstimatedExternalCartForces();

			Matrix3 tmporient; Vector3 tmppos;
			mLWRRobot->GetMeasuredCartPose(tmppos, tmporient);
			Vector3 tmpCT = mHABSOBJinABS.GetOrientation().Transpose().Mult(tmppos) - mHABSOBJinABS.GetTranslation();

			if(vCrtEEForce(2) > 2.0 )
			{
				currentTarget = tmpCT;
				currentTarget.Print();
				cout << "    >>> Grater Found!" << endl;
				mCurrentTask = mNextTask;
  				advanceStateMachine();
			}
		}


// ------------------------------------------------------------------------------------------------------------------
		// Always maintain the proper force for grating
		if ((mCurrentTask == TASK_REACH_GRATER_BOTTOM) && (ctrlmode == Robot::CTRLMODE_CARTIMPEDANCE))
		{
			// Read current EE Forces
			Vector vCrtEEForce; vCrtEEForce.Resize(6); vCrtEEForce.Zero();
			vCrtEEForce = mLWRRobot->GetEstimatedExternalCartForces();
			// Querry the model for the desired vertical force based on the position along the grater (i.e. the x axis)
			double nDesiredForce = 0;
			double x_disp = vPositionError3(0);
			mForceModel->getGMROutput(&x_disp, &nDesiredForce);

			double f_err;
			if (bForceBypass)
				f_err = des_force - vCrtEEForce(2);
			else
				f_err = nDesiredForce - vCrtEEForce(2);

			vCrtEEForce.Zero();
			vCrtEEForce(2) = f_err * 0.05;
			mLWRRobot->SetCartForce(vCrtEEForce);
		}

// ------------------------------------------------------------------------------------------------------------------
		if (checkTargetReached(mHTARGETinABS)){
			mCurrentTask = mNextTask;
			advanceStateMachine();
		}
		else{
			if(mRobot->IsSimulationMode() || (ctrlmode == Robot::CTRLMODE_POSITION))
			{
				// Sends command in joint position mode, but ignores the computed forces and stiffness - mainly useful for simulation mode
				Matrix4 mRobotEEAbsoluteFrame; mRobotEEAbsoluteFrame.Identity();
				mRobotEEAbsoluteFrame.SetTranslation(mRobotEEAbsolutePos);
				mRobotEEAbsoluteFrame.SetOrientation(mRobotEEAbsoluteOrient);
				solveIK(mRobotEEAbsoluteFrame);
			}
			else{
				// Send commands in Cartesian impedance mode
				mLWRRobot->SetCartCommand(mRobotEEAbsolutePos, mRobotEEAbsoluteOrient);
				Vector cartStiffness;
				cartStiffness.SetSubVector(0, vDesiredStiffness);
				cartStiffness.SetSubVector(3, Vector3(ROT_STIFF_X, ROT_STIFF_Y, ROT_STIFF_Z));			
				mLWRRobot->SetCartStiffness(cartStiffness);	
			}
		}

		break;
	}

	default:
		break;
	}


// ------------------------------------------------------------------------------------------------------------------
	if (bRecordingStarted){
		saveCurrentRobotState();
	}


    return STATUS_OK;
}
int kukaGratingModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){

	if (cmd == "home") {
		mState = MODE_HOME;
		bSync = true;
	} else if (cmd == "go") {
		mCurrentTask = TASK_START;
		advanceStateMachine();
		mState = MODE_REACH;
		bSync = true;
	} else if (cmd == "readPose") {
		if(args.size() >= 1)
		{
			string entity = args[0].c_str();	
			if (entity == "bowl"){
				cout << " >>> Bowl position wrt the arm" << endl;
				WorldObject*	oTmpObj;				
				oTmpObj = GetWorld()->Find("bowl");
				oTmpObj->GetReferenceFrame().GetHMatrix().GetTranslation().Print();
				oTmpObj->GetReferenceFrame().GetHMatrix().GetOrientation().Print();
			} else if (entity == "grater") {
				cout << " >>> Grater position wrt the arm" << endl;
				WorldObject*	oTmpObj;				
				oTmpObj = GetWorld()->Find("grater");
				oTmpObj->GetReferenceFrame().GetHMatrix().GetTranslation().Print();
				oTmpObj->GetReferenceFrame().GetHMatrix().GetOrientation().Print();
			} else if (entity == "robotee") {
				cout << endl << "Robot EE POSE " << endl;
				getCrtEEFullFrame().Print();
			} else if (entity == "att"){
				cout << endl << "Attractor Pose in Object" << endl;
				mTargetRelativeFrame.GetTranslation().Print();
				mTargetRelativeFrame.GetOrientation().Print();
			} else if (entity == "robotja"){
				cout << endl << "Current Robot Joint Angles" << endl;
				mSensorsGroup.ReadSensors();
				mSensorsGroup.GetJointAngles().Print();
			}
		}

	} else if (cmd == "go_rec") {
		mCurrentTask = TASK_START;
		advanceStateMachine();
		mState = MODE_REACH;
		bSync = true;
		bRecordingStarted = true;
	} else if (cmd == "save"){
		int fid = 1;
		if (args.size() > 0){
		    fid = atoi(args[0].c_str());
		}
		if (bRecordingStarted){
		    saveToFile(fid);
		}
	} else if (cmd == "ctrl_Cart") {
		 ctrlmode = Robot::CTRLMODE_CARTIMPEDANCE;
		 if(mRobot->GetControlMode() != ctrlmode)
		 {
		 	mRobot->SetControlMode(ctrlmode);
			bSync = true;
		 	// bSyncCart = true;
		 }
		// Synchronize Cartesian Position
		Vector3 cartPos; cartPos.Zero();
		Matrix3 cartRot; cartRot.Identity();
		mLWRRobot->GetMeasuredCartPose(cartPos, cartRot);
		mLWRRobot->SetCartCommand(cartPos,cartRot);

		// Synchronize Cartesian Stiffness
		Vector cartStiffness;
		cartStiffness.Resize(6);
		cartStiffness.SetSubVector(0, Vector3(1200, 1200, 1200));
		cartStiffness.SetSubVector(3, Vector3(700, 700, 700));
		mLWRRobot->SetCartStiffness(cartStiffness);

		Vector cartDamping;
		cartDamping.Resize(6);
		mLWRRobot->GetCartDamping().Print("Cart Damping");

		// Synchronize Cartesian Forces
		Vector cartForces;
		cartForces.Resize(6);
		cartForces = mLWRRobot->GetEstimatedExternalCartForces();
//		mLWRRobot->SetEstimatedExternalCartForces(cartForces);

	 } else if(cmd == "force") {	// Bypas model force by setting a fixed value
		 if(args.size() >= 1)
		 {
			 double tmp = atof(args[0].c_str());
			 if(tmp < 0 ) tmp = 0;
			 if(tmp >20) tmp = 20;
			 des_force = tmp;
			 bForceBypass = true;
		 }
	 }

    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    kukaGratingModule* create(){return new kukaGratingModule();}
    void destroy(kukaGratingModule* module){delete module;}
}

