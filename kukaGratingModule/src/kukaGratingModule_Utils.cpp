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

void kukaGratingModule::checkCtrlMode(){

	if (mRobot->GetControlMode() != ctrlmode) {
		mRobot->SetControlMode(ctrlmode);
		bSync = true;
	}

}

void kukaGratingModule::synchronizeJointMotion(){

	mSensorsGroup.ReadSensors();
	vCurrJoint = mSensorsGroup.GetJointAngles();
	mActuatorsGroup.SetJointAngles(vCurrJoint);
	mActuatorsGroup.WriteActuators();
	mSensorsGroup.SetJointAngles(vCurrJoint);
	mSensorsGroup.WriteSensors();

	// Set state for the CDDynamics joint filter
	mCDJointFilter->SetState(vCurrJoint);

	// Set State for the CDDynamics in Joint Space
	Vector robotDOF;
	robotDOF.Resize(mRobot->GetDOFCount());

	for (int i = 0; i < sizeof(robotDOF); i++)
		robotDOF[i] = vCurrJoint[mJointMapping[i]];
	genJoint->SetState(robotDOF);

	// Set State for CDDynamics in Cartesian Space
	// we actually do this when we assign a new target, no need to do it here then

	bSync = false;
}

void kukaGratingModule::synchronizeCartMotion(){

	// Synchronize Position
	Vector    x_state;
	x_state.Resize(6, false);
	Vector3 vEECrtPos; Matrix3 mEECrtRot; mEECrtRot.Identity();
	getCrtEEPose(vEECrtPos, mEECrtRot);

	x_state.InsertSubVector(0, vEECrtPos, 0, 3);
	x_state.InsertSubVector(3, mEECrtRot.GetExactRotationAxis(), 0, 3);

	genCart->SetState(x_state);
	mCartFilter->SetState(x_state);

	// Synchronize Stiffness
	Vector cartStiffness;
	cartStiffness.Resize(6);
	cartStiffness.SetSubVector(0, Vector3(1200, 1200, 1200));
	cartStiffness.SetSubVector(3, Vector3(700, 700, 700));
	mLWRRobot->SetCartStiffness(cartStiffness);

	// Synchronize Force
	Vector vCartForce; vCartForce.Resize(6); vCartForce.Zero();
	vCartForce = mLWRRobot->GetEstimatedExternalCartForces();
	mLWRRobot->SetEstimatedExternalCartForces(vCartForce);

	bSyncCart = false;
}

void kukaGratingModule::getCrtEEPose(Vector3& position, Matrix3& orientation){

	if (mRobot->IsSimulationMode() || (ctrlmode == Robot::CTRLMODE_POSITION)) {
		position    = mRobot->GetReferenceFrame(nEndEffectorId).GetOrigin();
		orientation = mRobot->GetReferenceFrame(nEndEffectorId).GetHMatrix().GetOrientation();
	} else {
		mLWRRobot->GetMeasuredCartPose(position, orientation);
	}
}

Matrix4 kukaGratingModule::getCrtEEFullFrame(){

	Vector3 tmppos; 
	Matrix3 tmporient; 
	getCrtEEPose(tmppos, tmporient); 
	
	Matrix4 tmpEEFrame; tmpEEFrame.Identity();
	tmpEEFrame.SetTranslation(tmppos); 
	tmpEEFrame.SetOrientation(tmporient);

	return(tmpEEFrame);
}

double kukaGratingModule::computeReachingError(Matrix4& mNextEEPose){

// Updates the global variables vPositionError3 and vOrientationError3

	if (mRobot->IsSimulationMode() || (ctrlmode == Robot::CTRLMODE_POSITION)) {
		vPositionError3 = mNextEEPose.GetTranslation() - mRobot->GetReferenceFrame(nEndEffectorId).GetOrigin();
		vOrientationError3 = ((Vector3) (mRobot->GetReferenceFrame(nEndEffectorId).GetOrient().GetColumn(2))).Cross(mNextEEPose.GetOrientation().GetColumn(2))
			    		 + ((Vector3) (mRobot->GetReferenceFrame(nEndEffectorId).GetOrient().GetColumn(1))).Cross(mNextEEPose.GetOrientation().GetColumn(1));
	} else {
		Matrix3 tmporient;Vector3 tmppos;
		mLWRRobot->GetMeasuredCartPose(tmppos, tmporient);
		vPositionError3 = mNextEEPose.GetTranslation() - tmppos;
		vOrientationError3 = ((Vector3)(tmporient.GetColumn(2))).Cross(mNextEEPose.GetOrientation().GetColumn(2)) +
				((Vector3) (tmporient.GetColumn(1))).Cross(mNextEEPose.GetOrientation().GetColumn(1));
	}

	double err = 0.0;

	if(mCurrentTask == TASK_REACH_GRATER_BOTTOM )
		err = sqrt(vPositionError3(0)*vPositionError3(0) + vPositionError3(1)*vPositionError3(1));
	else
		err = ( vPositionError3.Norm() + vOrientationError3.Norm() ) / 2.0;

	return err;

}

void kukaGratingModule::solveIK(Matrix4& mNextEEPose){

	Vector mCartesianTarget;     mCartesianTarget.Resize(6);     mCartesianTarget.Zero();

	// Multiplication factors for position and orientation errors -> These are global variables specified for each target
	// Vector3 vPosErrMult; 		vPosErrMult.Set(2,2,2);
	// Vector3 vOrientErrMult; 	vOrientErrMult.Set(2,2,2);

	for (int i=0; i<3; i++){
		mCartesianTarget(i) = vPositionError3(i) * vPosErrMult(i);
		mCartesianTarget(i+3) = vOrientationError3(i) * vOrientErrMult(i);
	}

	mSensorsGroup.ReadSensors();
	vCurrJoint = mSensorsGroup.GetJointAngles();

	mKinChain.Update();
	mIKSolver.SetJacobian(mKinChain.GetJacobian(), 0);
	mIKSolver.SetTarget(SharedVector(mCartesianTarget), 0);
	mIKSolver.Solve();
	Vector ikout;
	ikout.Resize(mKinChain.GetDOFCount());
	ikout = mIKSolver.GetOutput();

	Vector vJointTarget;
	vJointTarget.Resize(mRobot->GetDOFCount());
	for (int i = 0; i < ikout.Size(); i++) {
		vJointTarget(mJointMapping[i]) = vCurrJoint(mJointMapping[i]) + ikout(i);
	}

	Vector vJointCommand;
	vJointCommand.Resize(mRobot->GetDOFCount());

	mCDJointFilter->SetTarget(vJointTarget);
	mCDJointFilter->Update(module_dt);
	mCDJointFilter->GetState(vJointCommand);

	mActuatorsGroup.SetJointAngles(vJointCommand);
	mActuatorsGroup.WriteActuators();
}

bool kukaGratingModule::checkTargetReached(Matrix4& mGoalFrame){

	double err = 0;
	err = computeReachingError(mGoalFrame);

	if (err < reachingThreshold) {
		if (!bTargetReached) {
			cout << "Target Reached" << endl;
			bTargetReached = true;
		}
	}
	return bTargetReached;
}

void kukaGratingModule::updateVision(){
// ===================================================
// ========  Vision Tracking               ===========
// ===================================================
#ifdef USE_VISION
	for(int i=0; i< object_count; i++)
	{
		try
		{
			listener->lookupTransform("/robot", object_names[i], ros::Time(0), transform[i]);
			btmat2[i].setRotation(transform[i].getRotation());
			btvec3[i] = transform[i].getOrigin();

			visionHMatrix[i].Set(btmat2[i][0][0], btmat2[i][0][1], btmat2[i][0][2], btvec3[i][0],
					btmat2[i][1][0], btmat2[i][1][1], btmat2[i][1][2], btvec3[i][1],
					btmat2[i][2][0], btmat2[i][2][1], btmat2[i][2][2], btvec3[i][2],
					0, 0, 0, 1);

			object_handles[i]->GetReferenceFrame().SetOrient(visionHMatrix[i].GetOrientation());
			object_handles[i]->GetReferenceFrame().SetOrigin(visionHMatrix[i].GetTranslation());
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR_THROTTLE(1, "%s", ex.what());
		}

		if(tracking_vision) {
			object_handles[i]->GetReferenceFrame().SetOrient(visionHMatrix[i].GetOrientation());
			object_handles[i]->GetReferenceFrame().SetOrigin(visionHMatrix[i].GetTranslation());
		}
	}
	// Little hack to flip the axes of the object 

//	oTargetObj = GetWorld()->Find("grater");
//	oTargetObj-> GetReferenceFrame().GetHMatrix().Mult(mTargetRelativeFrame, tmpm);

	Matrix3 mNewObjOrient;
	mNewObjOrient.Set(object_handles[0]->GetReferenceFrame().GetOrient());
	mNewObjOrient(0,0) = -mNewObjOrient(0,0);
	mNewObjOrient(1,0) = -mNewObjOrient(1,0);
	mNewObjOrient(2,0) = -mNewObjOrient(2,0);

	mNewObjOrient(0,1) = -mNewObjOrient(0,1);
	mNewObjOrient(1,1) = -mNewObjOrient(1,1);
	mNewObjOrient(2,1) = -mNewObjOrient(2,1);

	object_handles[0]->GetReferenceFrame().SetOrient(mNewObjOrient);
#endif

}

void kukaGratingModule::RobotDraw()
{

	Matrix4 mat;
	mat.Identity();
	mat.SetOrientation(mRobotEEAbsoluteOrient);
	mat.SetTranslation(mRobotEEAbsolutePos);

	GLTools::DrawRef(0.1, &mat);
//
	GLTools::DrawRef(0.05, &mHTARGETinABS);
	
/*	Matrix4 mTmpObj; mTmpObj.Identity();
	mTmpObj = oTargetObj-> GetReferenceFrame().GetHMatrix();
	GLTools::DrawRef(0.08, &mTmpObj);
*/

	if (bTargetReached){
		GLTools::DrawLines(mRobTraj);
	}

}

