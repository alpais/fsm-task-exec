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

void kukaGratingModule::setTargetToGrater() {

	oTargetObj = GetWorld()->Find("grater");
	if (!oTargetObj) GetConsole()->Print("ERROR: Target object not found");

	Matrix3         mTargetRelativeOrient;  // the target orientation relative to the object we're trying to reach
	Vector3         vTargetRelativePos;     // the target position relative to the object we're trying to reach

	// Setting a target relative to the grater
	mTargetRelativeOrient.SetRow(Vector3(-1, 0, 0), 0);
	mTargetRelativeOrient.SetRow(Vector3(0, 1, 0), 1);
	mTargetRelativeOrient.SetRow(Vector3(0, 0, -1), 2);

	vTargetRelativePos.Set(0.10+shift, 0.12, 0.30);

	mTargetRelativeFrame.SetOrientation(mTargetRelativeOrient);
	mTargetRelativeFrame.SetTranslation((vTargetRelativePos));

	vPosErrMult.Set(1, 1, 0.5);
	vOrientErrMult.Set(2, 2, 2); // >> higher values, Orientation will converge first, position second
					// these values are effective only for the joint position control
	genCart->SetWn(1);

	Vector cartForces(6);
	cartForces.Zero();
	mLWRRobot->SetCartForce(cartForces);
	bSyncCart = true;
	bSync = true;
	bTargetReached = false;
}

void kukaGratingModule::setTargetToSearchGrater() {
	oTargetObj = GetWorld()->Find("grater");
	if (!oTargetObj) GetConsole()->Print("ERROR: Target object not found");

	Vector3         vTargetRelativePos;     // the target position relative to the object we're trying to reach
	vTargetRelativePos.Set(0.10+shift, 0.12, -0.5);
	mTargetRelativeFrame.SetTranslation((vTargetRelativePos));

	vPosErrMult.Set(1, 1, 0.5);
	vOrientErrMult.Set(2, 2, 2); // >> higher values, Orientation will converge first, position second
					// these values are effective only for the joint position control

	genCart->SetWn(0.5);
	Vector cartForces(6);
	cartForces.Zero();
	mLWRRobot->SetCartForce(cartForces);
	bSyncCart = true;
	bSync = true;
	bTargetReached = false;
}


void kukaGratingModule::setTargetToGraterTop() {
	oTargetObj = GetWorld()->Find("grater");
	if (!oTargetObj) GetConsole()->Print("ERROR: Target object not found");

	Vector3         vTargetRelativePos;     // the target position relative to the object we're trying to reach
	vTargetRelativePos.Set(0.10+shift, 0.12, 0.27);
	mTargetRelativeFrame.SetTranslation((vTargetRelativePos));

	Matrix4 tmpm4;
	oTargetObj-> GetReferenceFrame().GetHMatrix().Mult(mTargetRelativeFrame, tmpm4);
	cout<<"Target: ";
	tmpm4.Print();
	vPosErrMult.Set(1, 1, 1);
	vOrientErrMult.Set(1, 1, 1);

	genCart->SetWn(1);

	bSyncCart = true;
	bSync = true;

	Vector cartForces(6);
	cartForces.Zero();
	mLWRRobot->SetCartForce(cartForces);
	bTargetReached = false;
}


void kukaGratingModule::setTargetToGraterBottom() {
	oTargetObj = GetWorld()->Find("grater");
	if (!oTargetObj) GetConsole()->Print("ERROR: Target object not found");

	Vector3         vTargetRelativePos;     // the target position relative to the object we're trying to reach
	vTargetRelativePos.Set(0.10+shift, -0.04, currentTarget(2)+0.03);
	mTargetRelativeFrame.SetTranslation((vTargetRelativePos));

	Matrix4 tmpm4;
	oTargetObj-> GetReferenceFrame().GetHMatrix().Mult(mTargetRelativeFrame, tmpm4);
	cout<<"Target: ";
	tmpm4.Print();

	vPosErrMult.Set(2, 2, 2);
	vOrientErrMult.Set(1, 1, 1);

	bSyncCart = true;
	bSync = true;
	genCart->SetWn(1.5);

	Vector cartForces(6);
	cartForces.Zero();
	cartForces(2) = des_force;
	curr_force = des_force;
	mLWRRobot->SetCartForce(cartForces);
	bTargetReached = false;

}

void kukaGratingModule::setTargetToBowl() {
	oTargetObj = GetWorld()->Find("bowl");
	if (!oTargetObj) GetConsole()->Print("ERROR: Target object not found");

	Matrix3         mTargetRelativeOrient;  // the target orientation relative to the object we're trying to reach
	Vector3         vTargetRelativePos;     // the target position relative to the object we're trying to reach

	// Setting a target relative to the bowl
	mTargetRelativeOrient = oTargetObj->GetReferenceFrame().GetOrient().Transpose()*mRobotEEAbsoluteOrient;
	vTargetRelativePos.Set(0.0, 0.15, 0.35);

	mTargetRelativeFrame.SetOrientation(mTargetRelativeOrient);
	mTargetRelativeFrame.SetTranslation((vTargetRelativePos));

	vPosErrMult.Set(1, 1, 2);
	vOrientErrMult.Set(1, 1, 1);

	genCart->SetWn(1);
	bSyncCart = true;
	bSync = true;
	Vector cartForces(6);
	cartForces.Zero();
	mLWRRobot->SetCartForce(cartForces);

	bTargetReached = false;
}

Vector kukaGratingModule::getHomePositionforSim(){

	Vector  vJointCommand;
	vJointCommand.Resize(mRobot->GetDOFCount());

	vJointCommand[mJointMapping[0]] = DEG2RAD(30);
	vJointCommand[mJointMapping[1]] = DEG2RAD(30);
	vJointCommand[mJointMapping[2]] = DEG2RAD(0);
	vJointCommand[mJointMapping[3]] = DEG2RAD(-98);
	vJointCommand[mJointMapping[4]] = DEG2RAD(0);
	vJointCommand[mJointMapping[5]] = DEG2RAD(45);
	vJointCommand[mJointMapping[6]] = DEG2RAD(45);

	return vJointCommand;
}


