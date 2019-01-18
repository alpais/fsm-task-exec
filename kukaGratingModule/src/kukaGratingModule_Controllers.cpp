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


// oTargetObj is a global variable updated everytime we set a new target. Can change between the grater and the bowl
// mTargetRelativeFrame is a global variable, representing the attractor, updated everytime we perform a new task segment. 


void kukaGratingModule::initializeCDSController(int segmentID, int dynamicsType){

	if (bCDSinitialized){
		delete cdsRun;
		bCDSinitialized = false;
	}
	else {
		cdsRun = new CDSExecution();
		cdsRun->initSimple(segmentID); 
		cdsRun->setObjectFrame(oTargetObj->GetReferenceFrame().GetHMatrix());
		cdsRun->setAttractorFrame(mTargetRelativeFrame);
		cdsRun->setCurrentEEPose(getCrtEEFullFrame());
		cdsRun->setDT(module_dt);
		cdsRun->setMotionParameters(1,1,1,reachingThreshold, dynamicsType);
		cdsRun->postInit(); 
		cout << " >> CDS Controller Initialized for reaching" << endl;
		bCDSinitialized = true;
	}

}

void kukaGratingModule::initializeForce(int segmentID){

	if (bForceInitialized){
		delete mForceModel;
		bForceInitialized = false;
	}
	else{
	        char sForce[256];
        	sprintf(sForce, "data/packages/kukaGratingTaskPkg/Models/s%d/forceGMM.txt", segmentID);

		int in_dim  = 1; 
		int out_dim = 1;
	
		mForceModel = new GMR();
		mForceModel->initGMR(in_dim,out_dim);
		if(!mForceModel->getIsInit())
		{
			cout<<"ERROR could not innitialize force model"<<endl;
			exit(1);
		}
		else
			bForceInitialized = true;
	}

}

void kukaGratingModule::initializeStiffness(int segmentID){

	if (bStiffnessInitialized){
		delete mStiffnessModel;
		bStiffnessInitialized = false;
	}
	else{
	        char sForce[256];
	        sprintf(sForce, "data/packages/kukaGratingTaskPkg/Models/s%d/stiffnessGMM.txt", segmentID);

		int in_dim  = 3; 
		int out_dim = 3;

		mStiffnessModel = new GMR();
		mStiffnessModel->initGMR(in_dim,out_dim);
		if(!mStiffnessModel->getIsInit())
		{
			cout<<"ERROR could not innitialize stiffness model"<<endl;
			exit(1);
		}
		else
			bStiffnessInitialized = true;
	}

}

