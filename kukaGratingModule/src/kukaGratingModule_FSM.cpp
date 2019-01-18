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

// =====================================================
// ========    Control the Task Flow     ===============
// =====================================================

 void kukaGratingModule::advanceStateMachine() {
	switch(mCurrentTask) {
	case TASK_START: // start new grating session
		cout << "Starting the grating task!" << endl;
		nGratingPasses = 0;
		setTargetToGrater();
		initializeCDSController(SEGM_REACH, MODEL_DYNAMICS);
		initializeStiffness(SEGM_REACH);
		mNextTask = TASK_REACH_GRATER_TOP;
		break;
	case TASK_REACH_GRATER_TOP:
		nGratingPasses++;
		if (nGratingPasses <= nGratingTotalPasses){
			cout<<"Moving to top"<<endl;
			setTargetToGraterTop();
			mNextTask = TASK_SEARCH_GRATER;
		}
		else{
			cout << "Done, going to trash." << endl;
			setTargetToBowl();
			cout << "Reaching the Trash" << endl;
			mNextTask = TASK_NONE;
			mCurrentTask = TASK_REACH_TRASH;
		}
		break;
	case TASK_SEARCH_GRATER: // continue grating when touching the grater
		cout << "Searching grater" << endl;
		setTargetToSearchGrater();
		mNextTask = TASK_REACH_GRATER_BOTTOM;
		break;
	case TASK_REACH_GRATER_BOTTOM: // reached bottom of grater, continue or go to trash
		cout << "Moving to bottom" << endl;
		cout << "Grater pass " << nGratingPasses << endl;
		setTargetToGraterBottom();
		initializeCDSController(SEGM_GRATE, LINEAR_DYNAMICS);
		initializeStiffness(SEGM_GRATE);
		initializeForce(SEGM_GRATE);
		mNextTask = TASK_REACH_GRATER_TOP;
		break;
	case TASK_REACH_TRASH:
		setTargetToBowl();
		initializeCDSController(SEGM_TRASH, MODEL_DYNAMICS);
		initializeStiffness(SEGM_TRASH);
		cout << "Reaching the Trash" << endl;
		mNextTask = TASK_NONE;
		break;
	case TASK_NONE:
		cout << "Done..." << endl;
		mState = MODE_NONE;
		break;
	}

	bSync = true;
}


