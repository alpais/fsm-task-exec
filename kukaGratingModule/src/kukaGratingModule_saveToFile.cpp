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


void kukaGratingModule::saveCurrentRobotState(){

    if (crt_cart_pos.size() < 50000) {
	nTaskId.push_back(mCurrentTask);
	Vector3 cartPos; cartPos.Zero();
	Matrix3 cartRot; cartRot.Identity();
	if (mRobot->IsSimulationMode() || ctrlmode == Robot::CTRLMODE_POSITION) {
		cartPos    = mRobot->GetReferenceFrame(nEndEffectorId).GetOrigin();
		cartRot = mRobot->GetReferenceFrame(nEndEffectorId).GetHMatrix().GetOrientation();
    	} else {
	mLWRRobot->GetMeasuredCartPose(cartPos, cartRot);
	}
	crt_cart_pos.push_back(cartPos);
	crt_cart_or.push_back(cartRot.GetExactRotationAxis());

	Vector cartForces;
	cartForces.Resize(6);
	cartForces = mLWRRobot->GetEstimatedExternalCartForces();
	crt_cart_for.push_back(cartForces);	

	Vector vJntTorques;
	vJntTorques.Resize(7);
	vJntTorques = mLWRRobot->GetMeasuredJointTorques();
	crt_jnt_tqs.push_back(vJntTorques);

	}

}

void kukaGratingModule::saveToFile(int fid){
    //fid = 1;
    if(fid < 0) {
	GetConsole()->Print("ERROR: File index cannot be negative");
    }
    char buf[1024];
    sprintf(buf, "./packages/addons/kukaGratingTaskPkg/data/recordings/data_%03d.txt", fid);
    FILE* file = NULL;
    file = fopen(buf,"w");
    if (file == NULL) {
	GetConsole()->Print((string("ERROR: Cannot open file ")+buf).c_str());
    }

    // reducing number of points saved to avoid a segmentation fault problem
    for (int i = 0; i < crt_cart_pos.size() - 2; i++) {
	fprintf(file, "   %lf   %lf   %lf   %lf   ", nTaskId[i], crt_cart_pos[i].At(0), crt_cart_pos[i].At(1), crt_cart_pos[i].At(2));
	fprintf(file, "   %lf   %lf   %lf  ", crt_cart_or[i].At(0),  crt_cart_or[i].At(1), crt_cart_or[i].At(2));
	fprintf(file, "   %lf   %lf   %lf   %lf   %lf   %lf   ", crt_cart_for[i].At(0), crt_cart_for[i].At(1), crt_cart_for[i].At(2), crt_cart_for[i].At(3), crt_cart_for[i].At(4), crt_cart_for[i].At(5));
	fprintf(file, "   %lf   %lf   %lf   %lf   %lf   %lf   %lf   \n", crt_jnt_tqs[i].At(0), crt_jnt_tqs[i].At(1), crt_jnt_tqs[i].At(2), crt_jnt_tqs[i].At(3), crt_jnt_tqs[i].At(4), crt_jnt_tqs[i].At(5), crt_jnt_tqs[i].At(6));
    }

    fclose(file);
    cout << "file saved" << endl;

	crt_cart_pos.clear();
	crt_cart_or.clear();
	crt_cart_for.clear();
	crt_jnt_tqs.clear();
	nTaskId.clear();
}


