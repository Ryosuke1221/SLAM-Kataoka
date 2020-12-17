#pragma once

#include "PointcloudBasic.h"
#include"KataokaPCL.h"
#include"FPFH_PCL.h"

using namespace std;

class CGlobalFeatureRegistration : public CPointcloudBasic
{
public:
	void mainProcess();
	void FreeSpace();
	void DoDifferential_1pointcloud(string dir_);
	void DoDifferential_SomePointclouds(string dir_);
	void FPFH_unique(string dir_);
	void DoDifferential_showFeatureValue(string dir_);
	void DoDifferential_RigidTransformation_FPFH_Features(string dir_);
	void DoDifferential_RigidTransformation_FPFH_Features_new(string dir_);
	void DoDifferential_RigidTransformation_FPFH_Features_allFrames(string dir_);
	void DoDifferential_PairEvaluation(string dir_);
	void DoDifferential_PairEvaluation2(string dir_);
	void DoDifferential_PairEvaluation3(string dir_);
};
