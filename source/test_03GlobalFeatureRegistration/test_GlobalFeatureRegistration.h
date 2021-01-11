#pragma once

#include "PointcloudBasicProcess.h"
#include "GlobalFeatureRegistration.h"

using namespace std;

typedef pcl::PointXYZRGB T_PointType;


class CGlobalFeatureRegistration_test : public CPointcloudBasicProcess
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

private:
	vector<pcl::PointCloud<T_PointType>::Ptr> M_cloud_vec;
	vector<vector<float>> M_feature_vecvec_nir;
	vector<vector<float>> M_feature_vecvec_velodyne;
	vector<vector<int>> M_index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> M_fpfh_vec;

	vector<Eigen::Matrix4d> M_transformation_vec;

	vector<vector<pcl::Correspondences>> M_corrs_all_vecvec;
	vector<vector<float>> M_evaluation_corr_vecvec_nir;
	vector<vector<float>> M_evaluation_corr_vecvec_velodyne;
	vector<vector<float>> M_evaluation_corr_vecvec_fpfh;

	void DoDifferential_PairEvaluation3(string dir_);
	void inputData(string dir_, bool b_useNir, bool b_useVelodyne, bool b_changeColor_nir,
		bool b_useFPFH, bool b_useOldFPFH);
	void DoOldFPFHRegistration(vector<pair<int, int>> index_pair_vec);
	void DoFeatureRegistration(vector<pair<int, int>> index_pair_vec,
		float th_nearest_nir, float th_rank_rate_nir,
		float th_nearest_velodyne, float th_rank_rate_velodyne,
		float th_nearest_fpfh, int num_nearest_fpfh, float th_rank_rate_fpfh,
		int i_method_rigidTransformation, float th_geometricConstraint,
		bool b_useNir, bool b_useVelodyne, bool b_useFPFH,
		bool b_useGeometricConstraints);
	void DoEvaluation(string dir_, vector<pair<int, int>> index_pair_vec, bool b_useOldFPFH);
	void showAllPairs(vector<pair<int, int>> index_pair_vec, bool b_useNir, bool b_useVelodyne, bool b_useFPFH,
		bool b_useGeometricConstraints, bool b_useColorfullCorr);
	void showRigidTransformation(vector<pair<int, int>> index_pair_vec);

	void variParamaters(string dir_);
	void alignAllFrames(string dir_, vector<float> parameter_vec);
};
