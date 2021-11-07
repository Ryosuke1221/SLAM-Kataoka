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
	void showAllPairs(vector<pair<int, int>> index_pair_vec, bool b_useNir, bool b_useVelodyne, bool b_useFPFH,
		bool b_useGeometricConstraints, bool b_useColorfullCorr);
	void showRigidTransformation(vector<pair<int, int>> index_pair_vec);

private:
	vector<string> M_name_parameter_vec;
	vector<vector<string>> M_s_output_vecvec;
	vector<Eigen::Vector6d> M_trajectory_vec;
	vector<Eigen::Vector6d> M_trajectory_true_vec;
	string M_t_elapsed;
	vector<pcl::Correspondences> M_corrs_nir_vec;
	vector<pcl::Correspondences> M_corrs_velodyne_vec;
	vector<pcl::Correspondences> M_corrs_fpfh_vec;
	vector<pcl::Correspondences> M_corrs_output_vec;
	//20211107 add some bools to member variable
	bool M_b_changeColor_r_RGB;
	bool M_b_changeColor_g_RGB;
	bool M_b_calcUniquePointOfFPFHInSomeRadius;

public:
	void inputData(string dir_, vector<float> parameter_oldFPFH_vec, bool b_useNir, bool b_useVelodyne, 
		bool b_useFPFH, bool b_useOldFPFH);
	void DoOldFPFHRegistration(vector<pair<int, int>> index_pair_vec, vector<float> parameter_vec);
	void DoFeatureRegistration(vector<pair<int, int>> index_pair_vec, vector<float> parameter_vec,
		bool b_useNir, bool b_useVelodyne, bool b_useFPFH);
	void fillParameterToTXT(vector<float> parameter_oldFPFH_vec, vector<float> parameter_featureRegistration_vec);
	vector<pair<int, int>> getFramePairVec(string dir_);
	vector<vector<string>> DoEvaluation(string dir_save, vector<pair<int, int>> index_pair_vec, bool b_useProposed,
		bool b_useNir, bool b_useVelodyne, bool b_useFPFH, bool b_cout = false);
	void alignAllFrames(string dir_, vector<float> parameter_oldFPFH_vec, vector<float> parameter_featureRegistration_vec, int i_method);
	void variParamaters(string dir_);
	void compareGlobalRegistration(string dir_);

private:
	struct SInitPos_ICP
	{
		int i_tgt;
		int i_src;
		Eigen::Vector6d Init_Vector;
		SInitPos_ICP() { Init_Vector = Eigen::Vector6d::Zero(); }
	};
	vector<SInitPos_ICP> M_initPos_vec;
	vector<vector<int>> M_chara_vecvec;
	vector<bool> M_b_isConverged_vec;
	vector<bool> M_b_estimatedSuccess_vec;

public:
	void inputData_ICP(string dir_);
	void inputData_ICP_initPos(string dir_, bool b_calcOnlyBiggestCluster,
		float th_successOfGlobalRegistration_distance, bool &b_isProposed);
	void fillParameterToTXT_ICP(vector<float> parameter_vec);
	vector<vector<string>> DoEvaluation_ICP(string dir_save, float th_successOfICP_distance, bool b_useProposed, bool b_cout);
	vector<vector<string>> calcBiggestFrameCluster_ICP();
	void align_ICP_AllFrames(string dir_, string s_folder_arg, vector<float> parameter_vec);
	void align_ICP_fromGlobalRegistration_variParamaters(string dir_);
	void compareICP(string dir_);
	vector<vector<bool>> calcMatrixOfRemovingFramePairs(string filename_);

};
