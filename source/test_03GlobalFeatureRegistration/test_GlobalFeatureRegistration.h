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

public:
	void inputData(string dir_, bool b_useNir, bool b_useVelodyne, bool b_changeColor_nir,
		bool b_useFPFH, bool b_useOldFPFH);
	void DoOldFPFHRegistration(vector<pair<int, int>> index_pair_vec, vector<float> parameter_vec);
	void DoFeatureRegistration(vector<pair<int, int>> index_pair_vec, vector<float> parameter_vec,
		bool b_useNir, bool b_useVelodyne, bool b_useFPFH);
	void fillParameterToTXT(vector<vector<string>> &s_output_vecvec, vector<float> parameter_oldFPFH_vec,
		vector<float> parameter_featureRegistration_vec);
	vector<pair<int, int>> getFramePairVec(string dir_);
	vector<vector<string>> DoEvaluation(string dir_save, vector<pair<int, int>> index_pair_vec, bool b_useProposed,
		bool b_useNir, bool b_useVelodyne, bool b_useFPFH, bool b_cout = false);
	void alignAllFrames(string dir_, vector<float> parameter_oldFPFH_vec, vector<float> parameter_featureRegistration_vec, int i_method);
	void variParamaters(string dir_);

	void estimateSucceededFrames(string dir_)
	{
		M_name_parameter_vec.clear();

		bool b_isGotParameterName = false;
		bool b_isProposed = false;

		//read folder name
		vector<string> s_folder_vec;
		CTimeString::getFileNames_folder(dir_, s_folder_vec);
		for (int j = s_folder_vec.size() - 1; j >= 0; j--)
			if (s_folder_vec[j] == "_Ignore") s_folder_vec.erase(s_folder_vec.begin() + j);

		vector<vector<string>> s_output_vecvec;
		for (int j = 0; j < s_folder_vec.size(); j++)
		{
			//read text
			vector<vector<string>> s_txt_vecvec;
			{
				vector<string> s_filename_vec;
				CTimeString::getFileNames_extension(dir_ + "/" + s_folder_vec[j], s_filename_vec, ".csv");
				if (s_filename_vec.size() == 0)
				{
					cout << "ERROR: No .csv found and continuing." << endl;
					continue;
				}
				s_txt_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder_vec[j] + "/" + s_filename_vec[0]);
				int i_find = s_filename_vec[0].find("conventional");
				if (i_find == std::string::npos) b_isProposed = true;

			}

			//get information of parameter
			vector<float> parameter_vec;
			{
				vector<vector<string>> s_vecvec_temp =
					CTimeString::getMatrixData_fromSpecificAreaOfMatrix(s_txt_vecvec, "Parameter_oldFPFH", 1, "Result", -2, 1);
				for (int i = 0; i < s_vecvec_temp.size(); i++)
				{
					if (i == 9) continue;
					if (!b_isGotParameterName)
						M_name_parameter_vec.push_back(s_vecvec_temp[i][0]);
					parameter_vec.push_back(stof(s_vecvec_temp[i][4]));
				}
				if (M_name_parameter_vec.size() != 0) b_isGotParameterName = true;
			}

			//isProposed

			//get information of result
			vector<vector<string>> s_result_vecvec;
			s_result_vecvec = CTimeString::getMatrixData_fromSpecificAreaOfMatrix(s_txt_vecvec, "Result", 2, "time_elapsed:", -2, 23);

			//estimation
			vector<vector<int>> framePair_success_vecvec;
			for (int i = 0; i < s_result_vecvec.size(); i++)
			{
				int i_tgt, i_src;
				i_tgt = stoi(s_result_vecvec[i][0]);
				i_src = stoi(s_result_vecvec[i][1]);
				bool b_isConverged = (bool)(stoi(s_result_vecvec[i][12]));
				float e_error_PointCloudDistance = stof(s_result_vecvec[i][18]);

				float th_success = 6.;
				bool b_estimatedSuccess = false;
				if (e_error_PointCloudDistance < th_success && b_isConverged) b_estimatedSuccess = true;
				if (b_estimatedSuccess)
				{
					vector<int> framePair_success_vec;
					framePair_success_vec.push_back(i_tgt);
					framePair_success_vec.push_back(i_src);
					framePair_success_vecvec.push_back(framePair_success_vec);
				}
			}

			//calc cluster size
			vector<vector<int>> cluster_vecvec;
			cluster_vecvec = CTimeString::getIntCluster_SomeToSome(framePair_success_vecvec);

			//output to string
			vector<string> s_output_vec;
			s_output_vec.push_back(s_folder_vec[j]);
			for (int i = 0; i < parameter_vec.size(); i++)
				s_output_vec.push_back(to_string(parameter_vec[i]));

			//b_isProposed
			s_output_vec.push_back(to_string((int)b_isProposed));

			int num_allFramePairs = s_result_vecvec.size();
			int num_succeededFramePairs = framePair_success_vecvec.size();
			s_output_vec.push_back(to_string(num_allFramePairs));
			s_output_vec.push_back(to_string(num_succeededFramePairs));

			//succeededFramePairs
			{
				string s_output = "";
				for (int i = 0; i < framePair_success_vecvec.size(); i++)
				{
					s_output +=
						to_string(framePair_success_vecvec[i][0]) + "-"
						+ to_string(framePair_success_vecvec[i][1]) + " ";
				}
				s_output_vec.push_back(s_output);
			}

			//biggestCluster
			if (cluster_vecvec.size() > 0)
			{
				string s_output;
				for (int i = 0; i < cluster_vecvec[0].size(); i++)
					s_output += to_string(cluster_vecvec[0][i]) + " ";
				s_output_vec.push_back(s_output);
			}

			//size_biggestCluster
			if (cluster_vecvec.size() > 0)
				s_output_vec.push_back(to_string(cluster_vecvec[0].size()));

			//second_biggestCluster
			if (cluster_vecvec.size() > 1)
			{
				string s_output;
				for (int i = 0; i < cluster_vecvec[1].size(); i++)
					s_output += to_string(cluster_vecvec[1][i]) + " ";
				s_output_vec.push_back(s_output);
			}

			//calc matrix?

			s_output_vecvec.push_back(s_output_vec);
		}

		//header
		{
			vector<string> s_output_vec;
			s_output_vec.push_back("");
			for (int j = 0; j < M_name_parameter_vec.size(); j++)
				s_output_vec.push_back(M_name_parameter_vec[j]);
			s_output_vec.push_back("b_isProposed");
			s_output_vec.push_back("num_allFramePairs");
			s_output_vec.push_back("num_succeededFramePairs");
			s_output_vec.push_back("succeededFramePairs");
			s_output_vec.push_back("biggestCluster");
			s_output_vec.push_back("size_biggestCluster");
			s_output_vec.push_back("second_biggestCluster");
			s_output_vecvec.insert(s_output_vecvec.begin(), s_output_vec);
		}

		//output
		vector<vector<string>> s_output_vecvec_transposed;
		s_output_vecvec_transposed = CTimeString::getTranspositionOfVecVec(s_output_vecvec);
		CTimeString::getCSVFromVecVec(s_output_vecvec_transposed, dir_ + "/"  + CTimeString::getTimeString() + "_estimation.csv");
	}
};
