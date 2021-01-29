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

private:
	vector<string> M_name_parameter_vec;
	vector<vector<string>> M_s_output_vecvec;
	vector<Eigen::Vector6d> M_trajectory_vec;


	void variParamaters(string dir_)
	{
		vector<float> parameter_vec_init;

		M_name_parameter_vec.push_back("voxel_size");
		M_name_parameter_vec.push_back("radius_normal_FPFH");
		M_name_parameter_vec.push_back("radius_FPFH");
		M_name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");
		M_name_parameter_vec.push_back("SimilarityThreshold_SAC");
		M_name_parameter_vec.push_back("InlierFraction_SAC");
		M_name_parameter_vec.push_back("MaximumIterations_SAC");
		M_name_parameter_vec.push_back("NumberOfSamples_SAC");
		M_name_parameter_vec.push_back("CorrespondenceRandomness_SAC");
		M_name_parameter_vec.push_back("max_RANSAC");

		cout << "0: registration of all frames and output files(.csv and .pcd)" << endl;
		cout << "1: output error of fpfh value (all frames)" << endl;
		cout << "2: output show FPFH variance (all frames)" << endl;


		bool b_create_new_pattern_file = false;
		cout << "do you create new pattern?  Yes:1  No:0" << endl;
		cout << "->";
		cin >> b_create_new_pattern_file;

		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");
		for (int i = 0; i < filenames_.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
			cloud->is_dense = true;
			M_cloud_vec.push_back(cloud);
		}

		//true trajectory
		//vector<Eigen::Vector6d> trajectory_vec;
		{
			string filename_true = "transformation_fin.csv";
			vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
			for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
			{
				Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
				Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
					trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
					trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
				M_trajectory_vec.push_back(Pos_temp);
			}

			//for (int i = 0; i < trajectory_vec.size(); i++)
			//{
			//	cout << "i:" << i;
			//	cout << " x:" << trajectory_vec[i](0, 0);
			//	cout << " y:" << trajectory_vec[i](1, 0);
			//	cout << " z:" << trajectory_vec[i](2, 0);
			//	cout << " roll:" << trajectory_vec[i](3, 0);
			//	cout << " pitch:" << trajectory_vec[i](4, 0);
			//	cout << " yaw:" << trajectory_vec[i](5, 0);
			//	cout << endl;
			//}

		}


		//{
		//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		//	string filename_;
		//	pcl::io::loadPCDFile(filename_, *cloud);
		//	cloud->is_dense = true;
		//	M_cloud_vec.push_back(cloud);
		//}



		if (b_create_new_pattern_file)
		{
			vector<vector<float>> pattern_vec_vec_new;
			//input parameter
			vector<vector<float>> parameter_vec_vec;
			//CTimeString::changeParameter_2dimension(parameter_vec_vec, name_parameter_vec, parameter_vec_arg);
			CTimeString::changeParameter_2dimension(parameter_vec_vec, M_name_parameter_vec, parameter_vec_init,
				dir_ + "/" + "parameter_vecvec.csv", 1, 4, -1, -1);
			pattern_vec_vec_new = CTimeString::calcVectorPairPattern(parameter_vec_vec);
			//write new parameter_vec_vec
			{
				vector<vector<string>> s_vec_vec;
				//header
				{
					vector<string> s_header_vec;
					//s_header_vec.push_back("Parameter");
					for (int i = 0; i < M_name_parameter_vec.size(); i++)
						s_header_vec.push_back(M_name_parameter_vec[i]);
					s_vec_vec.push_back(s_header_vec);
				}
				for (int j = 0; j < pattern_vec_vec_new.size(); j++)
				{
					vector<string> s_vec;
					for (int i = 0; i < pattern_vec_vec_new[j].size(); i++)
						s_vec.push_back(to_string(pattern_vec_vec_new[j][i]));
					s_vec_vec.push_back(s_vec);
				}
				CTimeString::getCSVFromVecVec(s_vec_vec, dir_ + "/" + "pattern_vec_vec.csv");
			}
		}

		cout << "press 1 and Enter if you have closed file" << endl;
		{
			int aa;
			cin >> aa;
		}

		//read pattern_vec_vec.csv
		vector<vector<float>> pattern_vec_vec;
		{
			vector<vector<string>> s_vec_vec;
			s_vec_vec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + "pattern_vec_vec.csv");
			for (int j = 1; j < s_vec_vec.size(); j++)
			{
				vector<float> pattern_vec;
				for (int i = 0; i < s_vec_vec[j].size(); i++)
					pattern_vec.push_back(stof(s_vec_vec[j][i]));
				pattern_vec_vec.push_back(pattern_vec);
			}
		}
		cout << "show pattern" << endl;
		for (int j = 0; j < pattern_vec_vec.size(); j++)
		{
			cout << j << ":";
			for (int i = 0; i < pattern_vec_vec[j].size(); i++)
			{
				string s_value;
				s_value = to_string(pattern_vec_vec[j][i]);
				if (s_value.size() < 4) s_value = " " + s_value;
				if (s_value.size() < 4) s_value = " " + s_value;
				if (s_value.size() < 4) s_value = " " + s_value;
				cout << "  " << s_value;
			}
			cout << endl;
		}

		for (int j = 0; j < pattern_vec_vec.size(); j++)
		{
			vector<float> parameter_vec = pattern_vec_vec[j];
			alignAllFrames(dir_ + "/varyParameters", parameter_vec);
			//switch (i_method)
			//{
			//case 0:
			//	CTimeString::showParameter(parameter_vec, name_parameter_vec);
			//	GR_FPFH_SAC_IA_Allframes(dir_, parameter_vec, false);
			//	break;
			//case 1:
			//	CTimeString::showParameter(parameter_vec, name_parameter_vec, 3);
			//	GR_FPFH_error_AllFrames(dir_, parameter_vec, false);
			//	break;
			//case 2:
			//	CTimeString::showParameter(parameter_vec, name_parameter_vec);
			//	GR_FPFH_variance_AllFrames(dir_, parameter_vec, false);
			//	break;
			//default:
			//	break;
			//}
		}

		cout << endl;

	}


	void alignAllFrames(string dir_, vector<float> parameter_vec)
	{
		{
			typedef pcl::PointXYZRGB T_PointType;
			bool b_useParameterAdjustment = false;
			bool b_show_AllFrame_AllPairs = false;
			bool b_useGeometricConstraints = false;
			bool b_useRigidTransformation = false;

			bool b_useNir = false;
			bool b_useVelodyne = false;
			bool b_useFPFH = false;

			bool b_useColorfullCorr = false;

			bool b_changeColor_nir = false;
			bool b_changeColor_velodyne = false;

			bool b_useOldFPFH = false;

			b_useParameterAdjustment = true;
			b_useGeometricConstraints = true;

			b_useNir = true;
			b_useVelodyne = true;
			b_useFPFH = true;

			//b_useOldFPFH = true;

			cout << "1: use FPFH   0: use proposed  ->";
			cin >> b_useOldFPFH;

			b_changeColor_nir = true;

			b_useRigidTransformation = true;

			if (!b_useRigidTransformation) b_show_AllFrame_AllPairs = true;

			if (!b_useNir)  b_changeColor_nir = false;

			inputData(dir_, b_useNir, b_useVelodyne, b_changeColor_nir, b_useFPFH, b_useOldFPFH);

			vector<pair<int, int>> index_pair_vec;

			//{
			//	int i_tgt = 5;
			//	int i_src = 6;
			//	index_pair_vec.push_back(make_pair(i_tgt, i_src));
			//}
			//{
			//	int i_tgt = 5;
			//	int i_src = 7;
			//	index_pair_vec.push_back(make_pair(i_tgt, i_src));
			//}
			for (int j = 0; j < M_cloud_vec.size() - 1; j++)
			{
				for (int i = j + 1; i < M_cloud_vec.size(); i++)
				{
					int i_tgt = j;
					int i_src = i;
					if (!(j == 5
						|| j == 6
						|| j == 7
						//|| j == 8
						|| j == 11
						//|| j == 12
						//|| j == 16
						)) continue;
					if (!(i == 5
						|| i == 6
						|| i == 7
						//|| i == 8
						|| i == 11
						//|| i == 12
						//|| i == 16
						)) continue;

					//if (!(j == 5
					//	|| j == 6
					//	|| j == 7
					//	|| j == 8
					//	)) continue;
					//if (!(i == 5
					//	|| i == 6
					//	|| i == 7
					//	|| i == 8
					//	)) continue;

					index_pair_vec.push_back(make_pair(i_tgt, i_src));
				}
			}

			float th_nearest_nir;
			float th_rank_rate_nir;
			float th_nearest_velodyne;
			float th_rank_rate_velodyne;
			float th_nearest_fpfh;
			int num_nearest_fpfh;
			float th_rank_rate_fpfh;
			int i_method_rigidTransformation;
			float th_geometricConstraint;


			bool b_first = true;

			while (1)
			{
				th_nearest_nir = 10.;
				//th_rank_rate_nir = 0.5;
				//th_rank_rate_nir = 1.;
				th_rank_rate_nir = 0.2;
				//cout << "input th_nearest_nir:";
				//cin >> th_nearest_nir;
				th_nearest_velodyne = 10.;
				//th_rank_rate_velodyne = 0.5;
				th_rank_rate_velodyne = 1.;

				th_nearest_fpfh = 1800.;
				num_nearest_fpfh = 10;
				//th_rank_rate_fpfh = 0.5;
				th_rank_rate_fpfh = 0.5;

				th_geometricConstraint = 0.8;

				if (b_useParameterAdjustment)
				{
					if (!b_first)
					{
						int aa;
						cout << "input txt:";
						cin >> aa;
					}
					b_first = false;
					vector<vector<string>> s_temp_vecvec;
					s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter2.csv");
					th_nearest_nir = stof(s_temp_vecvec[1][3]);
					th_rank_rate_nir = stof(s_temp_vecvec[2][3]);
					th_nearest_velodyne = stof(s_temp_vecvec[3][3]);
					th_rank_rate_velodyne = stof(s_temp_vecvec[4][3]);
					th_rank_rate_fpfh = stof(s_temp_vecvec[5][3]);
					th_geometricConstraint = stof(s_temp_vecvec[6][3]);
					i_method_rigidTransformation = stoi(s_temp_vecvec[7][3]);
					b_useRigidTransformation = (bool)stoi(s_temp_vecvec[8][3]);
				}


				if (b_useOldFPFH)
				{
					DoOldFPFHRegistration(index_pair_vec);
				}
				else
				{
					DoFeatureRegistration(index_pair_vec, th_nearest_nir, th_rank_rate_nir,
						th_nearest_velodyne, th_rank_rate_velodyne,
						th_nearest_fpfh, num_nearest_fpfh, th_rank_rate_fpfh,
						i_method_rigidTransformation, th_geometricConstraint,
						b_useNir, b_useVelodyne, b_useFPFH,
						b_useGeometricConstraints);
				}

				//evaluation
				DoEvaluation(dir_, index_pair_vec, b_useOldFPFH);

				if (b_useRigidTransformation)
					showRigidTransformation(index_pair_vec);

				else if (b_show_AllFrame_AllPairs)
					showAllPairs(index_pair_vec, b_useNir, b_useVelodyne, b_useFPFH, b_useGeometricConstraints, b_useColorfullCorr);

				if (!b_useParameterAdjustment) break;
			}
		}

		//typedef pcl::PointXYZ T_PointType;
		typedef pcl::PointXYZRGB T_PointType;

		int th_minute_CSV;
		th_minute_CSV = 20;
		//th_minute_CSV = 2;	//for debug

		bool b_useClusterNotification = false;
		b_useClusterNotification = true;

		bool b_useRANSAC_EST = false;
		b_useRANSAC_EST = true;

		//if (b_changeParameter)
		//	CTimeString::changeParameter(parameter_vec, name_parameter_vec);

		//parameter
		float voxel_size;
		voxel_size = parameter_vec[0];

		float radius_normal_FPFH, radius_FPFH;
		radius_normal_FPFH = parameter_vec[1];
		radius_FPFH = parameter_vec[2];

		float MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
		MaxCorrespondenceDistance_SAC = parameter_vec[3];
		SimilarityThreshold_SAC = parameter_vec[4];
		InlierFraction_SAC = parameter_vec[5];

		int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;
		MaximumIterations_SAC = (int)parameter_vec[6];
		NumberOfSamples_SAC = (int)parameter_vec[7];
		CorrespondenceRandomness_SAC = (int)parameter_vec[8];

		int max_RANSAC;
		max_RANSAC = (int)parameter_vec[9];

		initOutputStringVecVec(parameter_vec);

		int i_tgt_start = 0;
		//cout << "select first tgt frame" << endl;
		//cout << "i_tgt_start ->";
		//cin >> i_tgt_start;

		if (!(0 <= i_tgt_start && i_tgt_start <= M_cloud_vec.size() - 2))
		{
			cout << "ERROR: first frame is invalid and insert 0" << endl;
			i_tgt_start = 0;
		}

		string time_start = CTimeString::getTimeString();
		string time_regular = time_start;
		cout << "time_start:" << time_start << endl;
		//make new folder
		string s_newfoldername = time_start;
		CTimeString::makenewfolder(dir_, s_newfoldername);

		for (int i = 0; i < M_cloud_vec.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(M_cloud_vec[i]);
			sor->filter(*cloud_VGF);
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
			fpfh = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, M_cloud_vec[i], radius_normal_FPFH, radius_FPFH);
			M_fpfh_vec.push_back(fpfh);
		}

		string time_end_FPFH = CTimeString::getTimeString();
		cout << "time_end_FPFH:" << time_end_FPFH << endl;

		vector<pair<int, int>> frame_pair_vec;
		frame_pair_vec = getFramePairVec(dir_);

		vector<pair<int, int>> frame_pair_est;

		for (int i_frame_pair = 0; i_frame_pair < frame_pair_vec.size(); i_frame_pair++)
		{
			int i_tgt = frame_pair_vec[i_frame_pair].first;
			int i_src = frame_pair_vec[i_frame_pair].second;
			cout << "i_tgt:" << i_tgt << " i_src:" << i_src << endl;

			vector<string> s_result_vec;
			s_result_vec = alignAllFrames_OneFramePair(dir_ + "/" + s_newfoldername, parameter_vec, i_tgt, i_src, M_cloud_vec, M_fpfh_vec,
				b_useRANSAC_EST, true);
			string time_end_frame = CTimeString::getTimeString();

			bool b_hasConverged = false;

			if (stoi(s_result_vec[8]) == 1)
				b_hasConverged = true;

			if (b_useRANSAC_EST)
			{
				if (b_hasConverged)
					frame_pair_est.push_back(make_pair(i_tgt, i_src));
				cout << "show frames estimated success" << endl;
				for (int j = 0; j < frame_pair_est.size(); j++)
					cout << "i_tgt:" << frame_pair_est[j].first << " i_src:" << frame_pair_est[j].second << endl;
			}

			M_s_output_vecvec.push_back(s_result_vec);

			//regular saving csv
			string s_elapsed_frame = CTimeString::getTimeElapsefrom2Strings(time_regular, time_end_frame);
			cout << "time_elapsed from last .csv output: " << s_elapsed_frame << endl;
			cout << "time_elapsed from start:            " << CTimeString::getTimeElapsefrom2Strings(time_start, time_end_frame) << endl;
			int elapsed_millisec = CTimeString::getTimeElapsefrom2Strings_millisec(time_regular, time_end_frame);
			int elapsed_minute = (int)(((float)elapsed_millisec / 1000.) / 60.);
			if (elapsed_minute >= th_minute_CSV)
			{
				//save
				CTimeString::getCSVFromVecVec(M_s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_regular + "_output.csv");
				time_regular = CTimeString::getTimeString();
				//clear s_output_vecvec
				M_s_output_vecvec.clear();
				GR_addToOutputString_OutputHeader_FPFH(M_s_output_vecvec);
			}
			cout << endl;

			if (i_frame_pair % 5 == 0 && !b_changeParameter)
			{
				cout << "Parameter list" << endl;
				CTimeString::showParameter(parameter_vec, M_name_parameter_vec);
				cout << endl;
			}
		}

		string time_end = CTimeString::getTimeString();
		string time_elapsed = CTimeString::getTimeElapsefrom2Strings(time_start, time_end);
		cout << "time_elapsed:" << time_elapsed << endl;

		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("");
			M_s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Sum elapsed time");
			M_s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back(time_elapsed);
			M_s_output_vecvec.push_back(s_temp_vec);
		}

		CTimeString::getCSVFromVecVec(M_s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_end + "_output.csv");

		GR_FPFH_getResultAnalysis(dir_, s_newfoldername);

		//cluster size
		if (b_useClusterNotification)
		{
			vector<vector<string>> s_vecvec;
			{
				vector<vector<string>> s_input_vecvec;
				vector<string> filenames_;
				CTimeString::getFileNames_extension(dir_ + "/" + s_newfoldername, filenames_, "_SucEst.csv");
				if (filenames_.size() != 1)
				{
					cout << "ERROR: one _SucEst.csv have not been found" << endl;
					return;
				}
				s_input_vecvec = CTimeString::getVecVecFromCSV_string(
					dir_ + "/" + s_newfoldername + "/" + filenames_[0]);
				s_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(s_input_vecvec, "Result", 2, "Sum elapsed time", -2);
			}

			vector<vector<int>> pairs_vecvec;
			for (int j = 0; j < s_vecvec.size(); j++)
			{
				if (stoi(s_vecvec[j][20]) == 0) continue;
				vector<int> pairs_vec;
				int i_tgt = stoi(s_vecvec[j][0]);
				int i_src = stoi(s_vecvec[j][1]);
				pairs_vec.push_back(i_tgt);
				pairs_vec.push_back(i_src);
				pairs_vecvec.push_back(pairs_vec);
			}
			vector<vector<int>> cluster_vecvec;
			cluster_vecvec = CTimeString::getIntCluster_SomeToSome(pairs_vecvec);
			cout << "cluster_vecvec[0].size():" << cluster_vecvec[0].size() << endl;
			if (cluster_vecvec[0].size() >= 11)
			{
				vector<vector<string>> s_temp;
				vector<string> s_temp2;
				s_temp.push_back(s_temp2);
				CTimeString::getCSVFromVecVec(s_temp, dir_ + "/" + s_newfoldername + "_hasClusterSize" + to_string(cluster_vecvec[0].size()) + ".csv");
			}
		}

		cout << endl;

	}

	void initOutputStringVecVec(vector<float> parameter_vec)
	{
		//push_back parameter information to s_output_vecvec
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Parameter");
			M_s_output_vecvec.push_back(s_temp_vec);
		}
		for (int i = 0; i < M_name_parameter_vec.size(); i++)
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back(M_name_parameter_vec[i]);
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(parameter_vec[i]));
			M_s_output_vecvec.push_back(s_temp_vec);
		}

		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("");
			M_s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Result");
			M_s_output_vecvec.push_back(s_temp_vec);
		}

		//GR_addToOutputString_OutputHeader_FPFH(s_output_vecvec);
		vector<string> s_temp_vec;
		s_temp_vec.push_back("target");
		s_temp_vec.push_back("source");
		s_temp_vec.push_back("tgt size");
		s_temp_vec.push_back("src size");
		s_temp_vec.push_back("tgt VGF size");
		s_temp_vec.push_back("src VGF size");
		s_temp_vec.push_back("inlier size");
		s_temp_vec.push_back("inlier rate");
		s_temp_vec.push_back("convergence");
		s_temp_vec.push_back("success_frame");
		s_temp_vec.push_back("distance");
		s_temp_vec.push_back("median");
		s_temp_vec.push_back("fitness");
		s_temp_vec.push_back("time");
		s_temp_vec.push_back("X");
		s_temp_vec.push_back("Y");
		s_temp_vec.push_back("Z");
		s_temp_vec.push_back("ROLL");
		s_temp_vec.push_back("PITCH");
		s_temp_vec.push_back("YAW");
		M_s_output_vecvec.push_back(s_temp_vec);

	}

	vector<pair<int, int>> getFramePairVec(string dir_)
	{
		vector<pair<int, int>> frame_pair_vec;

		vector<vector<bool>> b_ignore_vecvec;
		vector<vector<bool>> b_ahead_vecvec;

		//input from text
		vector<vector<string>> s_matrix_vecvec;
		{
			//input text
			vector<vector<string>> s_matrix_vecvec_temp;
			s_matrix_vecvec_temp = CTimeString::getVecVecFromCSV_string(dir_ + "/matrix_ignore_ahead.csv");
			for (int j = 1; j < s_matrix_vecvec_temp.size(); j++)
			{
				vector<string> s_frame_vec;
				for (int i = 1; i < s_matrix_vecvec_temp[j].size(); i++)
				{
					s_frame_vec.push_back(s_matrix_vecvec_temp[j][i]);
				}
				s_matrix_vecvec.push_back(s_frame_vec);
			}

			//init ignore and ahead
			for (int j = 0; j < s_matrix_vecvec.size(); j++)
			{
				vector<bool> b_vecvec;
				for (int i = 0; i < s_matrix_vecvec.size(); i++)
					b_vecvec.push_back(false);
				b_ignore_vecvec.push_back(b_vecvec);
			}
			for (int j = 0; j < s_matrix_vecvec.size(); j++)
			{
				vector<bool> b_vecvec;
				for (int i = 0; i < s_matrix_vecvec.size(); i++)
					b_vecvec.push_back(false);
				b_ahead_vecvec.push_back(b_vecvec);
			}

			//extract from vecvec
			for (int i_tgt = 0; i_tgt < s_matrix_vecvec.size(); i_tgt++)
			{
				for (int i_src = 0; i_src < s_matrix_vecvec.size(); i_src++)
				{
					string s_value = s_matrix_vecvec[i_tgt][i_src];
					if (s_value.size() == 0 || s_value == "-")
						continue;
					else if (stoi(s_value) == -1)
					{
						b_ignore_vecvec[i_tgt][i_src] = true;
						cout << "i_tgt:" << i_tgt << " i_src:" << i_src << "  ignored" << endl;

					}
					else if (stoi(s_value) == 1)
					{
						b_ahead_vecvec[i_tgt][i_src] = true;
						cout << "i_tgt:" << i_tgt << " i_src:" << i_src << "  aheaded" << endl;
					}
				}
			}
		}

		//ahead loop
		for (int i_tgt = 0; i_tgt < s_matrix_vecvec.size() - 1; i_tgt++)
		{
			for (int i_src = i_tgt + 1; i_src < s_matrix_vecvec.size(); i_src++)
			{
				if (!b_ahead_vecvec[i_tgt][i_src]) continue;
				frame_pair_vec.push_back(make_pair(i_tgt, i_src));
				b_ignore_vecvec[i_tgt][i_src] = true;
			}
		}

		//normal loop
		for (int i_tgt = 0; i_tgt < s_matrix_vecvec.size() - 1; i_tgt++)
		{
			for (int i_src = i_tgt + 1; i_src < s_matrix_vecvec.size(); i_src++)
			{
				if (b_ignore_vecvec[i_tgt][i_src]) continue;
				frame_pair_vec.push_back(make_pair(i_tgt, i_src));
				b_ignore_vecvec[i_tgt][i_src] = true;
			}
		}

		cout << endl;
		return frame_pair_vec;

	}

	vector<string> alignAllFrames_OneFramePair(string dir_, vector<float> parameter_vec, int i_tgt, int i_src,
		vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> cloud_vec, vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec,
		bool b_useRANSAC_EST, bool b_useOutputPointcloud)
	{
		//parameter
		float voxel_size;
		voxel_size = parameter_vec[0];

		float radius_normal_FPFH, radius_FPFH;
		radius_normal_FPFH = parameter_vec[1];
		radius_FPFH = parameter_vec[2];

		float MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
		MaxCorrespondenceDistance_SAC = parameter_vec[3];
		SimilarityThreshold_SAC = parameter_vec[4];
		InlierFraction_SAC = parameter_vec[5];

		int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;
		MaximumIterations_SAC = (int)parameter_vec[6];
		NumberOfSamples_SAC = (int)parameter_vec[7];
		CorrespondenceRandomness_SAC = (int)parameter_vec[8];

		int max_RANSAC;
		max_RANSAC = (int)parameter_vec[9];

		string time_start_frame = CTimeString::getTimeString();
		bool b_hasConverged = false;
		vector<int> inlier_;
		float fitnessscore;
		int frame_failed = 0;
		Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
		bool b_cout_RANSAC = false;
		if (!b_useRANSAC_EST)
		{
			b_hasConverged = CFPFH_PCL::align_SAC_AI_RANSAC<pcl::PointXYZRGB>(transform_, inlier_, fitnessscore, frame_failed,
				cloud_vec[i_src], fpfh_vec[i_src], cloud_vec[i_tgt], fpfh_vec[i_tgt],
				voxel_size, MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC,
				MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC, max_RANSAC, b_cout_RANSAC);
		}
		else
		{
			Eigen::Matrix4d T_src_TRUE = Eigen::Matrix4d::Identity();
			T_src_TRUE = calcHomogeneousMatrixFromVector6d(M_trajectory_vec[i_tgt]).inverse()
				* calcHomogeneousMatrixFromVector6d(M_trajectory_vec[i_src]);

			b_hasConverged = CFPFH_PCL::align_SAC_AI_RANSAC_TRUE<pcl::PointXYZRGB>(transform_, inlier_, fitnessscore, frame_failed,
				cloud_vec[i_src], fpfh_vec[i_src], cloud_vec[i_tgt], fpfh_vec[i_tgt],
				voxel_size, MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC,
				MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC,
				max_RANSAC, b_cout_RANSAC, T_src_TRUE, 2.3);
		}
		//save pointcloud
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>());
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_src_true(new pcl::PointCloud<pcl::PointXYZRGB>());
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_tgt(new pcl::PointCloud<pcl::PointXYZRGB>());
		cloud_src->clear();
		cloud_src_true->clear();
		cloud_tgt->clear();
		//VGF
		const boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZRGB>> sor(new pcl::VoxelGrid<pcl::PointXYZRGB>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_vec[i_src]);
		sor->filter(*cloud_src);
		sor->setInputCloud(cloud_vec[i_tgt]);
		sor->filter(*cloud_tgt);
		//color
		for (int i = 0; i < cloud_tgt->size(); i++)
		{
			cloud_tgt->points[i].r = 255;
			cloud_tgt->points[i].g = 0;
			cloud_tgt->points[i].b = 0;
		}
		for (int i = 0; i < cloud_src->size(); i++)
		{
			cloud_src->points[i].r = 0;
			cloud_src->points[i].g = 255;
			cloud_src->points[i].b = 0;
		}
		//transform
		pcl::copyPointCloud(*cloud_src, *cloud_src_true);		//evaluation
		{
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(transform_);
			pcl::transformPointCloud(*cloud_src, *cloud_src, Trans_temp);
		}
		//distance
		double distance_ = 0.;
		{
			Eigen::Matrix4d T_i_src = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d T_i1_tgt = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d T_i_GL = Eigen::Matrix4d::Identity();
			//T_i_src = T_i1_tgt * T_i_GL
			T_i_src = calcHomogeneousMatrixFromVector6d(M_trajectory_vec[i_src]);
			T_i1_tgt = calcHomogeneousMatrixFromVector6d(M_trajectory_vec[i_tgt]);
			T_i_GL = T_i1_tgt.inverse() * T_i_src;
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(T_i_GL);
			pcl::transformPointCloud(*cloud_src_true, *cloud_src_true, Trans_temp);
			//distance to true
			for (size_t i = 0; i < cloud_src->size(); i++)
			{
				pcl::PointXYZRGB point_, point_true;
				point_ = cloud_src->points[i];
				point_ = cloud_src->points.at(i);
				point_true = cloud_src_true->points[i];
				const float sqrt_before =
					pow(point_.x - point_true.x, 2.)
					+ pow(point_.y - point_true.y, 2.)
					+ pow(point_.z - point_true.z, 2.);
				distance_ += static_cast<double>(sqrt(
					pow(point_.x - point_true.x, 2.)
					+ pow(point_.y - point_true.y, 2.)
					+ pow(point_.z - point_true.z, 2.)));

			}
			if (cloud_src->size() != 0) distance_ /= cloud_src->size();
			else distance_ = 100.;
			cout << "distance_:" << distance_ << endl;
		}
		//median
		double median_ = getMedianDistance(cloud_src, cloud_tgt);
		cout << "median_:" << median_ << endl;
		//add for saving
		*cloud_tgt += *cloud_src;
		//filename for saving
		string s_filename_output;
		{
			string s_src;
			s_src = to_string(i_src);
			if (s_src.size() < 3) s_src = "0" + s_src;
			if (s_src.size() < 3) s_src = "0" + s_src;
			string s_tgt;
			s_tgt = to_string(i_tgt);
			if (s_tgt.size() < 3) s_tgt = "0" + s_tgt;
			if (s_tgt.size() < 3) s_tgt = "0" + s_tgt;
			string s_convergence;
			if (b_hasConverged) s_convergence = to_string(1);
			else s_convergence = to_string(0);
			s_filename_output = "T" + s_tgt + "S" + s_src + "C" + s_convergence + "_XYZRGB.pcd";
		}
		//save
		if (b_useOutputPointcloud)
			pcl::io::savePCDFile<pcl::PointXYZRGB>(dir_ + "/" + s_filename_output, *cloud_tgt);
		//output csv
		Eigen::Vector6d transform_vec = Eigen::Vector6d::Zero();
		transform_vec = calcVector6dFromHomogeneousMatrix(transform_);
		string time_end_frame = CTimeString::getTimeString();
		string time_elapsed_frame = CTimeString::getTimeElapsefrom2Strings(time_start_frame, time_end_frame);
		vector<string> s_temp_vec;
		s_temp_vec.push_back(to_string(i_tgt));
		s_temp_vec.push_back(to_string(i_src));
		s_temp_vec.push_back(to_string(cloud_vec[i_tgt]->size()));
		s_temp_vec.push_back(to_string(cloud_vec[i_src]->size()));
		s_temp_vec.push_back(to_string(fpfh_vec[i_tgt]->size()));
		s_temp_vec.push_back(to_string(fpfh_vec[i_src]->size()));
		s_temp_vec.push_back(to_string(inlier_.size()));
		s_temp_vec.push_back(to_string((float)inlier_.size() / (float)fpfh_vec[i_src]->size()));
		s_temp_vec.push_back(to_string((int)b_hasConverged));
		s_temp_vec.push_back(to_string(max_RANSAC - frame_failed) + "(/" + to_string(max_RANSAC) + ")");
		s_temp_vec.push_back(to_string(distance_));
		s_temp_vec.push_back(to_string(median_));
		s_temp_vec.push_back(to_string(fitnessscore));
		s_temp_vec.push_back(time_elapsed_frame);
		s_temp_vec.push_back(to_string(transform_vec(0, 0)));	//X
		s_temp_vec.push_back(to_string(transform_vec(1, 0)));	//Y
		s_temp_vec.push_back(to_string(transform_vec(2, 0)));	//Z
		s_temp_vec.push_back(to_string(transform_vec(3, 0)));	//ROLL
		s_temp_vec.push_back(to_string(transform_vec(4, 0)));	//PITCH
		s_temp_vec.push_back(to_string(transform_vec(5, 0)));	//YAW
		return s_temp_vec;
	}
};
