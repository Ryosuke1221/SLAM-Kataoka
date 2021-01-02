#include "test_ICPWithFeature.h"

void CICPWithFeature::mainProcess()
{
	int WhichProcess = 0;
	string filename1, filename2;
	bool b_finish = false;
	enum OPTION {
		EN_escape = 0,
		EN_FreeSpace,
		EN_FileProcess,
		EN_SequentShow,
		EN_DrawTrajectory,
		EN_DoMappingFromTrajectory,
		EN_GlobalRegistration_FPFH_SAC_IA,
		EN_DoICP_proposed_AllFrames,
		EN_DoEvaluation_ICP_property
	};

	while (!b_finish)
	{
		cout << endl;
		cout << "please input process number" << endl;
		cout << " " << EN_escape << ": escape" << endl;
		cout << " " << EN_FreeSpace << ": free space" << endl;
		cout << " " << EN_FileProcess << ": FileProcess" << endl;
		cout << " " << EN_SequentShow << ": sequent show" << endl;
		cout << " " << EN_DrawTrajectory << ": DrawTrajectory" << endl;
		cout << " " << EN_DoMappingFromTrajectory << ": DoMappingFromTrajectory" << endl;
		cout << " " << EN_GlobalRegistration_FPFH_SAC_IA << ": GlobalRegistration_FPFH_SAC_IA" << endl;
		cout << " " << EN_DoICP_proposed_AllFrames << ": DoICP_proposed_AllFrames" << endl;
		cout << " " << EN_DoEvaluation_ICP_property << ": DoEvaluation_ICP_property" << endl;

		cout << "WhichProcess: ";
		cin >> WhichProcess;

		cout << endl;
		cout << "//////////////////////////////////" << endl;
		cout << endl;

		string dir_ = "../../data/data_test_02ICPWithFeature";

		switch (WhichProcess)
		{
		case EN_escape:
			//escape
			b_finish = true;
			break;

		case EN_FreeSpace:
			CICPWithFeature::FreeSpace();
			break;

		case EN_FileProcess:
			FileProcess(dir_);
			break;

		case EN_SequentShow:
			show_sequent_PointTypes(dir_);
			break;

		case EN_DrawTrajectory:
			DrawTrajectory(dir_ + "/DrawTrajectory");
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory(dir_ + "/MappingFromTrajectory");
			break;

		case EN_GlobalRegistration_FPFH_SAC_IA:
			GlobalRegistration_FPFH_SAC_IA(dir_ + "/01_GR_FPFH_SAC_IA");
			break;

		case EN_DoICP_proposed_AllFrames:
			DoICP_proposed_AllFrames(dir_ + "/02_DoICP_proposed_AllFrames");
			break;

		case EN_DoEvaluation_ICP_property:
			DoEvaluation_ICP_property(dir_ + "/03_DoEvaluation_ICP_property");
			break;

		default:
			break;
		}

	}
}

void CICPWithFeature::FreeSpace()
{

}

void CICPWithFeature::GlobalRegistration_FPFH_SAC_IA(string dir_)
{
	vector<float> parameter_vec;

	float voxel_size;
	//voxel_size = 0.01;
	//voxel_size = 0.05;
	voxel_size = 0.1;

	float radius_normal_FPFH, radius_FPFH;
	//radius_normal_FPFH = voxel_size * 2.0;
	radius_normal_FPFH = 0.5;
	//radius_FPFH = voxel_size * 5.0;
	radius_FPFH = 1.;

	float MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
	//MaxCorrespondenceDistance_SAC = voxel_size * 2.5;
	MaxCorrespondenceDistance_SAC = 0.25;
	//MaxCorrespondenceDistance_SAC = 0.5;
	int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;
	//SimilarityThreshold_SAC = 0.9f;
	//SimilarityThreshold_SAC = 0.75f;
	SimilarityThreshold_SAC = 0.01f;
	//SimilarityThreshold_SAC = 0.5f;
	//SimilarityThreshold_SAC = 0.75f;
	InlierFraction_SAC = 0.25f;
	//InlierFraction_SAC = 0.15f;
	//MaximumIterations_SAC = 500000;
	MaximumIterations_SAC = 500;
	//MaximumIterations_SAC = 100;
	//MaximumIterations_SAC = 50000;
	//MaximumIterations_SAC = 10000;
	//NumberOfSamples_SAC = 4;//8 & 8
	//NumberOfSamples_SAC = 10;
	NumberOfSamples_SAC = 10;
	//CorrespondenceRandomness_SAC = 2;
	CorrespondenceRandomness_SAC = 10;

	int max_RANSAC;
	//max_RANSAC = 50;
	max_RANSAC = 20;
	//max_RANSAC = 5;

	parameter_vec.push_back(voxel_size);
	parameter_vec.push_back(radius_normal_FPFH);
	parameter_vec.push_back(radius_FPFH);
	parameter_vec.push_back(MaxCorrespondenceDistance_SAC);
	parameter_vec.push_back(SimilarityThreshold_SAC);
	parameter_vec.push_back(InlierFraction_SAC);
	parameter_vec.push_back((float)MaximumIterations_SAC);
	parameter_vec.push_back((float)NumberOfSamples_SAC);
	parameter_vec.push_back((float)CorrespondenceRandomness_SAC);
	parameter_vec.push_back((float)max_RANSAC);

	while (1)
	{
		cout << "0: registration of 2 frames" << endl;
		cout << "1: registration of all frames and output files(.csv and .pcd)" << endl;
		cout << "2: output fusion and matrix(of convergence) by .csv" << endl;
		cout << "3: output ESTs of patterns by .csv" << endl;
		cout << "4: output error of fpfh value (2 frames)" << endl;
		cout << "5: output error of fpfh value (all frames)" << endl;
		cout << "6: output show FPFH variance (all frames)" << endl;
		cout << "7: vary parameter by some patterns (all frames)" << endl;
		cout << "8: fix fusion (iterate FPFH in specific pair)" << endl;
		cout << "select ->";
		int i_method;
		cin >> i_method;

		if (i_method == 0)
			GR_FPFH_SAC_IA_2frames(dir_, parameter_vec);
		else if (i_method == 1)
			GR_FPFH_SAC_IA_Allframes(dir_, parameter_vec);
		else if (i_method == 2)
		{
			vector<string> filenames_folder;
			vector<int> i_folder_vec;
			{
				CTimeString::getFileNames_folder(dir_, filenames_folder);
				for (int i = 0; i < filenames_folder.size(); i++)
				{
					string s_i = to_string(i);
					if (s_i.size() < 2) s_i = " " + s_i;
					cout << "i:" << s_i << " " << filenames_folder[i] << endl;
				}
				cout << endl;
				cout << "input folder you want to calc (can input multinumber)" << endl;
				vector<string> s_input_vec;
				bool b_useCSV = false;
				{
					cout << "select: input number by csv or not  yes:1  no:0" << endl;
					cout << "->";
					cin >> b_useCSV;
				}
				if (!b_useCSV) s_input_vec = CTimeString::inputSomeString();
				else s_input_vec = CTimeString::inputSomeString_fromCSV(dir_ + "/" + "num_vector.csv");

				cout << "s_input_vec.size():" << s_input_vec.size() << endl;
				for (int i = 0; i < s_input_vec.size(); i++)
					i_folder_vec.push_back(stoi(s_input_vec[i]));
			}
			for (int i = 0; i < i_folder_vec.size(); i++)
				GR_FPFH_getResultAnalysis(dir_, filenames_folder[i_folder_vec[i]]);
		}
		else if (i_method == 3)
			GR_FPFH_getResultOfPatterns(dir_);
		else if (i_method == 4)
			GR_FPFH_error(dir_, parameter_vec);
		else if (i_method == 5)
			GR_FPFH_error_AllFrames(dir_, parameter_vec);
		else if (i_method == 6)
			GR_FPFH_variance_AllFrames(dir_, parameter_vec);
		else if (i_method == 7)
			GR_FPFH_varyParameter(dir_, parameter_vec);
		else if (i_method == 8)
		{
			vector<string> filenames_folder;
			{
				CTimeString::getFileNames_folder(dir_, filenames_folder);
				for (int i = 0; i < filenames_folder.size(); i++)
				{
					string s_i = to_string(i);
					if (s_i.size() < 2) s_i = " " + s_i;
					cout << "i:" << s_i << " " << filenames_folder[i] << endl;
				}
				cout << endl;
			}
			cout << "input folder ->";
			int i_selected;
			cin >> i_selected;
			GR_FPFH_FixFusion(dir_, filenames_folder[i_selected]);
		}

		else break;

		cout << endl;
	}

	return;
}

void CICPWithFeature::GR_FPFH_getResultAnalysis(string dir_, string s_folder)
{
	cout << "calc " << s_folder << endl;
	GR_FPFH_makeFusionCSV(dir_, s_folder);

	GR_FPFH_makeSuccessEstimation(dir_ + "/" + s_folder);
	cout << endl;
}

void CICPWithFeature::GR_FPFH_makeFusionCSV(string dir_, string s_folder)
{
	//get vecvec from .csv s
	vector<string> filenames_csv;
	CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_csv, "_output.csv");
	vector<vector<string>> s_output_vecvec;
	for (int j = 0; j < filenames_csv.size(); j++)	//file iteration
	{
		vector<vector<string>> s_output_vecvec_temp;
		s_output_vecvec_temp = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder + "/" + filenames_csv[j]);
		if (j == 0) s_output_vecvec = s_output_vecvec_temp;
		else
		{
			for (int i = 0; i < s_output_vecvec_temp.size(); i++)	//rows iteration
			{
				if (i == 0) continue;
				s_output_vecvec.push_back(s_output_vecvec_temp[i]);
			}
		}
	}

	//avoid error of "not found Sum elapsed time"
	if (s_output_vecvec[s_output_vecvec.size() - 2][0] != "Sum elapsed time")
	{
		cout << "ERROR: Sum_elapsed_time not found" << endl;
		return;
	}

	//sort
	{
		//insert
		vector<vector<string>> s_output_vecvec_front;
		vector<vector<string>> s_output_vecvec_forSort;
		vector<vector<string>> s_output_vecvec_back;
		for (int j = 0; j < s_output_vecvec.size(); j++)
		{
			if (j < 14) s_output_vecvec_front.push_back(s_output_vecvec[j]);
			else s_output_vecvec_forSort.push_back(s_output_vecvec[j]);
			if (s_output_vecvec[j + 2][0] == "Sum elapsed time") break;
		}
		for (int j = s_output_vecvec.size() - 3; j < s_output_vecvec.size(); j++)
			s_output_vecvec_back.push_back(s_output_vecvec[j]);
		//do sorting
		{
			vector<vector<int>> sort_vecvec;
			for (int j = 0; j < s_output_vecvec_forSort.size(); j++)
			{
				vector<int> sort_vec;
				sort_vec.push_back(stoi(s_output_vecvec_forSort[j][0]));
				sort_vec.push_back(stoi(s_output_vecvec_forSort[j][1]));
				sort_vec.push_back(j);
				sort_vecvec.push_back(sort_vec);
			}
			CTimeString::sortVector2d_2dimension(sort_vecvec, 0, 1, false);
			for (int j = 0; j < sort_vecvec.size(); j++)
				s_output_vecvec_front.push_back(s_output_vecvec_forSort[sort_vecvec[j][2]]);
		}
		//output
		for (int j = 0; j < s_output_vecvec_back.size(); j++)
			s_output_vecvec_front.push_back(s_output_vecvec_back[j]);
		s_output_vecvec.clear();
		s_output_vecvec = s_output_vecvec_front;
	}

	string s_folder_new;
	{
		vector<int> find_vec = CTimeString::find_all(s_folder, "_checked");
		if (0 != find_vec.size()) s_folder_new = s_folder.substr(0, find_vec[0]);
		else s_folder_new = s_folder;
	}
	CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_folder + "/"
		+ s_folder_new + "_output_fusion.csv");
}

vector<vector<string>> CICPWithFeature::GR_FPFH_makeMatrix(vector<vector<int>> int_vecvec)
{
	//get frame size
	int frame_end = 0;
	for (int j = 0; j < int_vecvec.size(); j++)
	{
		if (int_vecvec[j][1] > frame_end)
			frame_end = int_vecvec[j][1];
	}

	//init s_output_vecvec
	vector<vector<string>> s_output_vecvec;
	for (int j = 0; j < frame_end + 1; j++)
	{
		vector<string> s_output_vec;
		s_output_vec.resize(frame_end + 1);
		s_output_vecvec.push_back(s_output_vec);
	}

	//fill except value cell
	for (int j = 0; j < s_output_vecvec.size(); j++)
	{
		for (int i = 0; i < s_output_vecvec[j].size(); i++)
		{
			if (j == i || j > i) s_output_vecvec[j][i] = "-";
		}
	}

	//fill value cell
	for (int j = 0; j < int_vecvec.size(); j++)
	{
		int i_tgt, i_src;
		bool b_convergence = false;
		int EstSuc;
		i_tgt = int_vecvec[j][0];
		i_src = int_vecvec[j][1];
		EstSuc = int_vecvec[j][2];
		s_output_vecvec[i_tgt][i_src] = to_string(EstSuc);
	}

	//make matrix
	s_output_vecvec = CTimeString::getMatrixCSVFromVecVec(s_output_vecvec);

	return s_output_vecvec;
}

void CICPWithFeature::GR_FPFH_makeSuccessEstimation(string dir_)
{
	//input file name
	vector<string> filenames_csv;
	//CTimeString::getFileNames_extension(dir_, filenames_csv, "_fusion_matrix.csv");
	CTimeString::getFileNames_extension(dir_, filenames_csv, "_fusion.csv");
	if (filenames_csv.size() != 1)
	{
		cout << "ERROR: one fusion.csv have not been found" << endl;
		return;
	}

	//get vecvec from .csv s
	vector<vector<string>> s_value_vecvec_EachRows;
	vector<vector<string>> s_output_vecvec;
	{
		vector<vector<string>> s_input_vecvec;
		s_input_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + filenames_csv[0]);
		s_value_vecvec_EachRows = CTimeString::getMatrixData_fromFormatOfFPFH(s_input_vecvec, "Result", 2, "Sum elapsed time", -2);
		s_output_vecvec = s_input_vecvec;
	}

	//get frame size
	int frame_end = 0;
	for (int j = 0; j < s_value_vecvec_EachRows.size(); j++)
	{
		if (stoi(s_value_vecvec_EachRows[j][1]) > frame_end)
			frame_end = stoi(s_value_vecvec_EachRows[j][1]);
	}

	vector<int> num_success_vec;
	num_success_vec.resize(frame_end + 1);
	fill(num_success_vec.begin(), num_success_vec.end(), 0);
	vector<int> num_called_vec;
	num_called_vec.resize(frame_end + 1);
	fill(num_called_vec.begin(), num_called_vec.end(), 0);
	float th_distance;
	th_distance = 2.3;	//10

	vector<vector<int>> i_est_vecvec_EachRows;
	vector<vector<int>> frames_forClustering_vecvec;
	for (int j = 0; j < s_value_vecvec_EachRows.size(); j++)
	{
		vector<int> i_est_vec_EachRows;
		int i_tgt, i_src;
		i_tgt = stoi(s_value_vecvec_EachRows[j][0]);
		i_src = stoi(s_value_vecvec_EachRows[j][1]);

		i_est_vec_EachRows.push_back(i_tgt);
		i_est_vec_EachRows.push_back(i_src);

		num_called_vec[i_tgt]++;
		num_called_vec[i_src]++;

		if (!(th_distance >= stof(s_value_vecvec_EachRows[j][10])))
			i_est_vec_EachRows.push_back(0);
		else
		{
			num_success_vec[i_tgt]++;
			num_success_vec[i_src]++;
			i_est_vec_EachRows.push_back(1);
			vector<int> frames_forClustering_vec;
			frames_forClustering_vec.push_back(i_tgt);
			frames_forClustering_vec.push_back(i_src);
			frames_forClustering_vecvec.push_back(frames_forClustering_vec);
		}
		i_est_vecvec_EachRows.push_back(i_est_vec_EachRows);
	}

	s_output_vecvec[13].push_back("suc_est");

	for (int j = 0; j < i_est_vecvec_EachRows.size(); j++)
		s_output_vecvec[j + 14].push_back(to_string(i_est_vecvec_EachRows[j][2]));

	{
		vector<string> s_vec_temp;
		s_output_vecvec.push_back(s_vec_temp);
	}
	{
		vector<string> s_vec_temp;
		s_vec_temp.push_back("Success_Estimation");
		s_output_vecvec.push_back(s_vec_temp);
	}

	//show success
	int num_suc_est_sum = 0;
	int num_called_sum = 0;
	for (int j = 0; j < num_success_vec.size(); j++)
	{
		vector<string> s_vec_temp;
		s_vec_temp.push_back(to_string(j));
		s_vec_temp.push_back(to_string(num_success_vec[j]) + "/(" + to_string(num_called_vec[j]) + ")");
		s_output_vecvec.push_back(s_vec_temp);
		num_suc_est_sum += num_success_vec[j];
		num_called_sum += num_called_vec[j];
	}

	{
		vector<string> s_vec_temp;
		s_vec_temp.push_back("sum");
		s_vec_temp.push_back(to_string(num_suc_est_sum) + "/(" + to_string(num_called_sum) + ")");
		s_output_vecvec.push_back(s_vec_temp);
	}
	{
		vector<string> s_vec_temp;
		s_output_vecvec.push_back(s_vec_temp);
	}

	//clustering
	{
		vector<vector<int>> frames_forClustering_vecvec_new;
		frames_forClustering_vecvec_new =
			CTimeString::getIntCluster_SomeToSome(frames_forClustering_vecvec);
		//show biggest cluster
		vector<string> s_vec_temp;
		s_vec_temp.push_back("cluster");
		string s_cluster;
		for (int i = 0; i < frames_forClustering_vecvec_new[0].size(); i++)
			s_cluster = s_cluster + to_string(frames_forClustering_vecvec_new[0][i]) + " ";
		s_vec_temp.push_back(s_cluster);
		s_output_vecvec.push_back(s_vec_temp);
		//show size of biggest cluster
		s_vec_temp.clear();
		s_vec_temp.push_back("cluster_size");
		s_vec_temp.push_back(to_string(frames_forClustering_vecvec_new[0].size()));
		s_output_vecvec.push_back(s_vec_temp);
	}

	{
		vector<string> s_vec_temp;
		s_output_vecvec.push_back(s_vec_temp);
	}
	{
		vector<string> s_vec_temp;
		s_vec_temp.push_back("Matrix_SucEst");
		s_output_vecvec.push_back(s_vec_temp);
	}

	//make matrix
	vector<vector<string>> s_suc_mat_vecvec;
	s_suc_mat_vecvec = GR_FPFH_makeMatrix(i_est_vecvec_EachRows);

	//push back matrix
	for (int j = 0; j < s_suc_mat_vecvec.size(); j++)
		s_output_vecvec.push_back(s_suc_mat_vecvec[j]);

	//output file
	string filename_;
	filename_ = filenames_csv[0].substr(0, filenames_csv[0].size() - 4) + "_SucEst.csv";
	CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + filename_);
}

void CICPWithFeature::GR_FPFH_getResultOfPatterns(string dir_)
{
	vector<string> filenames_folder;
	vector<int> i_folder_vec;
	{
		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc (can input multinumber)" << endl;
		vector<string> s_input_vec;
		bool b_useCSV = false;
		{
			cout << "select: input number by csv or not  yes:1  no:0" << endl;
			cout << "->";
			cin >> b_useCSV;
		}
		if (!b_useCSV) s_input_vec = CTimeString::inputSomeString();
		else s_input_vec = CTimeString::inputSomeString_fromCSV(dir_ + "/" + "num_vector.csv");
		cout << "s_input_vec.size():" << s_input_vec.size() << endl;
		for (int i = 0; i < s_input_vec.size(); i++)
			i_folder_vec.push_back(stoi(s_input_vec[i]));
	}

	vector<vector<string>> s_output_vecvec;
	{
		vector<string> s_output_vec;
		s_output_vec.push_back("");
		s_output_vec.push_back("voxel_size");
		s_output_vec.push_back("radius_normal_FPFH");
		s_output_vec.push_back("radius_FPFH");
		s_output_vec.push_back("MaxCorrespondenceDistance_SAC");
		s_output_vec.push_back("SimilarityThreshold_SAC");
		s_output_vec.push_back("InlierFraction_SAC");
		s_output_vec.push_back("MaximumIterations_SAC");
		s_output_vec.push_back("NumberOfSamples_SAC");
		s_output_vec.push_back("CorrespondenceRandomness_SAC");
		s_output_vec.push_back("max_RANSAC");
		s_output_vec.push_back("est");
		s_output_vec.push_back("failed");
		s_output_vec.push_back("cluster_biggest");
		s_output_vec.push_back("cluster_size");
		s_output_vecvec.push_back(s_output_vec);
	}

	for (int i = 0; i < i_folder_vec.size(); i++)
		s_output_vecvec.push_back(
			GR_FPFH_getResultOfOnePattern(dir_, filenames_folder[i_folder_vec[i]]));

	//transposition
	{
		vector<vector<string>> s_output_vecvec_temp;
		s_output_vecvec_temp = CTimeString::getTranspositionOfVecVec(s_output_vecvec);
		s_output_vecvec = s_output_vecvec_temp;
	}

	string s_t = CTimeString::getTimeString();
	CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/ESTResult_" + s_t + ".csv");
}

vector<string> CICPWithFeature::GR_FPFH_getResultOfOnePattern(string dir_, string s_folder)
{
	vector<string> s_vec_output;
	{
		string s_folder_clern;
		vector<int> find_vec = CTimeString::find_all(s_folder, "_checked");
		if (0 == find_vec.size()) s_folder_clern = s_folder;
		else s_folder_clern = s_folder.substr(0, find_vec[0]);
		s_vec_output.push_back(s_folder_clern);
	}

	vector<vector<string>> s_vecvec_temp;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_, "_SucEst.csv");
		if (filenames_.size() != 1)
		{
			cout << "ERROR: one fusion.csv have not been found" << endl;
			return s_vec_output;
		}
		s_vecvec_temp = CTimeString::getVecVecFromCSV_string(
			dir_ + "/" + s_folder + "/" + filenames_[0]);
	}

	for (int j = 1; j < 11; j++)
		s_vec_output.push_back(s_vecvec_temp[j][4]);

	vector<vector<string>> s_vecvec_est;
	{
		bool b_useLine = false;
		for (int j = 0; j < s_vecvec_temp.size(); j++)
		{
			if (b_useLine)
				s_vecvec_est.push_back(s_vecvec_temp[j]);
			if ("Success_Estimation" == s_vecvec_temp[j][0])
				b_useLine = true;
			if ("cluster" == s_vecvec_temp[j + 2][0]) break;
		}
	}
	//extract sum value
	{
		string s_sum = s_vecvec_est.back()[1];
		vector<int> find_vec = CTimeString::find_all(s_sum, "/");
		s_vec_output.push_back(s_sum.substr(0, find_vec[0]));
	}

	//extract alone frames
	vector<bool> b_alone_vec;
	for (int j = 0; j < s_vecvec_est.size() - 1; j++)
	{
		string s_success = s_vecvec_est[j][1];
		vector<int> find_vec = CTimeString::find_all(s_success, "/");
		bool b_alone = false;
		if (stoi(s_success.substr(0, find_vec[0])) == 0)
			b_alone = true;
		b_alone_vec.push_back(b_alone);
	}
	string s_frames;
	for (int i = 0; i < b_alone_vec.size(); i++)
	{
		if (b_alone_vec[i])
			s_frames += to_string(i) + " ";
	}
	s_vec_output.push_back(s_frames);

	//cluster
	vector<vector<string>> s_vecvec_cluster;
	{
		bool b_useLine = false;
		for (int j = 0; j < s_vecvec_temp.size(); j++)
		{
			if (b_useLine)
				s_vecvec_cluster.push_back(s_vecvec_temp[j]);
			if ("cluster" == s_vecvec_temp[j + 1][0])
				b_useLine = true;
			if ("Matrix_SucEst" == s_vecvec_temp[j + 2][0]) break;
		}
	}
	//extract cluster value
	{
		string s_cluster = s_vecvec_cluster[0][1];
		string s_size_cluster = s_vecvec_cluster[1][1];
		s_vec_output.push_back(s_cluster);
		s_vec_output.push_back(s_size_cluster);
	}
	return s_vec_output;
}

void CICPWithFeature::GR_FPFH_SAC_IA_2frames(string dir_, vector<float> parameter_vec)
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	vector<string> name_parameter_vec;
	name_parameter_vec.push_back("voxel_size");
	name_parameter_vec.push_back("radius_normal_FPFH");
	name_parameter_vec.push_back("radius_FPFH");
	name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");
	name_parameter_vec.push_back("SimilarityThreshold_SAC");
	name_parameter_vec.push_back("InlierFraction_SAC");
	name_parameter_vec.push_back("MaximumIterations_SAC");
	name_parameter_vec.push_back("NumberOfSamples_SAC");
	name_parameter_vec.push_back("CorrespondenceRandomness_SAC");
	name_parameter_vec.push_back("max_RANSAC");

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	int i_tgt, i_src;

	pcl::PointCloud<T_PointType>::Ptr cloud_temp(new pcl::PointCloud<T_PointType>());

	while (1)
	{
		cout << "select frame  (ESCAPE by typing same frame or -1 )" << endl;
		cout << "i_tgt ->";
		cin >> i_tgt;
		if (i_tgt == -1 || i_tgt > filenames_.size() - 1) break;
		cout << "i_src ->";
		cin >> i_src;
		if (i_src == -1 || i_src > filenames_.size() - 1) break;

		if (i_tgt == i_src) break;

		CTimeString::changeParameter(parameter_vec, name_parameter_vec);

		cout << "calculation start" << endl;

		//input parameter to variable
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


		//input pointcloud
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i_tgt], *cloud_tgt);
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i_src], *cloud_src);
		cloud_tgt->is_dense = true;
		cloud_src->is_dense = true;

		//compute fpfh
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(cloud_tgt);
			sor->filter(*cloud_VGF);
			fpfh_tgt = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_tgt, radius_normal_FPFH, radius_FPFH);
		}
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(cloud_src);
			sor->filter(*cloud_VGF);
			fpfh_src = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_src, radius_normal_FPFH, radius_FPFH);
		}

		bool b_hasConverged = false;
		vector<int> inlier_;
		float fitnessscore;
		int frame_failed = 0;
		Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
		bool b_cout_RANSAC = false;

		cout << "i_tgt:" << i_tgt << " i_src" << i_src << endl;

		b_hasConverged = CFPFH_PCL::align_SAC_AI_RANSAC<T_PointType>(transform_, inlier_, fitnessscore, frame_failed,
			cloud_src, fpfh_src, cloud_tgt, fpfh_tgt,
			voxel_size, MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC,
			MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC, max_RANSAC, b_cout_RANSAC);

		cout << "b_hasConverged: " << b_hasConverged << endl;
		cout << "fitnessscore: " << fitnessscore << endl;
		cout << "inlierrate: " << (float)inlier_.size() / (float)fpfh_src->points.size() << endl;
		cout << "frame_failed: " << frame_failed << endl;

		//show pointcloud
		//save pointcloud
		pcl::PointCloud<T_PointType>::Ptr cloud_src_show(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt_show(new pcl::PointCloud<T_PointType>());
		//VGF
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_src);
		sor->filter(*cloud_src_show);
		sor->setInputCloud(cloud_tgt);
		sor->filter(*cloud_tgt_show);
		//color
		for (int i = 0; i < cloud_tgt_show->size(); i++)
		{
			cloud_tgt_show->points[i].r = 255;
			cloud_tgt_show->points[i].g = 0;
			cloud_tgt_show->points[i].b = 0;
		}
		for (int i = 0; i < cloud_src_show->size(); i++)
		{
			cloud_src_show->points[i].r = 0;
			cloud_src_show->points[i].g = 255;
			cloud_src_show->points[i].b = 0;
		}
		//add to init
		CPointVisualization<T_PointType> pv_init;
		pv_init.setWindowName("Initial");
		CPointVisualization<T_PointType> pv_global;
		pv_global.setWindowName("Global");

		cloud_temp->clear();
		*cloud_temp = *cloud_tgt_show + *cloud_src_show;
		pv_init.setPointCloud(cloud_temp);
		//transform
		{
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(transform_);
			pcl::transformPointCloud(*cloud_src_show, *cloud_src_show, Trans_temp);
		}
		//add to global
		cloud_temp->clear();
		*cloud_temp = *cloud_tgt_show + *cloud_src_show;
		pv_global.setPointCloud(cloud_temp);
		cout << "Press ESC to next registration" << endl;
		cout << endl;
		//show
		while (1)
		{
			pv_init.updateViewer();
			pv_global.updateViewer();
			if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
		}
		//remove viewer
		pv_init.closeViewer();
		pv_global.closeViewer();

		cout << "one frame process finished" << endl;
		cout << endl;
	}


	cout << "escaped" << endl;
}

void CICPWithFeature::GR_FPFH_SAC_IA_Allframes(string dir_, vector<float> parameter_vec, bool b_changeParameter)
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	int th_minute_CSV;
	th_minute_CSV = 20;
	//th_minute_CSV = 2;	//for debug

	bool b_useClusterNotification = false;
	b_useClusterNotification = true;

	bool b_useRANSAC_EST = false;
	b_useRANSAC_EST = true;

	vector<string> name_parameter_vec;
	name_parameter_vec.push_back("voxel_size");
	name_parameter_vec.push_back("radius_normal_FPFH");
	name_parameter_vec.push_back("radius_FPFH");
	name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");
	name_parameter_vec.push_back("SimilarityThreshold_SAC");
	name_parameter_vec.push_back("InlierFraction_SAC");
	name_parameter_vec.push_back("MaximumIterations_SAC");
	name_parameter_vec.push_back("NumberOfSamples_SAC");
	name_parameter_vec.push_back("CorrespondenceRandomness_SAC");
	name_parameter_vec.push_back("max_RANSAC");

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	//true trajectory
	vector<Eigen::Vector6d> trajectory_vec;
	{
		string filename_true = "transformation_fin.csv";
		vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
		for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
				trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
				trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
			trajectory_vec.push_back(Pos_temp);
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

	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	vector<vector<string>> s_output_vecvec;

	if (b_changeParameter)
		CTimeString::changeParameter(parameter_vec, name_parameter_vec);

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

	//push_back parameter information to s_output_vecvec
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("Parameter");
		s_output_vecvec.push_back(s_temp_vec);
	}
	for (int i = 0; i < name_parameter_vec.size(); i++)
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back(name_parameter_vec[i]);
		s_temp_vec.push_back("");
		s_temp_vec.push_back("");
		s_temp_vec.push_back("");
		s_temp_vec.push_back(to_string(parameter_vec[i]));
		s_output_vecvec.push_back(s_temp_vec);
	}

	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("");
		s_output_vecvec.push_back(s_temp_vec);
	}
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("Result");
		s_output_vecvec.push_back(s_temp_vec);
	}

	GR_addToOutputString_OutputHeader_FPFH(s_output_vecvec);

	int i_tgt_start = 0;
	//cout << "select first tgt frame" << endl;
	//cout << "i_tgt_start ->";
	//cin >> i_tgt_start;

	if (!(0 <= i_tgt_start && i_tgt_start <= filenames_.size() - 2))
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

	for (int i = 0; i < filenames_.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
		cloud->is_dense = true;
		cloud_vec.push_back(cloud);
	}

	for (int i = 0; i < filenames_.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_vec[i]);
		sor->filter(*cloud_VGF);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_vec[i], radius_normal_FPFH, radius_FPFH);
		fpfh_vec.push_back(fpfh);
	}

	string time_end_FPFH = CTimeString::getTimeString();
	cout << "time_end_FPFH:" << time_end_FPFH << endl;

	vector<pair<int, int>> frame_pair_vec;
	frame_pair_vec = GR_FPFH_SAC_IA_get_frame_pair_vec(dir_);

	vector<pair<int, int>> frame_pair_est;

	for (int i_frame_pair = 0; i_frame_pair < frame_pair_vec.size(); i_frame_pair++)
	{
		int i_tgt = frame_pair_vec[i_frame_pair].first;
		int i_src = frame_pair_vec[i_frame_pair].second;
		cout << "i_tgt:" << i_tgt << " i_src:" << i_src << endl;

		vector<string> s_result_vec;
		s_result_vec = GR_FPFH_SAC_IA_Allframes_OnePair(dir_ + "/" + s_newfoldername, parameter_vec, i_tgt, i_src, cloud_vec, fpfh_vec,
			trajectory_vec, b_useRANSAC_EST, true);
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

		s_output_vecvec.push_back(s_result_vec);

		//regular saving csv
		string s_elapsed_frame = CTimeString::getTimeElapsefrom2Strings(time_regular, time_end_frame);
		cout << "time_elapsed from last .csv output: " << s_elapsed_frame << endl;
		cout << "time_elapsed from start:            " << CTimeString::getTimeElapsefrom2Strings(time_start, time_end_frame) << endl;
		int elapsed_millisec = CTimeString::getTimeElapsefrom2Strings_millisec(time_regular, time_end_frame);
		int elapsed_minute = (int)(((float)elapsed_millisec / 1000.) / 60.);
		if (elapsed_minute >= th_minute_CSV)
		{
			//save
			CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_regular + "_output.csv");
			time_regular = CTimeString::getTimeString();
			//clear s_output_vecvec
			s_output_vecvec.clear();
			GR_addToOutputString_OutputHeader_FPFH(s_output_vecvec);
		}
		cout << endl;

		if (i_frame_pair % 5 == 0 && !b_changeParameter)
		{
			cout << "Parameter list" << endl;
			CTimeString::showParameter(parameter_vec, name_parameter_vec);
			cout << endl;
		}
	}

	string time_end = CTimeString::getTimeString();
	string time_elapsed = CTimeString::getTimeElapsefrom2Strings(time_start, time_end);
	cout << "time_elapsed:" << time_elapsed << endl;

	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("");
		s_output_vecvec.push_back(s_temp_vec);
	}
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("Sum elapsed time");
		s_output_vecvec.push_back(s_temp_vec);
	}
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back(time_elapsed);
		s_output_vecvec.push_back(s_temp_vec);
	}

	CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_end + "_output.csv");

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

vector<string> CICPWithFeature::GR_FPFH_SAC_IA_Allframes_OnePair(string dir_, vector<float> parameter_vec, int i_tgt, int i_src,
	vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> cloud_vec, vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec,
	vector<Eigen::Vector6d> trajectory_vec, bool b_useRANSAC_EST, bool b_useOutputPointcloud)
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
		T_src_TRUE = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]).inverse()
			* calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);

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
		T_i_src = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
		T_i1_tgt = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
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

vector<pair<int, int>> CICPWithFeature::GR_FPFH_SAC_IA_get_frame_pair_vec(string dir_)
{
	vector<pair<int, int>> frame_pair_vec;

	vector<vector<bool>> b_ignore_vecvec;
	vector<vector<bool>> b_ahead_vecvec;

	//input from text
	vector<vector<string>> s_matrix_vecvec;
	{
		//input text
		vector<vector<string>> s_matrix_vecvec_temp;
		s_matrix_vecvec_temp = CTimeString::getVecVecFromCSV_string(dir_ + "/" + "matrix_ignore_ahead.csv");
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

void CICPWithFeature::GR_addToOutputString_OutputHeader_FPFH(vector<vector<string>> &s_output_vecvec)
{
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
	s_output_vecvec.push_back(s_temp_vec);
}

void CICPWithFeature::GR_FPFH_error(string dir_, vector<float> parameter_vec)
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	vector<string> name_parameter_vec;
	name_parameter_vec.push_back("voxel_size");
	name_parameter_vec.push_back("radius_normal_FPFH");
	name_parameter_vec.push_back("radius_FPFH");
	name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");
	name_parameter_vec.push_back("SimilarityThreshold_SAC");
	name_parameter_vec.push_back("InlierFraction_SAC");
	name_parameter_vec.push_back("MaximumIterations_SAC");
	name_parameter_vec.push_back("NumberOfSamples_SAC");
	name_parameter_vec.push_back("CorrespondenceRandomness_SAC");
	name_parameter_vec.push_back("max_RANSAC");

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	//0.15	0
	//0.1	29 / 50
	//0.125	1 / 50

	int i_tgt, i_src;

	pcl::PointCloud<T_PointType>::Ptr cloud_temp(new pcl::PointCloud<T_PointType>());

	//true trajectory
	vector<Eigen::Vector6d> trajectory_vec;
	{
		vector<string> filenames_trajectory_temp;
		vector<vector<double>> trajectory_vecvec_temp;
		CTimeString::getFileNames_extension(dir_, filenames_trajectory_temp, ".csv");
		if (filenames_trajectory_temp.size() != 1)
		{
			cout << "ERROR: true trajectory not found" << endl;
			return;
		}
		trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames_trajectory_temp[0]);
		for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
				trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
				trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
			trajectory_vec.push_back(Pos_temp);
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


	while (1)
	{
		cout << "select frame  (ESCAPE by typing same frame or -1 )" << endl;
		cout << "i_tgt ->";
		cin >> i_tgt;
		if (i_tgt == -1 || i_tgt > filenames_.size() - 1) break;
		cout << "i_src ->";
		cin >> i_src;
		if (i_src == -1 || i_src > filenames_.size() - 1) break;

		if (i_tgt == i_src) break;

		//change parameter
		CTimeString::changeParameter(parameter_vec, name_parameter_vec);

		//parameter
		float voxel_size;
		voxel_size = parameter_vec[0];

		float radius_normal_FPFH, radius_FPFH;
		radius_normal_FPFH = parameter_vec[1];
		radius_FPFH = parameter_vec[2];

		float MaxCorrespondenceDistance_SAC;
		MaxCorrespondenceDistance_SAC = parameter_vec[3];


		cout << "calculation start" << endl;

		//input pointcloud
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i_tgt], *cloud_tgt);
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i_src], *cloud_src);
		cloud_tgt->is_dense = true;
		cloud_src->is_dense = true;

		//compute fpfh
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(cloud_tgt);
			sor->filter(*cloud_VGF);
			fpfh_tgt = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_tgt, radius_normal_FPFH, radius_FPFH);
		}
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(cloud_src);
			sor->filter(*cloud_VGF);
			fpfh_src = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_src, radius_normal_FPFH, radius_FPFH);
		}

		//vector<int> inlier_;

		cout << "i_tgt:" << i_tgt << " i_src" << i_src << endl;
		//cout << "inlier_.size():" << inlier_.size() << endl;

		//show pointcloud
		pcl::PointCloud<T_PointType>::Ptr cloud_src_show(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt_show(new pcl::PointCloud<T_PointType>());
		//VGF
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_src);
		sor->filter(*cloud_src_show);
		sor->setInputCloud(cloud_tgt);
		sor->filter(*cloud_tgt_show);
		//color
		for (int i = 0; i < cloud_tgt_show->size(); i++)
		{
			cloud_tgt_show->points[i].r = 255;
			cloud_tgt_show->points[i].g = 255;
			cloud_tgt_show->points[i].b = 255;
		}
		for (int i = 0; i < cloud_src_show->size(); i++)
		{
			cloud_src_show->points[i].r = 255;
			cloud_src_show->points[i].g = 255;
			cloud_src_show->points[i].b = 255;
		}

		////add to viewer
		//CPointVisualization<T_PointType> pv_tgt;
		//pv_tgt.setWindowName("Target");
		//CPointVisualization<T_PointType> pv_src;
		//pv_src.setWindowName("Source");

		{
			Eigen::Matrix4d T_i_src = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d T_i1_tgt = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d T_i_GL = Eigen::Matrix4d::Identity();
			//T_i_src = T_i1_tgt * T_i_GL
			T_i_src = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
			T_i1_tgt = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
			T_i_GL = T_i1_tgt.inverse() * T_i_src;
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(T_i_GL);
			pcl::transformPointCloud(*cloud_src_show, *cloud_src_show, Trans_temp);
		}

		//corr
		pcl::Correspondences correspondences;
		{
			correspondences.resize(cloud_src_show->size());
			std::vector<int> index(1);
			std::vector<float> distance(1);
			unsigned int nr_valid_correspondences = 0;
			pcl::KdTreeFLANN<T_PointType> match_search;
			match_search.setInputCloud(cloud_tgt_show);
			for (size_t i = 0; i < cloud_src_show->size(); ++i)
			{
				int found_neighs = match_search.nearestKSearch(cloud_src_show->at(i), 1, index, distance);
				if (distance[0] > MaxCorrespondenceDistance_SAC * MaxCorrespondenceDistance_SAC) continue;
				pcl::Correspondence corr;
				corr.index_query = i;
				corr.index_match = index[0];
				corr.distance = distance[0];	//squared
				correspondences[nr_valid_correspondences++] = corr;
			}
			correspondences.resize(nr_valid_correspondences);
			//cout << "correspondences size = " << nr_valid_correspondences << endl;
		}
		//calc error
		vector<float> error_fpfh_vec;
		float median_;
		error_fpfh_vec = CFPFH_PCL::getErrorOfFPFHSource_corr(median_, correspondences, fpfh_src, fpfh_tgt);

		////calc variance
		//vector<float> variance_vec;
		//variance_vec = CExtendableICP::getFPFHVariance(fpfh_src);
		//cout << "calc variance" << endl;
		//for (int i = 0; i < variance_vec.size(); i++)
		//	cout << "i:" << i << " " << variance_vec[i] << endl;

		//change color
		for (int i = 0; i < correspondences.size(); i++)
		{
			int i_src_temp, i_tgt_temp;
			int i_color = 0;
			i_src_temp = correspondences[i].index_query;
			i_tgt_temp = correspondences[i].index_match;
			if (error_fpfh_vec[i_src_temp] <= median_) i_color = 1;
			else if (error_fpfh_vec[i_src_temp] > median_) i_color = 2;

			if (i_color == 1)
			{
				//src
				cloud_src_show->points[i_src_temp].r = 0;
				cloud_src_show->points[i_src_temp].g = 255;
				cloud_src_show->points[i_src_temp].b = 0;
				//tgt
				cloud_tgt_show->points[i_tgt_temp].r = 0;
				cloud_tgt_show->points[i_tgt_temp].g = 255;
				cloud_tgt_show->points[i_tgt_temp].b = 0;
			}
			else if (i_color == 2)
			{
				//src
				cloud_src_show->points[i_src_temp].r = 255;
				cloud_src_show->points[i_src_temp].g = 0;
				cloud_src_show->points[i_src_temp].b = 0;
				//tgt
				cloud_tgt_show->points[i_tgt_temp].r = 255;
				cloud_tgt_show->points[i_tgt_temp].g = 0;
				cloud_tgt_show->points[i_tgt_temp].b = 0;
			}

		}

		//pv_tgt.setPointCloud(cloud_tgt_show);
		//pv_src.setPointCloud(cloud_src_show);

		//while (1)
		//{
		//	pv_src.updateViewer();
		//	pv_tgt.updateViewer();
		//	if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
		//}
		////remove viewer
		//pv_src.closeViewer();
		//pv_tgt.closeViewer();

		cout << endl;
	}
	cout << "escaped" << endl;
}

void CICPWithFeature::GR_FPFH_error_AllFrames(string dir_, vector<float> parameter_vec, bool b_changeParameter)
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	vector<string> name_parameter_vec;
	name_parameter_vec.push_back("voxel_size");
	name_parameter_vec.push_back("radius_normal_FPFH");
	name_parameter_vec.push_back("radius_FPFH");
	name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");
	name_parameter_vec.push_back("SimilarityThreshold_SAC");
	name_parameter_vec.push_back("InlierFraction_SAC");
	name_parameter_vec.push_back("MaximumIterations_SAC");
	name_parameter_vec.push_back("NumberOfSamples_SAC");
	name_parameter_vec.push_back("CorrespondenceRandomness_SAC");
	name_parameter_vec.push_back("max_RANSAC");

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	pcl::PointCloud<T_PointType>::Ptr cloud_temp(new pcl::PointCloud<T_PointType>());

	//int i_tgt, i_src;
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;

	if (b_changeParameter)
		CTimeString::changeParameter(parameter_vec, name_parameter_vec);

	//parameter
	float voxel_size;
	voxel_size = parameter_vec[0];

	float radius_normal_FPFH, radius_FPFH;
	radius_normal_FPFH = parameter_vec[1];
	radius_FPFH = parameter_vec[2];

	float MaxCorrespondenceDistance_SAC;
	MaxCorrespondenceDistance_SAC = parameter_vec[3];

	for (int i = 0; i < filenames_.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
		cloud->is_dense = true;
		cloud_vec.push_back(cloud);
	}

	for (int i = 0; i < filenames_.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_vec[i]);
		sor->filter(*cloud_VGF);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_vec[i], radius_normal_FPFH, radius_FPFH);
		fpfh_vec.push_back(fpfh);
	}

	vector<double> result_vec;


	//true trajectory
	vector<Eigen::Vector6d> trajectory_vec;
	{
		//vector<string> filenames_trajectory_temp;
		//vector<vector<double>> trajectory_vecvec_temp;
		//CTimeString::getFileNames_extension(dir_, filenames_trajectory_temp, ".csv");
		//if (filenames_trajectory_temp.size() != 1)
		//{
		//	cout << "ERROR: true trajectory not found" << endl;
		//	return;
		//}
		//trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames_trajectory_temp[0]);
		//for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		//{
		//	Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
		//	Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
		//		trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
		//		trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
		//	trajectory_vec.push_back(Pos_temp);
		//}

		string filename_true = "transformation_fin.csv";
		vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
		for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
				trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
				trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
			trajectory_vec.push_back(Pos_temp);
		}
	}

	cout << "calculation start" << endl;

	int i_tgt_start = 0;
	for (int i_tgt = i_tgt_start; i_tgt < cloud_vec.size() - 1; i_tgt++)
	{
		for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
			pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
			cloud_tgt->clear();
			cloud_src->clear();
			{
				const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
				sor->setLeafSize(voxel_size, voxel_size, voxel_size);
				sor->setInputCloud(cloud_vec[i_tgt]);
				sor->filter(*cloud_tgt);
			}
			{
				const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
				sor->setLeafSize(voxel_size, voxel_size, voxel_size);
				sor->setInputCloud(cloud_vec[i_src]);
				sor->filter(*cloud_src);
			}

			{
				Eigen::Matrix4d T_i_src = Eigen::Matrix4d::Identity();
				Eigen::Matrix4d T_i1_tgt = Eigen::Matrix4d::Identity();
				Eigen::Matrix4d T_i_GL = Eigen::Matrix4d::Identity();
				//T_i_src = T_i1_tgt * T_i_GL
				T_i_src = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
				T_i1_tgt = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
				T_i_GL = T_i1_tgt.inverse() * T_i_src;
				Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
				Trans_temp = calcAffine3fFromHomogeneousMatrix(T_i_GL);
				pcl::transformPointCloud(*cloud_src, *cloud_src, Trans_temp);
			}

			//corr
			pcl::Correspondences correspondences;
			{
				correspondences.resize(cloud_src->size());
				std::vector<int> index(1);
				std::vector<float> distance(1);
				unsigned int nr_valid_correspondences = 0;
				pcl::KdTreeFLANN<T_PointType> match_search;
				match_search.setInputCloud(cloud_tgt);
				for (size_t i = 0; i < cloud_src->size(); ++i)
				{
					int found_neighs = match_search.nearestKSearch(cloud_src->at(i), 1, index, distance);
					if (distance[0] > MaxCorrespondenceDistance_SAC * MaxCorrespondenceDistance_SAC) continue;
					pcl::Correspondence corr;
					corr.index_query = i;
					corr.index_match = index[0];
					corr.distance = distance[0];	//squared
					correspondences[nr_valid_correspondences++] = corr;
				}
				correspondences.resize(nr_valid_correspondences);
			}
			//cacl error
			vector<float> error_fpfh_vec;
			float median_;
			error_fpfh_vec = CFPFH_PCL::getErrorOfFPFHSource_corr(median_, correspondences, fpfh_vec[i_src], fpfh_vec[i_tgt]);
			result_vec.push_back((double)median_);
			//if (b_changeParameter) cout << "i_tgt:" << i_tgt << " i_src:" << i_src << endl;
			cout << "i_tgt:" << i_tgt << " i_src:" << i_src;
			cout << "  median_:" << median_;
			cout << "  inlier_rate:" << (float)correspondences.size() / (float)fpfh_vec[i_src]->size();
			cout << endl;
		}
	}

	cout << endl;
	//show parameter
	cout << "output" << endl;
	cout << "Parameter list" << endl;
	CTimeString::showParameter(parameter_vec, name_parameter_vec, 3);
	//cout << "0: voxel_size:                     " << voxel_size << endl;
	//cout << "1: radius_normal_FPFH:             " << radius_normal_FPFH << endl;
	//cout << "2: radius_FPFH:                    " << radius_FPFH << endl;
	//cout << "3: MaxCorrespondenceDistance_SAC:  " << MaxCorrespondenceDistance_SAC << endl;

	//result mean and median
	double mean_result = 0.;
	for (int i = 0; i < result_vec.size(); i++)
		mean_result += result_vec[i];
	mean_result /= (float)result_vec.size();
	cout << "mean_result:" << mean_result << endl;
	double median_result;
	sort(result_vec.begin(), result_vec.end());
	if (result_vec.size() % 2 == 1) median_result = result_vec[(result_vec.size() - 1) / 2];
	else median_result = (result_vec[result_vec.size() / 2 - 1] + result_vec[result_vec.size() / 2]) / 2.;
	cout << "median_result:" << median_result << endl;

	cout << endl;
}

void CICPWithFeature::GR_FPFH_variance_AllFrames(string dir_, vector<float> parameter_vec, bool b_changeParameter)
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	vector<string> name_parameter_vec;
	name_parameter_vec.push_back("voxel_size");
	name_parameter_vec.push_back("radius_normal_FPFH");
	name_parameter_vec.push_back("radius_FPFH");
	name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");
	name_parameter_vec.push_back("SimilarityThreshold_SAC");
	name_parameter_vec.push_back("InlierFraction_SAC");
	name_parameter_vec.push_back("MaximumIterations_SAC");
	name_parameter_vec.push_back("NumberOfSamples_SAC");
	name_parameter_vec.push_back("CorrespondenceRandomness_SAC");
	name_parameter_vec.push_back("max_RANSAC");

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	if (b_changeParameter)
	{
		//CTimeString::changeParameter(parameter_vec, name_parameter_vec);
		CTimeString::changeParameter(parameter_vec, name_parameter_vec, dir_ + "/" + "parameter_vec.csv", 1, 10, 4);
	}

	//parameter
	float voxel_size;
	voxel_size = parameter_vec[0];

	float radius_normal_FPFH, radius_FPFH;
	radius_normal_FPFH = parameter_vec[1];
	radius_FPFH = parameter_vec[2];

	float MaxCorrespondenceDistance_SAC;
	MaxCorrespondenceDistance_SAC = parameter_vec[3];

	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	for (int i = 0; i < filenames_.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
		cloud->is_dense = true;
		cloud_vec.push_back(cloud);
	}

	vector<vector<float>> variance_vec_vec;

	for (int j = 0; j < filenames_.size(); j++)
	{
		//if (b_changeParameter) cout << "frame:" << j << endl;
		cout << "frame:" << j << endl;
		pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
		{
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(cloud_vec[j]);
			sor->filter(*cloud_VGF);
		}

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh_ = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_vec[j], radius_normal_FPFH, radius_FPFH);

		//calc variance
		vector<float> variance_vec;
		variance_vec = CFPFH_PCL::getFPFHVariance(fpfh_);
		variance_vec_vec.push_back(variance_vec);
	}

	vector<float> variance_vec_allFrame;
	variance_vec_allFrame.resize(variance_vec_vec[0].size());
	fill(variance_vec_allFrame.begin(), variance_vec_allFrame.end(), 0.);
	for (int j = 0; j < variance_vec_vec.size(); j++)
	{
		for (int i = 0; i < variance_vec_vec[j].size(); i++)
			variance_vec_allFrame[i] += variance_vec_vec[j][i];
	}

	for (int i = 0; i < variance_vec_allFrame.size(); i++)
		variance_vec_allFrame[i] /= (float)variance_vec_vec.size();

	//cout << endl;
	//for (int i = 0; i < variance_vec_allFrame.size(); i++)
	//	cout << "i:" << i << " " << variance_vec_allFrame[i] << endl;
	//cout << endl;

	cout << "for compare" << endl;
	CTimeString::showParameter(parameter_vec, name_parameter_vec, 2);
	cout << endl;
	cout << "show FPFH variance by histogram" << endl;
	for (int i = 0; i < variance_vec_allFrame.size(); i++)
		cout << variance_vec_allFrame[i] << endl;

	cout << endl;
}

void CICPWithFeature::GR_FPFH_varyParameter(string dir_, vector<float> parameter_vec_arg)
{
	vector<string> name_parameter_vec;
	name_parameter_vec.push_back("voxel_size");
	name_parameter_vec.push_back("radius_normal_FPFH");
	name_parameter_vec.push_back("radius_FPFH");
	name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");
	name_parameter_vec.push_back("SimilarityThreshold_SAC");
	name_parameter_vec.push_back("InlierFraction_SAC");
	name_parameter_vec.push_back("MaximumIterations_SAC");
	name_parameter_vec.push_back("NumberOfSamples_SAC");
	name_parameter_vec.push_back("CorrespondenceRandomness_SAC");
	name_parameter_vec.push_back("max_RANSAC");

	cout << "0: registration of all frames and output files(.csv and .pcd)" << endl;
	cout << "1: output error of fpfh value (all frames)" << endl;
	cout << "2: output show FPFH variance (all frames)" << endl;

	int i_method;
	cin >> i_method;

	bool b_create_new_pattern_file = false;
	cout << "do you create new pattern?  Yes:1  No:0" << endl;
	cout << "->";
	cin >> b_create_new_pattern_file;

	if (b_create_new_pattern_file)
	{
		vector<vector<float>> pattern_vec_vec_new;
		//input parameter
		vector<vector<float>> parameter_vec_vec;
		//CTimeString::changeParameter_2dimension(parameter_vec_vec, name_parameter_vec, parameter_vec_arg);
		CTimeString::changeParameter_2dimension(parameter_vec_vec, name_parameter_vec, parameter_vec_arg,
			dir_ + "/" + "parameter_vecvec.csv", 1, 4, -1, -1);
		pattern_vec_vec_new = CTimeString::calcVectorPairPattern(parameter_vec_vec);
		//write new parameter_vec_vec
		{
			vector<vector<string>> s_vec_vec;
			//header
			{
				vector<string> s_header_vec;
				//s_header_vec.push_back("Parameter");
				for (int i = 0; i < name_parameter_vec.size(); i++)
					s_header_vec.push_back(name_parameter_vec[i]);
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
		switch (i_method)
		{
		case 0:
			CTimeString::showParameter(parameter_vec, name_parameter_vec);
			GR_FPFH_SAC_IA_Allframes(dir_, parameter_vec, false);
			break;
		case 1:
			CTimeString::showParameter(parameter_vec, name_parameter_vec, 3);
			GR_FPFH_error_AllFrames(dir_, parameter_vec, false);
			break;
		case 2:
			CTimeString::showParameter(parameter_vec, name_parameter_vec);
			GR_FPFH_variance_AllFrames(dir_, parameter_vec, false);
			break;
		default:
			break;
		}
	}

	cout << endl;
}

void CICPWithFeature::GR_FPFH_FixFusion(string dir_, string s_folder)
{ //search folder in out of funtion

	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	double th_distance_ = 2.3;
	int num_distance_ = 10;
	bool b_useDistanceIteration = false;
	b_useDistanceIteration = true;

	vector<string> filenames_csv;
	CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_csv, "_fusion.csv");
	if (filenames_csv.size() > 1)
	{
		cout << "ERROR: too many fusion.csv found" << endl;
		return;
	}
	else if (filenames_csv.size() == 0)
	{
		cout << "ERROR: no fusion.csv found" << endl;
		return;
	}

	//input csv
	string filename_csv_old = filenames_csv[0];
	vector<vector<string>> s_input_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder + "/" + filename_csv_old);

	//input parameter
	vector<float> parameter_vec;
	{
		vector<vector<string>> s_temp_vecvec;
		for (int j = 1; j < 11; j++)
			s_temp_vecvec.push_back(s_input_vecvec[j]);
		for (int j = 0; j < s_temp_vecvec.size(); j++)
			parameter_vec.push_back(stof(s_temp_vecvec[j][4]));
	}
	float voxel_size;
	voxel_size = parameter_vec[0];

	float radius_normal_FPFH, radius_FPFH;
	radius_normal_FPFH = parameter_vec[1];
	radius_FPFH = parameter_vec[2];

	//true trajectory
	vector<Eigen::Vector6d> trajectory_vec;
	{
		string filename_true = "transformation_fin.csv";
		vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
		for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
				trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
				trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
			trajectory_vec.push_back(Pos_temp);
		}
	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");
		for (int i = 0; i < filenames_.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	//calc FPFH
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	for (int i = 0; i < cloud_vec.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_vec[i]);
		sor->filter(*cloud_VGF);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_vec[i], radius_normal_FPFH, radius_FPFH);
		fpfh_vec.push_back(fpfh);
	}

	bool b_useRANSAC_EST = false;
	b_useRANSAC_EST = true;

	while (1)
	{
		//extract
		vector<pair<int, int>> pair_vec;
		{
			vector<vector<string>> s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
				s_input_vecvec, "Result", 2, "Sum elapsed time", -2);
			for (int j = 0; j < s_temp_vecvec.size(); j++)
			{
				int i_tgt = stoi(s_temp_vecvec[j][0]);
				int i_src = stoi(s_temp_vecvec[j][1]);
				double distance_ = stod(s_temp_vecvec[j][10]);
				pair_vec.push_back(make_pair(i_tgt, i_src));
				string s_j = to_string(j);
				if (s_j.size() < 2) s_j = " " + s_j;
				string s_i_tgt = to_string(i_tgt);
				if (s_i_tgt.size() < 2) s_i_tgt = " " + s_i_tgt;
				string s_i_src = to_string(i_src);
				if (s_i_src.size() < 2) s_i_src = " " + s_i_src;
				cout << "j:" << s_j << " i_tgt:" << s_i_tgt << " i_src:" << s_i_src << "  distance_:" << distance_ << endl;
			}
		}

		//select pair
		int i_pair_selected;
		cout << "select pair (finish by -1):";
		cin >> i_pair_selected;

		if (i_pair_selected == -1) break;

		//int num_iteration = 0;
		//cout << "input: num_iterate ->";
		//cin >> num_iteration;

		//calc
		{
			int i_tgt = pair_vec[i_pair_selected].first;
			int i_src = pair_vec[i_pair_selected].second;
			cout << "i_tgt:" << i_tgt << " i_src:" << i_src << endl;

			vector<string> s_result_vec;
			//for (int i = 0; i < num_iteration; i++)
			//	s_result_vec = GR_FPFH_SAC_IA_Allframes_OnePair("", parameter_vec, i_tgt, i_src, cloud_vec, fpfh_vec,
			//		trajectory_vec, b_useRANSAC_EST, false);
			//for(int i=0;i<)
			if (b_useDistanceIteration)
			{
				cout << "iteration(distance)" << endl;
				for (int i = 0; i < num_distance_; i++)
				{
					cout << "i:" << i << endl;
					s_result_vec = GR_FPFH_SAC_IA_Allframes_OnePair("", parameter_vec, i_tgt, i_src, cloud_vec, fpfh_vec,
						trajectory_vec, b_useRANSAC_EST, false);
					if (stod(s_result_vec[10]) < th_distance_) break;
				}
			}
			else
			{
				s_result_vec = GR_FPFH_SAC_IA_Allframes_OnePair("", parameter_vec, i_tgt, i_src, cloud_vec, fpfh_vec,
					trajectory_vec, b_useRANSAC_EST, false);
			}

			bool b_hasConverged = false;
			if (stoi(s_result_vec[8]) == 1)
				b_hasConverged = true;

			cout << "Estimation: ";
			if (b_useRANSAC_EST && b_hasConverged)
				cout << "Success" << endl;
			else
				cout << "not Success" << endl;

			int i_distace = 14;
			s_input_vecvec[i_pair_selected + i_distace] = s_result_vec;

		}
		cout << endl;
	}

	//save file
	bool b_saveFile = false;
	cout << "select: saveFile:1  not:0" << endl;
	cout << "->";
	cin >> b_saveFile;
	if (!b_saveFile) return;

	//move csv
	{
		vector<string> filenames_folder;
		CTimeString::getFileNames_folder(dir_ + "/" + s_folder, filenames_folder);
		string foldername_old = "_old";
		if (filenames_folder.size() == 0)
			CTimeString::makenewfolder(dir_ + "/" + s_folder, foldername_old);
		string filename_csv_old_new = CTimeString::getTimeString() + "_from" + filename_csv_old;
		CTimeString::movefile(dir_ + "/" + s_folder + "/" + filename_csv_old, dir_ + "/" + s_folder + "/" + foldername_old + "/" + filename_csv_old_new);
	}

	//save
	CTimeString::getCSVFromVecVec(s_input_vecvec, dir_ + "/" + s_folder + "/" + filename_csv_old);
}

void CICPWithFeature::DoICP_addToOutputString_OutputHeader(vector<vector<string>> &s_output_vecvec)
{
	vector<string> s_temp_vec;
	s_temp_vec.push_back("target");
	s_temp_vec.push_back("source");
	s_temp_vec.push_back("inlier rate");
	s_temp_vec.push_back("distance");
	s_temp_vec.push_back("median");
	s_temp_vec.push_back("distance_Pos");
	s_temp_vec.push_back("fitness");
	s_temp_vec.push_back("time");
	s_temp_vec.push_back("X");
	s_temp_vec.push_back("Y");
	s_temp_vec.push_back("Z");
	s_temp_vec.push_back("ROLL");
	s_temp_vec.push_back("PITCH");
	s_temp_vec.push_back("YAW");
	s_output_vecvec.push_back(s_temp_vec);
}

void CICPWithFeature::DoICP_proposed_AllFrames(string dir_)
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	int i_method;
	cout << "select: single parameter:0  vary parameters:1  merge results:2" << endl;
	cout << "->";
	cin >> i_method;

	vector<string> filenames_input;
	if (i_method == 0 || i_method == 1)
	{
		vector<string> filenames_temp;
		CTimeString::getFileNames_extension(dir_, filenames_temp, "_SucEst.csv");
		for (int i = 0; i < filenames_temp.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_temp[i] << endl;
		}
		cout << endl;
		cout << "input .csv you want to calc (can input multinumber)" << endl;
		vector<string> s_input_vec;
		bool b_useCSV = false;
		{
			cout << "select: input number by csv or not  yes:1  no:0" << endl;
			cout << "->";
			cin >> b_useCSV;
		}
		if (!b_useCSV) s_input_vec = CTimeString::inputSomeString();
		else s_input_vec = CTimeString::inputSomeString_fromCSV(dir_ + "/" + "num_vector.csv");
		cout << "s_input_vec.size():" << s_input_vec.size() << endl;
		for (int i = 0; i < s_input_vec.size(); i++)
			filenames_input.push_back(filenames_temp[stoi(s_input_vec[i])]);
	}

	if (i_method == 0)
	{
		for (int j = 0; j < filenames_input.size(); j++)
		{
			string filename_csv_input;
			filename_csv_input = filenames_input[j];
			vector<float> parameter_vec;
			{
				int MaximumIterations;
				MaximumIterations = 50000;
				double MaxCorrespondenceDistance, EuclideanFitnessEpsilon, TransformationEpsilon;
				//MaxCorrespondenceDistance = 1.;
				MaxCorrespondenceDistance = 0.25;	//SII2021
				EuclideanFitnessEpsilon = 1e-5;
				TransformationEpsilon = 1e-6;
				//property
				double penalty_chara, weight_dist_chara;
				penalty_chara = 1.;
				weight_dist_chara = 2.;				//SII2021
				//vector
				parameter_vec.push_back(MaximumIterations);
				parameter_vec.push_back(MaxCorrespondenceDistance);
				parameter_vec.push_back(EuclideanFitnessEpsilon);
				parameter_vec.push_back(TransformationEpsilon);
				parameter_vec.push_back(penalty_chara);
				parameter_vec.push_back(weight_dist_chara);
			}
			DoICP_proposed_givenParameter(dir_, filename_csv_input, parameter_vec);
		}
	}

	else if (i_method == 1)
	{
		for (int j = 0; j < filenames_input.size(); j++)
		{
			string filename_csv_input;
			filename_csv_input = filenames_input[j];
			DoICP_proposed_varyParameters(dir_, filename_csv_input);
		}
	}

	else if (i_method == 2)
		DoICP_proposed_mergeResult();
}

void CICPWithFeature::DoICP_proposed_givenParameter(string dir_, string filename_csv, vector<float> parameter_vec)
{
	vector<vector<string>> s_output_vecvec;
	s_output_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + filename_csv);

	vector<string> name_parameter_vec;
	//name_parameter_vec.push_back("i_method_");
	name_parameter_vec.push_back("MaximumIterations");
	name_parameter_vec.push_back("MaxCorrespondenceDistance");
	name_parameter_vec.push_back("EuclideanFitnessEpsilon");
	name_parameter_vec.push_back("TransformationEpsilon");
	name_parameter_vec.push_back("penalty_chara");
	name_parameter_vec.push_back("weight_dist_chara");

	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("");
		s_output_vecvec.push_back(s_temp_vec);
	}
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("Input: " + filename_csv);
		s_output_vecvec.push_back(s_temp_vec);
	}
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("");
		s_output_vecvec.push_back(s_temp_vec);
	}

	//push_back parameter information to s_output_vecvec
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("Parameter_ICP");
		s_output_vecvec.push_back(s_temp_vec);
	}
	for (int i = 0; i < name_parameter_vec.size(); i++)
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back(name_parameter_vec[i]);
		s_temp_vec.push_back("");
		s_temp_vec.push_back("");
		s_temp_vec.push_back("");
		s_temp_vec.push_back(to_string(parameter_vec[i]));
		s_output_vecvec.push_back(s_temp_vec);
	}

	string time_start = CTimeString::getTimeString();
	//string time_regular = time_start;
	cout << "time_start:" << time_start << endl;
	//make new folder
	string s_newfoldername = time_start;
	CTimeString::makenewfolder(dir_, s_newfoldername);

	//ICP
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("");
		s_output_vecvec.push_back(s_temp_vec);
	}
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("Result_ICP");
		s_output_vecvec.push_back(s_temp_vec);
	}
	DoICP_addToOutputString_OutputHeader(s_output_vecvec);
	DoICP_proposed_only1method(dir_, s_newfoldername, s_output_vecvec, parameter_vec, 0);

	//ICP_proposed
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("");
		s_output_vecvec.push_back(s_temp_vec);
	}
	{
		vector<string> s_temp_vec;
		s_temp_vec.push_back("Result_ICP_proposed");
		s_output_vecvec.push_back(s_temp_vec);
	}
	DoICP_addToOutputString_OutputHeader(s_output_vecvec);
	DoICP_proposed_only1method(dir_, s_newfoldername, s_output_vecvec, parameter_vec, 1);

	string time_end = CTimeString::getTimeString();
	string time_elapsed = CTimeString::getTimeElapsefrom2Strings(time_start, time_end);
	cout << "time_elapsed:" << time_elapsed << endl;

	CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_start + "_output_ICP.csv");
	cout << endl;
}

void CICPWithFeature::DoICP_proposed_only1method(
	string dir_, string s_folder, vector<vector<string>> &s_input_vecvec, vector<float> parameter_vec, int i_method)
{
	bool b_calcOnlyBiggestCluster = false;
	b_calcOnlyBiggestCluster = true;
	vector<int> frames_all;

	bool b_skip_artifitial = false;
	//b_skip_artifitial = true;

	//parameter
	//icp
	int MaximumIterations;
	float MaxCorrespondenceDistance, EuclideanFitnessEpsilon, TransformationEpsilon;
	MaximumIterations = (int)parameter_vec[0];
	MaxCorrespondenceDistance = parameter_vec[1];
	EuclideanFitnessEpsilon = parameter_vec[2];
	TransformationEpsilon = parameter_vec[3];
	//property
	float penalty_chara, weight_dist_chara;
	penalty_chara = parameter_vec[4];
	weight_dist_chara = parameter_vec[5];

	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	struct SInitPos
	{
		int i_tgt;
		int i_src;
		Eigen::Vector6d Init_Vector;
		SInitPos() { Init_Vector = Eigen::Vector6d::Zero(); }
	};

	//input initPos
	vector<SInitPos> initPos_vec;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(s_input_vecvec, "0", 0, "Sum elapsed time", -2);
		for (int j = 0; j < s_temp_vecvec.size(); j++)
		{
			SInitPos initPos_;
			//14 15 16 17 18 19  20
			if (1 == stoi(s_temp_vecvec[j][20]))
			{
				int i_tgt = stoi(s_temp_vecvec[j][0]);
				int i_src = stoi(s_temp_vecvec[j][1]);

				if (b_skip_artifitial)
				{
					//skip(compare conventional with proposed)
					if (i_tgt == 0 && i_src == 14) continue;
					if (i_tgt == 10 && i_src == 12) continue;
					if (i_tgt == 11 && i_src == 13) continue;
					//skip(for clear looking)
					if (i_tgt == 1 && i_src == 15) continue;
					if (i_tgt == 2 && i_src == 15) continue;
				}

				initPos_.i_tgt = i_tgt;
				initPos_.i_src = i_src;
				initPos_.Init_Vector <<
					stod(s_temp_vecvec[j][14])
					, stod(s_temp_vecvec[j][15])
					, stod(s_temp_vecvec[j][16])
					, stod(s_temp_vecvec[j][17])
					, stod(s_temp_vecvec[j][18])
					, stod(s_temp_vecvec[j][19]);
				initPos_vec.push_back(initPos_);
			}
		}
	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");
		for (int i = 0; i < filenames_.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}
	for (int j = 0; j < cloud_vec.size(); j++)
		frames_all.push_back(j);

	if (b_calcOnlyBiggestCluster)
	{
		frames_all.clear();
		//calc frames that should be deleted
		{
			vector<vector<int>> value_vecvec;

			for (int j = 0; j < initPos_vec.size(); j++)
			{
				vector<int> value_vec;
				value_vec.push_back(initPos_vec[j].i_tgt);
				value_vec.push_back(initPos_vec[j].i_src);
				value_vecvec.push_back(value_vec);
			}
			for (int j = 0; j < value_vecvec.size(); j++)
			{
				cout << "j:" << j;
				for (int i = 0; i < value_vecvec[j].size(); i++)
					cout << " " << value_vecvec[j][i];
				cout << endl;
			}
			cout << endl;

			vector<vector<int>> value_vecvec_new;
			value_vecvec_new = CTimeString::getIntCluster_SomeToSome(value_vecvec);
			cout << "finished" << endl;
			for (int j = 0; j < value_vecvec_new.size(); j++)
			{
				cout << "j:" << j;
				for (int i = 0; i < value_vecvec_new[j].size(); i++)
					cout << " " << value_vecvec_new[j][i];
				cout << endl;
			}
			cout << endl;

			//not contained to bigger cluster
			vector<bool> b_frames_isInBiggerCluster;
			b_frames_isInBiggerCluster.resize(cloud_vec.size());
			fill(b_frames_isInBiggerCluster.begin(), b_frames_isInBiggerCluster.end(), false);
			for (int i = 0; i < value_vecvec_new[0].size(); i++)
				b_frames_isInBiggerCluster[value_vecvec_new[0][i]] = true;

			int index_new = 0;
			for (int i = 0; i < cloud_vec.size(); i++)
			{
				if (b_frames_isInBiggerCluster[i])
				{
					frames_all.push_back(index_new);
					index_new++;
				}
				else frames_all.push_back(-1);
			}

			cout << "show deleted frames and others" << endl;
			for (int i = 0; i < frames_all.size(); i++)
				cout << "i:" << i << " " << frames_all[i] << endl;
		}
	}

	//input character
	vector<vector<int>> chara_vecvec;
	{
		for (int i = 0; i < cloud_vec.size(); i++)
		{
			vector<int> chara_vec;
			chara_vec = CExtendableICP::ICP_Chara_GetCharaData(cloud_vec[i]);
			chara_vecvec.push_back(chara_vec);
		}
	}

	//true trajectory
	vector<Eigen::Vector6d> trajectory_vec;
	{
		string filename_true = "transformation_fin.csv";
		vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
		for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
				trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
				trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
			trajectory_vec.push_back(Pos_temp);
		}
	}

	vector<vector<string>> s_output_vecvec;
	for (int j = 0; j < initPos_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_temp(new pcl::PointCloud<T_PointType>());

		int i_tgt = initPos_vec[j].i_tgt;
		int i_src = initPos_vec[j].i_src;

		if (frames_all[i_tgt] == -1) continue;
		if (frames_all[i_src] == -1) continue;

		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src_transformed(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);
		pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);

		//transform src by InitPos
		{
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(initPos_vec[j].Init_Vector));
			pcl::transformPointCloud(*cloud_src, *cloud_src_transformed, Trans_temp);
		}

		string time_start_frame = CTimeString::getTimeString();

		bool b_hasConverged = false;
		//vector<int> inlier_;
		float fitnessscore;
		//int frame_failed = 0;
		//Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
		//bool b_cout_RANSAC = false;
		Eigen::Vector6d Registration_Vector = Eigen::Vector6d::Zero();

		cout << "i_tgt:" << i_tgt << " i_src:" << i_src << endl;

		if (i_method == 0)
		{
			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> align_ICP;
			//parameter
			align_ICP.setMaximumIterations(MaximumIterations);
			align_ICP.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
			align_ICP.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
			align_ICP.setTransformationEpsilon(TransformationEpsilon);
			//data
			align_ICP.setInputTarget(cloud_tgt);
			align_ICP.setInputSource(cloud_src_transformed);
			//align
			align_ICP.align(*cloud_temp);
			b_hasConverged = align_ICP.hasConverged();
			Registration_Vector = calcVector6dFromHomogeneousMatrix(
				align_ICP.getFinalTransformation().cast<double>());
			fitnessscore = align_ICP.getFitnessScore();
		}
		else if (i_method == 1)
		{
			CExtendableICP align_ICP_proposed;
			//parameter
			align_ICP_proposed.setMothodInt(i_method);
			align_ICP_proposed.setMaximumIterations(MaximumIterations);
			align_ICP_proposed.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
			align_ICP_proposed.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
			align_ICP_proposed.setTransformationEpsilon(TransformationEpsilon);
			align_ICP_proposed.setCharaParameter(penalty_chara, weight_dist_chara);
			//data
			align_ICP_proposed.setInputTarget(cloud_tgt);
			align_ICP_proposed.setInputSource(cloud_src_transformed);
			align_ICP_proposed.setCharaVector_tgt(chara_vecvec[i_tgt]);
			align_ICP_proposed.setCharaVector_src(chara_vecvec[i_src]);
			//align
			align_ICP_proposed.align();
			b_hasConverged = align_ICP_proposed.hasConverged();
			Registration_Vector = align_ICP_proposed.getFinalTransformation_Vec();
			fitnessscore = align_ICP_proposed.getFitnessScore();
		}
		cout << "b_hasConverged:" << b_hasConverged << endl;

		//color (src, whiten)
		{
			int i_add = 100;
			int i_subt = 100;
			for (int i = 0; i < cloud_src_transformed->size(); i++)
			{
				if (cloud_src_transformed->points[i].r + i_add < 255)
					cloud_src_transformed->points[i].r += i_add;
				else cloud_src_transformed->points[i].r = 255;
				if (cloud_src_transformed->points[i].g + i_add < 255)
					cloud_src_transformed->points[i].g += i_add;
				else cloud_src_transformed->points[i].g = 255;
				if (cloud_src_transformed->points[i].b + i_add < 255)
					cloud_src_transformed->points[i].b += i_add;
				else cloud_src_transformed->points[i].b = 255;
				//if (cloud_src_transformed->points[i].r - i_subt > 0)
				//	cloud_src_transformed->points[i].r -= i_subt;
				//else cloud_src_transformed->points[i].r = 0;
				//if (cloud_src_transformed->points[i].g - i_subt > 0)
				//	cloud_src_transformed->points[i].g -= i_subt;
				//else cloud_src_transformed->points[i].g = 0;
				//if (cloud_src_transformed->points[i].b - i_subt > 0)
				//	cloud_src_transformed->points[i].b -= i_subt;
				//else cloud_src_transformed->points[i].b = 0;
			}
		}

		//transformation
		{
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(Registration_Vector));
			pcl::transformPointCloud(*cloud_src_transformed, *cloud_src_transformed, Trans_temp);
		}

		//inlier rate
		double rate_inlier;
		if (cloud_src->size() != 0)
		{
			pcl::Correspondences corr;
			corr = determineCorrespondences_output(cloud_src_transformed, cloud_tgt, MaxCorrespondenceDistance);
			rate_inlier = (float)corr.size() / (float)(cloud_src->size());
		}
		else rate_inlier = 0.;
		cout << "rate_inlier:" << rate_inlier << endl;
		//distance and error_Pos_translation
		double distance_ = 0.;
		double error_Pos_translation_ = 0.;
		{
			Eigen::Matrix4d T_i_src = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d T_i1_tgt = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d T_i_TRUE = Eigen::Matrix4d::Identity();
			//T_i_src = T_i1_tgt * T_i_GL
			T_i_src = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
			T_i1_tgt = calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
			T_i_TRUE = T_i1_tgt.inverse() * T_i_src;
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(T_i_TRUE);
			pcl::transformPointCloud(*cloud_src, *cloud_src, Trans_temp);
			//distance to true
			for (size_t i = 0; i < cloud_src_transformed->size(); i++)
			{
				T_PointType point_, point_true;
				point_ = cloud_src_transformed->points[i];
				point_true = cloud_src->points[i];
				const float sqrt_before =
					pow(point_.x - point_true.x, 2.)
					+ pow(point_.y - point_true.y, 2.)
					+ pow(point_.z - point_true.z, 2.);
				distance_ += static_cast<double>(sqrt(
					pow(point_.x - point_true.x, 2.)
					+ pow(point_.y - point_true.y, 2.)
					+ pow(point_.z - point_true.z, 2.)));
			}
			if (cloud_src_transformed->size() != 0) distance_ /= cloud_src_transformed->size();
			else distance_ = 100.;
			cout << "distance_:" << distance_ << endl;
			error_Pos_translation_ = sqrt(
				pow(Registration_Vector[0] - T_i_TRUE(0, 3), 2.)
				+ pow(Registration_Vector[1] - T_i_TRUE(1, 3), 2.)
				+ pow(Registration_Vector[2] - T_i_TRUE(2, 3), 2.));
			cout << "error_Pos_translation_:" << error_Pos_translation_ << endl;
		}
		//median
		double median_;
		median_ = getMedianDistance(cloud_src_transformed, cloud_tgt);
		cout << "median_:" << median_ << endl;
		//save pointcloud
		{
			//add
			*cloud_tgt += *cloud_src_transformed;
			//filename for saving PointCloud
			string s_filename_output;
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
			s_filename_output = "T" + s_tgt + "S" + s_src + "_XYZRGB";
			if (i_method == 1) s_filename_output += "_proposed";
			s_filename_output += ".pcd";
			//save
			pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_folder + "/" + s_filename_output, *cloud_tgt);
		}

		//output csv
		Eigen::Vector6d transform_vec = Eigen::Vector6d::Zero();
		transform_vec = calcVector6dFromHomogeneousMatrix(
			calcHomogeneousMatrixFromVector6d(Registration_Vector)
			*calcHomogeneousMatrixFromVector6d(initPos_vec[j].Init_Vector));
		string time_end_frame = CTimeString::getTimeString();
		string time_elapsed_frame = CTimeString::getTimeElapsefrom2Strings(time_start_frame, time_end_frame);
		vector<string> s_temp_vec;
		s_temp_vec.push_back(to_string(i_tgt));
		s_temp_vec.push_back(to_string(i_src));
		s_temp_vec.push_back(to_string(rate_inlier));
		s_temp_vec.push_back(to_string(distance_));
		s_temp_vec.push_back(to_string(median_));
		s_temp_vec.push_back(to_string(error_Pos_translation_));
		s_temp_vec.push_back(to_string(fitnessscore));
		s_temp_vec.push_back(time_elapsed_frame);
		s_temp_vec.push_back(to_string(transform_vec(0, 0)));	//X
		s_temp_vec.push_back(to_string(transform_vec(1, 0)));	//Y
		s_temp_vec.push_back(to_string(transform_vec(2, 0)));	//Z
		s_temp_vec.push_back(to_string(transform_vec(3, 0)));	//ROLL
		s_temp_vec.push_back(to_string(transform_vec(4, 0)));	//PITCH
		s_temp_vec.push_back(to_string(transform_vec(5, 0)));	//YAW
		s_output_vecvec.push_back(s_temp_vec);
		cout << endl;
	}

	//pushback
	for (int j = 0; j < s_output_vecvec.size(); j++)
		s_input_vecvec.push_back(s_output_vecvec[j]);
}

void CICPWithFeature::DoICP_proposed_varyParameters(string dir_, string filename_csv)
{
	vector<vector<float>> pattern_vecvec;
	{
		vector<vector<float>> parameter_vecvec;
		parameter_vecvec = CTimeString::inputParameters_2dimension(
			dir_ + "/" + "parameter_vecvec.csv", 1, 4);
		pattern_vecvec = CTimeString::calcVectorPairPattern(parameter_vecvec);
	}

	cout << "show patterns" << endl;
	for (int j = 0; j < pattern_vecvec.size(); j++)
	{
		cout << j << ":";
		for (int i = 0; i < pattern_vecvec[j].size(); i++)
		{
			string s_value;
			s_value = to_string(pattern_vecvec[j][i]);
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			cout << "  " << s_value;
		}
		cout << endl;
	}

	vector<string> s_name_parameter;
	s_name_parameter.push_back("MaximumIterations");
	s_name_parameter.push_back("MaxCorrespondenceDistance");
	s_name_parameter.push_back("EuclideanFitnessEpsilon");
	s_name_parameter.push_back("TransformationEpsilon");
	s_name_parameter.push_back("penalty_chara");
	s_name_parameter.push_back("weight_dist_chara");

	for (int j = 0; j < pattern_vecvec.size(); j++)
	{
		vector<float> parameter_vec;
		parameter_vec = pattern_vecvec[j];
		cout << "parameter" << endl;
		for (int i = 0; i < s_name_parameter.size(); i++)
			cout << s_name_parameter[i] << ": " << parameter_vec[i] << endl;
		DoICP_proposed_givenParameter(dir_, filename_csv, parameter_vec);
		cout << endl;
	}
}

void CICPWithFeature::DoICP_proposed_mergeResult()
{
	bool b_useMedian = false;
	//b_useMedian = true;

	string dir_ = "../../data/process_DoICP_proposed_AllFrames";

	vector<string> filenames_folder;
	vector<int> i_folder_vec;
	{
		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc (can input multinumber)" << endl;
		vector<string> s_input_vec;
		bool b_useCSV = false;
		{
			cout << "select: input number by csv or not  yes:1  no:0" << endl;
			cout << "->";
			cin >> b_useCSV;
		}
		if (!b_useCSV) s_input_vec = CTimeString::inputSomeString();
		else s_input_vec = CTimeString::inputSomeString_fromCSV(dir_ + "/" + "num_vector.csv");
		cout << "s_input_vec.size():" << s_input_vec.size() << endl;
		for (int i = 0; i < s_input_vec.size(); i++)
			i_folder_vec.push_back(stoi(s_input_vec[i]));
	}

	vector<vector<string>> s_output_vecvec;
	//header of output file
	{
		vector<string> s_output_vec;
		s_output_vec.push_back("Filename");
		s_output_vec.push_back("MaximumIterations");
		s_output_vec.push_back("MaxCorrespondenceDistance");
		s_output_vec.push_back("EuclideanFitnessEpsilon");
		s_output_vec.push_back("TransformationEpsilon");
		s_output_vec.push_back("penalty_chara");
		s_output_vec.push_back("weight_dist_chara");
		s_output_vec.push_back("Cluster");
		if (b_useMedian)
			s_output_vec.push_back("Distance_Median_Transformation");
		else
			s_output_vec.push_back("Distance_Transformation");
		s_output_vecvec.push_back(s_output_vec);
	}

	for (int i = 0; i < i_folder_vec.size(); i++)
		s_output_vecvec.push_back(
			DoICP_proposed_mergeResult_OnePattern(dir_, filenames_folder[i_folder_vec[i]], b_useMedian));

	////transposition
	//{
	//	vector<vector<string>> s_output_vecvec_temp;
	//	s_output_vecvec_temp = CTimeString::getTranspositionOfVecVec(s_output_vecvec);
	//	s_output_vecvec = s_output_vecvec_temp;
	//}

	string s_t = CTimeString::getTimeString();
	CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/ICPResult_" + s_t + ".csv");
}

vector<string> CICPWithFeature::DoICP_proposed_mergeResult_OnePattern(string dir_, string s_folder, bool b_useMedian)
{
	bool b_useTransformation = false;
	//b_useTransformation = true;

	vector<string> s_vec_output;
	s_vec_output.push_back(s_folder);

	vector<vector<string>> s_input_vecvec;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_, "_output_ICP.csv");
		if (filenames_.size() != 1)
		{
			cout << "ERROR: one fusion.csv have not been found" << endl;
			return s_vec_output;
		}
		s_input_vecvec = CTimeString::getVecVecFromCSV_string(
			dir_ + "/" + s_folder + "/" + filenames_[0]);
	}

	//ICP parameter
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(s_input_vecvec, "Parameter_ICP", 1, "Result_ICP", -2);
		s_vec_output.push_back(s_temp_vecvec[0][4]);
		s_vec_output.push_back(s_temp_vecvec[1][4]);
		s_vec_output.push_back(s_temp_vecvec[2][4]);
		s_vec_output.push_back(s_temp_vecvec[3][4]);
		s_vec_output.push_back(s_temp_vecvec[4][4]);
		s_vec_output.push_back(s_temp_vecvec[5][4]);
	}

	//Cluster
	vector<int> frames_all;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Result_ICP", 2, "Result_ICP_proposed", -2);
		//init frames_all
		{
			int num_frames = 0;
			for (int j = 0; j < s_temp_vecvec.size(); j++)
			{
				int i_tgt;
				int i_src;
				i_tgt = stod(s_temp_vecvec[j][0]);
				i_src = stod(s_temp_vecvec[j][1]);
				if (num_frames < i_tgt) num_frames = i_tgt;
				if (num_frames < i_src) num_frames = i_src;
			}
			num_frames++;
			for (int j = 0; j < num_frames; j++)
				frames_all.push_back(-1);
		}
		//clustering
		{
			vector<vector<int>> value_vecvec;
			for (int j = 0; j < s_temp_vecvec.size(); j++)
			{
				vector<int> value_vec;
				int i_tgt;
				int i_src;
				i_tgt = stod(s_temp_vecvec[j][0]);
				i_src = stod(s_temp_vecvec[j][1]);
				value_vec.push_back(i_tgt);
				value_vec.push_back(i_src);
				value_vecvec.push_back(value_vec);
			}
			vector<vector<int>> value_vecvec_new;
			value_vecvec_new = CTimeString::getIntCluster_SomeToSome(value_vecvec);
			int frame_valid = 0;
			for (int j = 0; j < value_vecvec_new[0].size(); j++)
			{
				frames_all[value_vecvec_new[0][j]] = frame_valid;
				frame_valid++;
			}
			//cout << "show deleted frames and others" << endl;
			//for (int i = 0; i < frames_all.size(); i++)
			//	cout << "i:" << i << " " << frames_all[i] << endl;
		}
		//add cluster to s_vec_output
		string s_cluster;
		for (int j = 0; j < frames_all.size(); j++)
			if (frames_all[j] != -1) s_cluster += to_string(j) + " ";
		s_vec_output.push_back(s_cluster);
	}

	//input ICP
	vector<vector<double>> value_vecvec_ICP;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Result_ICP", 2, "Result_ICP_proposed", -2);
		for (int j = 0; j < s_temp_vecvec.size(); j++)
		{
			int i_tgt;
			int i_src;
			i_tgt = stod(s_temp_vecvec[j][0]);
			i_src = stod(s_temp_vecvec[j][1]);
			if (frames_all[i_tgt] == -1) continue;
			if (frames_all[i_src] == -1) continue;
			vector<double> value_vec;
			value_vec.push_back(i_tgt);
			value_vec.push_back(i_src);
			value_vec.push_back(stod(s_temp_vecvec[j][3]));
			value_vec.push_back(stod(s_temp_vecvec[j][4]));
			value_vec.push_back(stod(s_temp_vecvec[j][5]));
			value_vecvec_ICP.push_back(value_vec);
		}
	}
	//input ICP_proposed
	vector<vector<double>> value_vecvec_ICP_proposed;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Result_ICP_proposed", 2, "", 1);
		for (int j = 0; j < s_temp_vecvec.size(); j++)
		{
			int i_tgt;
			int i_src;
			i_tgt = stod(s_temp_vecvec[j][0]);
			i_src = stod(s_temp_vecvec[j][1]);
			if (frames_all[i_tgt] == -1) continue;
			if (frames_all[i_src] == -1) continue;
			vector<double> value_vec;
			value_vec.push_back(i_tgt);
			value_vec.push_back(i_src);
			value_vec.push_back(stod(s_temp_vecvec[j][3]));
			value_vec.push_back(stod(s_temp_vecvec[j][4]));
			value_vec.push_back(stod(s_temp_vecvec[j][5]));
			value_vecvec_ICP_proposed.push_back(value_vec);
		}
	}

	//Compare
	for (int j = 0; j < value_vecvec_ICP.size(); j++)
	{
		string s_frame_pair = to_string((int)value_vecvec_ICP[j][0])
			+ "--" + to_string((int)value_vecvec_ICP[j][1]);
		double compare_distance;
		compare_distance = value_vecvec_ICP_proposed[j][2] - value_vecvec_ICP[j][2];
		double compare_median;
		compare_median = value_vecvec_ICP_proposed[j][3] - value_vecvec_ICP[j][3];
		double compare_transformation;
		compare_transformation = value_vecvec_ICP_proposed[j][4] - value_vecvec_ICP[j][4];
		s_vec_output.push_back(s_frame_pair);
		s_vec_output.push_back(to_string(compare_distance));
		if (b_useMedian)
			s_vec_output.push_back(to_string(compare_median));
		if (b_useTransformation)
			s_vec_output.push_back(to_string(compare_transformation));
	}

	return s_vec_output;
}

void CICPWithFeature::DoEvaluation_ICP_property(string dir_)
{
	int i_method;
	cout << "select: optimization:0  mergeResult:1" << endl;
	cout << "->";
	cin >> i_method;

	if (i_method == 0)
		DoEvaluation_ICP_property_files(dir_);
	else if (i_method == 1)
		DoEvaluation_ICP_property_mergeResult(dir_);
}

void CICPWithFeature::DoEvaluation_ICP_property_files(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	vector<string> filenames_input;
	{
		vector<string> filenames_temp;
		CTimeString::getFileNames_extension(dir_, filenames_temp, "_optimization.csv");
		for (int i = 0; i < filenames_temp.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_temp[i] << endl;
		}
		cout << endl;
		cout << "input .csv you want to calc (can input multinumber)" << endl;
		vector<string> s_input_vec;
		bool b_useCSV = false;
		{
			cout << "select: input number by csv or not  yes:1  no:0" << endl;
			cout << "->";
			cin >> b_useCSV;
		}
		if (!b_useCSV) s_input_vec = CTimeString::inputSomeString();
		else s_input_vec = CTimeString::inputSomeString_fromCSV(dir_ + "/" + "num_vector.csv");
		cout << "s_input_vec.size():" << s_input_vec.size() << endl;
		for (int i = 0; i < s_input_vec.size(); i++)
			filenames_input.push_back(filenames_temp[stoi(s_input_vec[i])]);
	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");
		for (int i = 0; i < filenames_.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	for (int j = 0; j < filenames_input.size(); j++)
	{
		string filename_input = filenames_input[j];
		string time_start = CTimeString::getTimeString();
		cout << "time_start:" << time_start << endl;
		//make new folder
		string s_newfoldername = time_start;
		CTimeString::makenewfolder(dir_, s_newfoldername);

		vector<vector<string>> s_output_vecvec;
		s_output_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + filename_input);
		DoEvaluation_ICP_property_addToFile(dir_, s_newfoldername, s_output_vecvec, cloud_vec);

		//output
		string filename_output;
		filename_output = s_newfoldername + "_evaluation.csv";
		CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + filename_output);
	}
}

void CICPWithFeature::DoEvaluation_ICP_property_addToFile(string dir_,
	string s_newfoldername, vector<vector<string>> &s_input_vecvec,
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vec)
{
	typedef pcl::PointXYZRGB T_PointType;

	//input trajectory of ICP
	vector<Eigen::Vector6d> trajectoryVector_vec_ICP;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Result_PoseGraphOptimization", 2, "Result_PoseGraphOptimization_proposed", -2);
		for (int j = 0; j < s_temp_vecvec.size(); j++)
		{
			Eigen::Vector6d trajectoryVector_ = Eigen::Vector6d::Zero();
			trajectoryVector_ <<
				stod(s_temp_vecvec[j][1]),
				stod(s_temp_vecvec[j][2]),
				stod(s_temp_vecvec[j][3]),
				stod(s_temp_vecvec[j][4]),
				stod(s_temp_vecvec[j][5]),
				stod(s_temp_vecvec[j][6]);
			trajectoryVector_vec_ICP.push_back(trajectoryVector_);
		}
	}

	//input trajectory of ICP_proposed
	vector<Eigen::Vector6d> trajectoryVector_vec_ICP_proposed;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Result_PoseGraphOptimization_proposed", 2, "", 1);
		for (int j = 0; j < s_temp_vecvec.size(); j++)
		{
			Eigen::Vector6d trajectoryVector_ = Eigen::Vector6d::Zero();
			trajectoryVector_ <<
				stod(s_temp_vecvec[j][1]),
				stod(s_temp_vecvec[j][2]),
				stod(s_temp_vecvec[j][3]),
				stod(s_temp_vecvec[j][4]),
				stod(s_temp_vecvec[j][5]),
				stod(s_temp_vecvec[j][6]);
			trajectoryVector_vec_ICP_proposed.push_back(trajectoryVector_);
		}
	}

	vector<int> frames_all;
	for (int j = 0; j < trajectoryVector_vec_ICP_proposed.size(); j++)
	{
		Eigen::Vector6d trajectoryVector_ = trajectoryVector_vec_ICP_proposed[j];
		int num_minus1 = 0;
		if (trajectoryVector_(0, 0) == -1) num_minus1++;
		if (trajectoryVector_(1, 0) == -1) num_minus1++;
		if (trajectoryVector_(2, 0) == -1) num_minus1++;
		if (trajectoryVector_(3, 0) == -1) num_minus1++;
		if (trajectoryVector_(4, 0) == -1) num_minus1++;
		if (trajectoryVector_(5, 0) == -1) num_minus1++;
		if (num_minus1 == 6) frames_all.push_back(-1);
		else  frames_all.push_back(j);
	}

	//true trajectory
	vector<Eigen::Vector6d> trajectoryVector_vec_TRUE;
	{
		string filename_true = "transformation_fin.csv";
		vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
		for (int j = 0; j < trajectory_vecvec_temp.size(); j++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[j][1], trajectory_vecvec_temp[j][2],
				trajectory_vecvec_temp[j][3], trajectory_vecvec_temp[j][4],
				trajectory_vecvec_temp[j][5], trajectory_vecvec_temp[j][6];
			trajectoryVector_vec_TRUE.push_back(Pos_temp);
		}
	}

	vector<vector<string>> s_output_vecvec;
	vector<string> s_headers_evaluation;
	s_headers_evaluation.push_back("Frame");
	s_headers_evaluation.push_back("X");
	s_headers_evaluation.push_back("Y");
	s_headers_evaluation.push_back("Z");
	s_headers_evaluation.push_back("ROLL");
	s_headers_evaluation.push_back("PITCH");
	s_headers_evaluation.push_back("YAW");
	s_headers_evaluation.push_back("Pos_relative");
	s_headers_evaluation.push_back("Pos_absolute");
	s_headers_evaluation.push_back("frameCloudMedian_vecvec");
	s_headers_evaluation.push_back("Mean_map");

	//calc
	vector<vector<double>> error_relative_vecvec;
	vector<vector<double>> error_absolute_vecvec;
	vector<vector<double>> frameCloudMedian_vecvec;
	vector<double> mean_error_relative_vec;
	vector<double> mean_error_absolute_vec;
	vector<double> mean_map_vec;
	//int
	{
		double temp_;
		vector<double> temp_vec;
		mean_error_relative_vec.push_back(temp_);
		mean_error_relative_vec.push_back(temp_);
		mean_error_absolute_vec.push_back(temp_);
		mean_error_absolute_vec.push_back(temp_);
		mean_map_vec.push_back(temp_);
		mean_map_vec.push_back(temp_);
		error_relative_vecvec.push_back(temp_vec);
		error_relative_vecvec.push_back(temp_vec);
		error_absolute_vecvec.push_back(temp_vec);
		error_absolute_vecvec.push_back(temp_vec);
		frameCloudMedian_vecvec.push_back(temp_vec);
		frameCloudMedian_vecvec.push_back(temp_vec);
	}

	//calc ICP
	{
		int i_method = 0;
		DoEvaluation_ICP_property_calculation(dir_, s_newfoldername, trajectoryVector_vec_ICP,
			cloud_vec, trajectoryVector_vec_TRUE, i_method, error_relative_vecvec[i_method], error_absolute_vecvec[i_method],
			frameCloudMedian_vecvec[i_method], mean_map_vec[i_method]);
		double mean_error_relative_vecvec = 0.;
		for (int j = 0; j < error_relative_vecvec[i_method].size(); j++)
		{
			if (j == 0) continue;
			mean_error_relative_vecvec += error_relative_vecvec[i_method][j];
		}
		if (error_relative_vecvec[i_method].size() == 0) mean_error_relative_vecvec = 10000.;
		else mean_error_relative_vecvec /= (float)(error_relative_vecvec[i_method].size() - 1);
		mean_error_relative_vec[i_method] = mean_error_relative_vecvec;
		double mean_error_absolute_vecvec = 0.;
		for (int j = 0; j < error_absolute_vecvec[i_method].size(); j++)
		{
			if (j == 0) continue;
			mean_error_absolute_vecvec += error_absolute_vecvec[i_method][j];
		}
		if (error_relative_vecvec[i_method].size() == 0) mean_error_absolute_vecvec = 10000.;
		else mean_error_absolute_vecvec /= (float)(error_absolute_vecvec[i_method].size() - 1);
		mean_error_absolute_vec[i_method] = mean_error_absolute_vecvec;
	}
	//output ICP
	{
		int i_method = 0;
		{
			vector<string> s_temp_vec;
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Evaluation_OptimizedICP");
			s_output_vecvec.push_back(s_temp_vec);
		}
		s_output_vecvec.push_back(s_headers_evaluation);
		int i_frame_valid = 0;
		for (int j = 0; j < frames_all.size(); j++)
		{
			if (frames_all[j] == -1) continue;
			vector<string> s_temp_vec;
			s_temp_vec.push_back(to_string(i_frame_valid));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP[j](0, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP[j](1, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP[j](2, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP[j](3, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP[j](4, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP[j](5, 0)));
			s_temp_vec.push_back(to_string(error_relative_vecvec[i_method][i_frame_valid]));
			s_temp_vec.push_back(to_string(error_absolute_vecvec[i_method][i_frame_valid]));
			s_temp_vec.push_back(to_string(frameCloudMedian_vecvec[i_method][i_frame_valid]));
			s_output_vecvec.push_back(s_temp_vec);
			i_frame_valid++;
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Mean");
			for (int i = 0; i < 6; i++)
				s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(mean_error_relative_vec[i_method]));
			s_temp_vec.push_back(to_string(mean_error_absolute_vec[i_method]));
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(mean_map_vec[i_method]));
			s_output_vecvec.push_back(s_temp_vec);
		}
	}

	//calc ICP_proposed
	{
		int i_method = 1;
		DoEvaluation_ICP_property_calculation(dir_, s_newfoldername, trajectoryVector_vec_ICP_proposed,
			cloud_vec, trajectoryVector_vec_TRUE, i_method, error_relative_vecvec[i_method], error_absolute_vecvec[i_method],
			frameCloudMedian_vecvec[i_method], mean_map_vec[i_method]);
		double mean_error_relative_vecvec = 0.;
		for (int j = 0; j < error_relative_vecvec[i_method].size(); j++)
		{
			if (j == 0) continue;
			mean_error_relative_vecvec += error_relative_vecvec[i_method][j];
		}
		if (error_relative_vecvec[i_method].size() == 0) mean_error_relative_vecvec = 10000.;
		else mean_error_relative_vecvec /= (float)(error_relative_vecvec[i_method].size() - 1);
		mean_error_relative_vec[i_method] = mean_error_relative_vecvec;
		double mean_error_absolute_vecvec = 0.;
		for (int j = 0; j < error_absolute_vecvec[i_method].size(); j++)
		{
			if (j == 0) continue;
			mean_error_absolute_vecvec += error_absolute_vecvec[i_method][j];
		}
		if (error_relative_vecvec[i_method].size() == 0) mean_error_absolute_vecvec = 10000.;
		else mean_error_absolute_vecvec /= (float)(error_absolute_vecvec[i_method].size() - 1);
		mean_error_absolute_vec[i_method] = mean_error_absolute_vecvec;
	}
	//output ICP_proposed
	{
		int i_method = 1;
		{
			vector<string> s_temp_vec;
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Evaluation_OptimizedICP_proposed");
			s_output_vecvec.push_back(s_temp_vec);
		}
		s_output_vecvec.push_back(s_headers_evaluation);
		int i_frame_valid = 0;
		for (int j = 0; j < frames_all.size(); j++)
		{
			if (frames_all[j] == -1) continue;
			vector<string> s_temp_vec;
			s_temp_vec.push_back(to_string(i_frame_valid));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP_proposed[j](0, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP_proposed[j](1, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP_proposed[j](2, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP_proposed[j](3, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP_proposed[j](4, 0)));
			s_temp_vec.push_back(to_string(trajectoryVector_vec_ICP_proposed[j](5, 0)));
			s_temp_vec.push_back(to_string(error_relative_vecvec[i_method][i_frame_valid]));
			s_temp_vec.push_back(to_string(error_absolute_vecvec[i_method][i_frame_valid]));
			s_temp_vec.push_back(to_string(frameCloudMedian_vecvec[i_method][i_frame_valid]));
			s_output_vecvec.push_back(s_temp_vec);
			i_frame_valid++;
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Mean");
			for (int i = 0; i < 6; i++)
				s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(mean_error_relative_vec[i_method]));
			s_temp_vec.push_back(to_string(mean_error_absolute_vec[i_method]));
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(mean_map_vec[i_method]));
			s_output_vecvec.push_back(s_temp_vec);
		}
	}

	//compare
	{
		{
			vector<string> s_temp_vec;
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Compare");
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			for (int j = 0; j < s_headers_evaluation.size(); j++)
			{
				if ((1 <= j && j <= 6) || j == 10)
					s_temp_vec.push_back("");
				else
					s_temp_vec.push_back(s_headers_evaluation[j]);
			}
			s_output_vecvec.push_back(s_temp_vec);
		}
		for (int j = 0; j < error_relative_vecvec[0].size(); j++)
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back(to_string(j));
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(error_relative_vecvec[1][j] - error_relative_vecvec[0][j]));
			s_temp_vec.push_back(to_string(error_absolute_vecvec[1][j] - error_absolute_vecvec[0][j]));
			s_temp_vec.push_back(to_string(frameCloudMedian_vecvec[1][j] - frameCloudMedian_vecvec[0][j]));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Mean");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(mean_error_relative_vec[1] - mean_error_relative_vec[0]));
			s_temp_vec.push_back(to_string(mean_error_absolute_vec[1] - mean_error_absolute_vec[0]));
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(mean_map_vec[1] - mean_map_vec[0]));
			s_output_vecvec.push_back(s_temp_vec);
		}

	}
	//result
	s_input_vecvec.insert(s_input_vecvec.end(), s_output_vecvec.begin(), s_output_vecvec.end());
}

void CICPWithFeature::DoEvaluation_ICP_property_calculation(string dir_, string s_folder, vector<Eigen::Vector6d> trajectoryVector_vec,
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vec, vector<Eigen::Vector6d> trajectoryVector_vec_TRUE,
	int i_method, vector<double> &error_relative_vec, vector<double> &error_absolute_vec,
	vector<double> &frameCloudMedian_vec, double &map_mean)
{
	typedef pcl::PointXYZRGB T_PointType;

	vector<int> frames_all;
	{
		for (int j = 0; j < trajectoryVector_vec.size(); j++)
		{
			int num_minus1 = 0;
			if (trajectoryVector_vec[j](0, 0) == -1) num_minus1++;
			if (trajectoryVector_vec[j](1, 0) == -1) num_minus1++;
			if (trajectoryVector_vec[j](2, 0) == -1) num_minus1++;
			if (trajectoryVector_vec[j](3, 0) == -1) num_minus1++;
			if (trajectoryVector_vec[j](4, 0) == -1) num_minus1++;
			if (trajectoryVector_vec[j](5, 0) == -1) num_minus1++;
			if (num_minus1 == 6) frames_all.push_back(-1);
			else frames_all.push_back(j);
		}
	}

	vector<int> frames_ajusted;
	for (int j = 0; j < frames_all.size(); j++)
	{
		if (frames_all[j] == -1) continue;
		frames_ajusted.push_back(frames_all[j]);
	}


	//calc homogenerous of displacement
	vector<Eigen::Matrix4d> displacementMat_vector_TRUE;
	for (int j = 0; j < frames_ajusted.size(); j++)
	{
		Eigen::Matrix4d displacementMat_ = Eigen::Matrix4d::Identity();
		if (j != 0)
		{
			displacementMat_ =
				calcHomogeneousMatrixFromVector6d(
					trajectoryVector_vec_TRUE[frames_ajusted[j - 1]]).inverse()
				* calcHomogeneousMatrixFromVector6d(
					trajectoryVector_vec_TRUE[frames_ajusted[j]]);
		}
		displacementMat_vector_TRUE.push_back(displacementMat_);
	}

	vector<Eigen::Matrix4d> displacementMat_vector;
	for (int j = 0; j < frames_ajusted.size(); j++)
	{
		Eigen::Matrix4d displacementMat_ = Eigen::Matrix4d::Identity();
		if (j != 0)
		{
			displacementMat_ =
				calcHomogeneousMatrixFromVector6d(
					trajectoryVector_vec[frames_ajusted[j - 1]]).inverse()
				* calcHomogeneousMatrixFromVector6d(
					trajectoryVector_vec[frames_ajusted[j]]);
		}
		displacementMat_vector.push_back(displacementMat_);
	}

	//calc error of relative displacement at each frame
	error_relative_vec.clear();
	for (int j = 0; j < displacementMat_vector.size(); j++)
	{
		double error_relative = sqrt(
			pow(displacementMat_vector[j](0, 3) - displacementMat_vector_TRUE[j](0, 3), 2.)
			+ pow(displacementMat_vector[j](1, 3) - displacementMat_vector_TRUE[j](1, 3), 2.)
			+ pow(displacementMat_vector[j](2, 3) - displacementMat_vector_TRUE[j](2, 3), 2.));
		error_relative_vec.push_back(error_relative);
	}
	//calc error of position (in world coordinate) at each frame
	error_absolute_vec.clear();
	for (int j = 0; j < frames_all.size(); j++)
	{
		if (frames_all[j] == -1) continue;
		double error_absolute = sqrt(
			pow(trajectoryVector_vec[j](0, 0)
				- trajectoryVector_vec_TRUE[j](0, 0), 2.)
			+ pow(trajectoryVector_vec[j](1, 0)
				- trajectoryVector_vec_TRUE[j](1, 0), 2.)
			+ pow(trajectoryVector_vec[j](2, 0)
				- trajectoryVector_vec_TRUE[j](2, 0), 2.));
		error_absolute_vec.push_back(error_absolute);
	}

	//calc median of pointcloud distance between each frame
	frameCloudMedian_vec.clear();
	for (int j = 0; j < frames_ajusted.size(); j++)
	{
		if (j == 0)
		{
			frameCloudMedian_vec.push_back(0.);
			continue;
		}
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[frames_ajusted[j - 1]], *cloud_tgt);
		pcl::copyPointCloud(*cloud_vec[frames_ajusted[j]], *cloud_src);
		Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
		Trans_temp = calcAffine3fFromHomogeneousMatrix(
			displacementMat_vector[j]);
		pcl::transformPointCloud(*cloud_src, *cloud_src, Trans_temp);
		frameCloudMedian_vec.push_back(getMedianDistance(cloud_src, cloud_tgt));
	}

	//map
	pcl::PointCloud<T_PointType>::Ptr cloud_map_TRUE(new pcl::PointCloud<T_PointType>());
	for (int j = 0; j < frames_ajusted.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[frames_ajusted[j]], *cloud_);
		Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
		Trans_temp = calcAffine3fFromHomogeneousMatrix(displacementMat_vector_TRUE[j]);
		pcl::transformPointCloud(*cloud_, *cloud_, Trans_temp);
		*cloud_map_TRUE += *cloud_;
	}
	pcl::PointCloud<T_PointType>::Ptr cloud_map_(new pcl::PointCloud<T_PointType>());
	for (int j = 0; j < frames_ajusted.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[frames_ajusted[j]], *cloud_);
		Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
		Trans_temp = calcAffine3fFromHomogeneousMatrix(displacementMat_vector[j]);
		pcl::transformPointCloud(*cloud_, *cloud_, Trans_temp);
		*cloud_map_ += *cloud_;
	}

	//save map
	string s_filename_cloud;
	s_filename_cloud = "map_ICP";
	if (i_method == 1) s_filename_cloud += "_proposed";
	pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_folder + "/" + s_filename_cloud + ".pcd", *cloud_map_);

	//map_mean
	{
		double distance_ = 0;
		for (int i = 0; i < cloud_map_->size(); i++)
		{
			auto point_ = cloud_map_->points[i];
			auto point_TRUE = cloud_map_TRUE->points[i];
			distance_ += sqrt(
				pow(point_.x - point_TRUE.x, 2.)
				+ pow(point_.y - point_TRUE.y, 2.)
				+ pow(point_.z - point_TRUE.z, 2.));
		}
		if (cloud_map_->size() == 0) distance_ = 10000.;
		else distance_ /= (float)cloud_map_->size();
		map_mean = distance_;
	}
}

void CICPWithFeature::DoEvaluation_ICP_property_mergeResult(string dir_)
{
	vector<string> filenames_folder;
	vector<int> i_folder_vec;
	{
		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc (can input multinumber)" << endl;
		vector<string> s_input_vec;
		bool b_useCSV = false;
		{
			cout << "select: input number by csv or not  yes:1  no:0" << endl;
			cout << "->";
			cin >> b_useCSV;
		}
		if (!b_useCSV) s_input_vec = CTimeString::inputSomeString();
		else s_input_vec = CTimeString::inputSomeString_fromCSV(dir_ + "/" + "num_vector.csv");
		cout << "s_input_vec.size():" << s_input_vec.size() << endl;
		for (int i = 0; i < s_input_vec.size(); i++)
			i_folder_vec.push_back(stoi(s_input_vec[i]));
	}

	vector<vector<string>> s_output_vecvec;
	//header of output file
	{
		//get frame size
		int num_allFrames;
		{
			vector<vector<string>> s_temp1_vecvec;
			{
				vector<string> filenames_temp1;
				CTimeString::getFileNames_extension(dir_ + "/" + filenames_folder[i_folder_vec[0]], filenames_temp1, "_evaluation.csv");
				if (filenames_temp1.size() != 1)
				{
					cout << "ERROR: one fusion.csv have not been found" << endl;
				}
				s_temp1_vecvec = CTimeString::getVecVecFromCSV_string(
					dir_ + "/" + filenames_folder[i_folder_vec[0]] + "/" + filenames_temp1[0]);
			}
			vector<vector<string>> s_temp2_vecvec;
			s_temp2_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
				s_temp1_vecvec, "Result_PoseGraphOptimization", 2, "Result_PoseGraphOptimization_proposed", -2);
			num_allFrames = stoi(s_temp2_vecvec.back()[0]);
			num_allFrames++;
			cout << "num_allFrames:" << num_allFrames << endl;
		}
		vector<string> s_output_vec;
		s_output_vec.push_back("Filename");
		s_output_vec.push_back("MaximumIterations");
		s_output_vec.push_back("MaxCorrespondenceDistance");
		s_output_vec.push_back("EuclideanFitnessEpsilon");
		s_output_vec.push_back("TransformationEpsilon");
		s_output_vec.push_back("penalty_chara");
		s_output_vec.push_back("weight_dist_chara");
		s_output_vec.push_back("edge_inf_threshold");
		s_output_vec.push_back("max_correspondence_distance");
		s_output_vec.push_back("edge_prune_threshold");
		s_output_vec.push_back("Pos_relative");
		for (int j = 0; j < num_allFrames; j++)
			s_output_vec.push_back(to_string(j));
		s_output_vec.push_back("Pos_absolute");
		for (int j = 0; j < num_allFrames; j++)
			s_output_vec.push_back(to_string(j));
		s_output_vec.push_back("Mean_relative");
		s_output_vec.push_back("Mean_absolute");
		s_output_vec.push_back("Mean_map");
		s_output_vec.push_back("Cluster_biggest");
		s_output_vec.push_back("Cluster_size");
		s_output_vecvec.push_back(s_output_vec);
	}

	for (int i = 0; i < i_folder_vec.size(); i++)
		s_output_vecvec.push_back(
			DoEvaluation_ICP_property_mergeResult_OnePattern(dir_, filenames_folder[i_folder_vec[i]]));

	//transposition
	{
		vector<vector<string>> s_output_vecvec_temp;
		s_output_vecvec_temp = CTimeString::getTranspositionOfVecVec(s_output_vecvec);
		s_output_vecvec = s_output_vecvec_temp;
	}

	string s_t = CTimeString::getTimeString();
	CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/EvaluationResult_" + s_t + ".csv");
}

vector<string> CICPWithFeature::DoEvaluation_ICP_property_mergeResult_OnePattern(string dir_, string s_folder)
{
	vector<string> s_vec_output;
	s_vec_output.push_back(s_folder);

	vector<vector<string>> s_input_vecvec;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_, "_evaluation.csv");
		if (filenames_.size() != 1)
		{
			cout << "ERROR: one fusion.csv have not been found" << endl;
			return s_vec_output;
		}
		s_input_vecvec = CTimeString::getVecVecFromCSV_string(
			dir_ + "/" + s_folder + "/" + filenames_[0]);
	}

	//ICP parameter
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(s_input_vecvec, "Parameter_ICP", 1, "Result_ICP", -2);
		s_vec_output.push_back(s_temp_vecvec[0][4]);
		s_vec_output.push_back(s_temp_vecvec[1][4]);
		s_vec_output.push_back(s_temp_vecvec[2][4]);
		s_vec_output.push_back(s_temp_vecvec[3][4]);
		s_vec_output.push_back(s_temp_vecvec[4][4]);
		s_vec_output.push_back(s_temp_vecvec[5][4]);
	}

	//PoseGraphOptimization parameter
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Parameter_PoseGraphOptimization", 2, "Result_PoseGraphOptimization", -2);
		s_vec_output.push_back(s_temp_vecvec[0][4]);
		s_vec_output.push_back(s_temp_vecvec[1][4]);
		s_vec_output.push_back(s_temp_vecvec[2][4]);
	}

	vector<int> frames_all;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Result_PoseGraphOptimization", 2, "Result_PoseGraphOptimization_proposed", -2);
		for (int j = 0; j < s_temp_vecvec.size(); j++)
		{
			if (stoi(s_temp_vecvec[j][7]) == 1) frames_all.push_back(-1);
			else
				frames_all.push_back(j);
		}
	}

	//Compare
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getMatrixData_fromFormatOfFPFH(
			s_input_vecvec, "Compare", 2, "", 1);

		//Pos_relative
		s_vec_output.push_back("");
		{
			int num_valid = 0;
			for (int j = 0; j < frames_all.size(); j++)
			{
				if (frames_all[j] == -1) s_vec_output.push_back("");
				else
				{
					s_vec_output.push_back(s_temp_vecvec[num_valid][7]);
					num_valid++;
				}
			}
		}

		//Pos_absolute
		s_vec_output.push_back("");
		{
			int num_valid = 0;
			for (int j = 0; j < frames_all.size(); j++)
			{
				if (frames_all[j] == -1) s_vec_output.push_back("");
				else
				{
					s_vec_output.push_back(s_temp_vecvec[num_valid][8]);
					num_valid++;
				}
			}
		}

		//Mean_relative
		s_vec_output.push_back(s_temp_vecvec.back()[7]);
		//Mean_absolute
		s_vec_output.push_back(s_temp_vecvec.back()[8]);
		//Mean_map
		s_vec_output.push_back(s_temp_vecvec.back()[10]);

		//cluster
		vector<int> frames_valid;
		string s_cluster;
		{
			int num_validFrames = 0;
			for (int j = 0; j < frames_all.size(); j++)
			{
				if (frames_all[j] == -1) continue;
				frames_valid.push_back(num_validFrames);
				s_cluster += to_string(j) + " ";
				num_validFrames++;
			}
		}
		s_vec_output.push_back(s_cluster);
		s_vec_output.push_back(to_string(frames_valid.size()));
	}

	return s_vec_output;
}
