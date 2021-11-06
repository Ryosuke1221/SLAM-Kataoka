#include "PointcloudGeneration.h"

void CPointcloudGeneration::mainProcess()
{
	string dir_;
	//dir_ = "../../data/temp/_Hand";
	dir_ = "../../data/data_test_PointcloudGeneration";

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

		EN_PCDGeneration_fromCSV,
		EN_PCDGeneration_fromPCD,
		EN_NarahaWinter202001,
		EN_ThermalCloudGeneration,

		EN_GetPcdFromCSV,
		EN_FilterPointCloud,
		EN_CombinePointCloud,
		EN_CSV_FromPointCloud,
		EN_Segmentation,
		EN_DoOutlierRejector,
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

		cout << " " << EN_PCDGeneration_fromCSV << ": PCDGeneration_fromCSV" << endl;
		cout << " " << EN_PCDGeneration_fromPCD << ": PCDGeneration_fromPCD" << endl;
		cout << " " << EN_NarahaWinter202001 << ": NarahaWinter202001" << endl;
		cout << " " << EN_ThermalCloudGeneration << ": ThermalCloudGeneration" << endl;

		cout << "WhichProcess: ";
		cin >> WhichProcess;

		cout << endl;
		cout << "//////////////////////////////////" << endl;
		cout << endl;

		switch (WhichProcess)
		{
		case EN_escape:
			//escape
			b_finish = true;
			break;

		case EN_FreeSpace:
			CPointcloudGeneration::FreeSpace();
			break;

		case EN_FileProcess:
			FileProcess(dir_);
			break;

		case EN_SequentShow:
			//show_sequent(dir_);
			show_sequent_PointTypes(dir_);
			break;

		case EN_DrawTrajectory:
			DrawTrajectory(dir_ + "/01_DrawTrajectory");
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory(dir_ + "/02_MappingFromTrajectory");
			break;

		case EN_PCDGeneration_fromCSV:
			PCDGeneration_fromCSV(dir_ + "/03_PCDGeneration_fromCSV");
			break;

		case EN_PCDGeneration_fromPCD:
			PCDGeneration_fromPCD(dir_ + "/04_PCDGeneration_fromPCD");
			break;

		case EN_NarahaWinter202001:
			NarahaWinter202001(dir_ + "/05_NarahaWinter202001");

		case EN_ThermalCloudGeneration:
			ThermalCloudGeneration(dir_ + "/06_thermal");

		default:
			break;
		}
	}
}

void CPointcloudGeneration::FreeSpace()
{
	
}

void CPointcloudGeneration::PCDGeneration_fromCSV(string dir_)
{
	enum
	{
		EN_XYZI_fromXYZ,
		XYZI_fromXYZI,
		EN_XYZRGB_fromXYZRGB
	};

	cout << EN_XYZI_fromXYZ << ": XYZI from .csv(index, X, Y, Z)" << endl;
	cout << XYZI_fromXYZI << ": XYZI .csv(index, X, Y, Z, Intensity)" << endl;
	cout << EN_XYZRGB_fromXYZRGB << ": XYZRGB from .csv(index, X, Y, Z, R, G, B)" << endl;
	cout << "->";
	
	int i_method;
	cin >> i_method;

	if (i_method == EN_XYZI_fromXYZ)
	{
		typedef typename pcl::PointXYZI T_PointType;

		vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;

		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".csv");

		for (int j = 0; j < filenames_.size(); j++)
		{
			vector<vector<double>> pc_vecvec = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames_[j]);
			pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
			for (int i = 0; i < pc_vecvec.size(); i++)
			{
				T_PointType point_;
				point_.x = pc_vecvec[i][1];
				point_.y = pc_vecvec[i][2];
				point_.z = pc_vecvec[i][3];
				point_.intensity = 255;
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}

		if (cloud_vec.size() == 0)
		{
			cout << "ERROR: no file found." << endl;
			throw std::runtime_error("ERROR: no file found.");
		}

		//save
		string s_foldername = CTimeString::getTimeString() + "_Output_XYZI";
		CTimeString::makenewfolder(dir_, s_foldername);
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			string s_name;
			s_name = filenames_[j].substr(0, filenames_[j].size() - 4) + ".pcd";
			pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_foldername + "/" + s_name, *cloud_vec[j]);
		}
	}

	else if (i_method == XYZI_fromXYZI)
	{
		typedef typename pcl::PointXYZI T_PointType;

		vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;

		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".csv");

		for (int j = 0; j < filenames_.size(); j++)
		{
			vector<vector<double>> pc_vecvec = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames_[j]);
			pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
			for (int i = 0; i < pc_vecvec.size(); i++)
			{
				T_PointType point_;
				point_.x = pc_vecvec[i][1];
				point_.y = pc_vecvec[i][2];
				point_.z = pc_vecvec[i][3];
				point_.intensity = pc_vecvec[i][4];
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}

		if (cloud_vec.size() == 0)
		{
			cout << "ERROR: no file found." << endl;
			throw std::runtime_error("ERROR: no file found.");
		}

		//save
		string s_foldername = CTimeString::getTimeString() + "_Output_XYZI";
		CTimeString::makenewfolder(dir_, s_foldername);
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			string s_name;
			s_name = filenames_[j].substr(0, filenames_[j].size() - 4) + ".pcd";
			pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_foldername + "/" + s_name, *cloud_vec[j]);
		}

	}

	else if (i_method == EN_XYZI_fromXYZ)
	{
		typedef typename pcl::PointXYZRGB T_PointType;

		vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;

		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".csv");

		for (int j = 0; j < filenames_.size(); j++)
		{
			vector<vector<double>> pc_vecvec = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames_[j]);
			pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
			for (int i = 0; i < pc_vecvec.size(); i++)
			{
				T_PointType point_;
				point_.x = pc_vecvec[i][1];
				point_.y = pc_vecvec[i][2];
				point_.z = pc_vecvec[i][3];
				point_.r = pc_vecvec[i][4];
				point_.g = pc_vecvec[i][5];
				point_.b = pc_vecvec[i][6];
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}

		if (cloud_vec.size() == 0)
		{
			cout << "ERROR: no file found." << endl;
			throw std::runtime_error("ERROR: no file found.");
		}

		//save
		string s_foldername = CTimeString::getTimeString() + "_Output_XYZRGB";
		CTimeString::makenewfolder(dir_, s_foldername);
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			string s_name;
			s_name = filenames_[j].substr(0, filenames_[j].size() - 4) + ".pcd";
			pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_foldername + "/" + s_name, *cloud_vec[j]);
		}

	}
}

void CPointcloudGeneration::PCDGeneration_fromPCD(string dir_)
{
	enum
	{
		EN_XYZRGB_fromXYZI,
		EN_XYZI_fromXYZRGB,
	};

	cout << EN_XYZRGB_fromXYZI << ": XYZRGB_fromXYZI" << endl;
	cout << EN_XYZI_fromXYZRGB << ": XYZI_fromXYZRGB" << endl;
	cout << "->";

	int i_method;
	cin >> i_method;

	if (i_method == EN_XYZRGB_fromXYZI)
	{
		typedef typename pcl::PointXYZI T_PointType_input;
		typedef typename pcl::PointXYZRGB T_PointType_output;

		string s_folder_input = "_XYZI";

		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder_input, filenames_, ".pcd");

		vector<pcl::PointCloud<T_PointType_input>::Ptr> cloud_input_vec;
		for (int j = 0; j < filenames_.size(); j++)
		{
			pcl::PointCloud<T_PointType_input>::Ptr cloud_(new pcl::PointCloud<T_PointType_input>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder_input + "/" + filenames_[j], *cloud_);
			cloud_->is_dense = true;
			cloud_input_vec.push_back(cloud_);
		}

		if (cloud_input_vec.size() == 0)
		{
			cout << "ERROR: no file found." << endl;
			throw std::runtime_error("ERROR: no file found.");
		}

		vector<vector<float>> intensity_vecvec;
		float value_max = -std::numeric_limits<float>::max();
		float value_min = std::numeric_limits<float>::max();
		for (int j = 0; j < cloud_input_vec.size(); j++)
		{
			vector<float> intensity_vec;
			for (int i = 0; i < cloud_input_vec[j]->size(); i++)
			{
				float value_ = cloud_input_vec[j]->points[i].intensity;
				if (value_max < value_) value_max = value_;
				if (value_min > value_) value_min = value_;
				intensity_vec.push_back(value_);
			}
			intensity_vecvec.push_back(intensity_vec);
			cout << "j:" << j << " size:" << intensity_vec.size() << endl;
		}

		cout << "value_max:" << value_max << endl;
		cout << "value_min:" << value_min << endl;

		//Quartile
		cout << endl;
		cout << "Do you calculate quartile of intensity ?  1:Yes  0:No" << endl;
		cout << "->";
		bool b_calcQuartile = false;
		cin >> b_calcQuartile;
		if(b_calcQuartile)
		{
			string t_start_q = CTimeString::getTimeString();
			vector<float> intensity_vec_all;
			for (int j = 0; j < intensity_vecvec.size(); j++)
			{
				intensity_vec_all.insert(intensity_vec_all.end(), intensity_vecvec[j].begin(), intensity_vecvec[j].end());
			}
			vector<float> Quartile_vec = CTimeString::getMedian_Quartile(intensity_vec_all);
			cout << " first_quartile:" << Quartile_vec[0];
			cout << " median_:" << Quartile_vec[1];
			cout << " third_quartile:" << Quartile_vec[1] << endl;
			cout << endl;
			string t_end_q = CTimeString::getTimeString();
			cout << "elapsed time (quartile):" << CTimeString::getTimeElapsefrom2Strings(t_start_q, t_end_q) << endl;;
		}

		float th_max, th_min;
		th_max = value_max;
		th_min = value_min;
		//th_max = 180.;//naraha winter?

		//change
		cout << endl;
		cout << "th_max(manual):" << th_max << endl;
		cout << "th_min(manual):" << th_min << endl;
		cout << "Change?  1:Yes  0:No" << endl;
		cout << "->";
		bool b_change = false;
		cin >> b_change;
		if (b_change)
		{
			cout << "th_max ->";
			cin >> th_max;
			cout << "th_min ->";
			cin >> th_min;
			cout << "th_max(changed):" << th_max << endl;
			cout << "th_min(changed):" << th_min << endl;
		}

		vector<pcl::PointCloud<T_PointType_output>::Ptr> cloud_vec;
		for (int j = 0; j < intensity_vecvec.size(); j++)
		{
			pcl::PointCloud<T_PointType_output>::Ptr cloud_(new pcl::PointCloud<T_PointType_output>());
			for (int i = 0; i < intensity_vecvec[j].size(); i++)
			{
				vector<std::uint8_t> color_vec;
				color_vec = CPointVisualization<pcl::PointXYZRGB>::getRGBwithValuebyPseudoColor(intensity_vecvec[j][i], th_max, th_min);
				T_PointType_output point_;
				point_.x = cloud_input_vec[j]->points[i].x;
				point_.y = cloud_input_vec[j]->points[i].y;
				point_.z = cloud_input_vec[j]->points[i].z;
				point_.r = color_vec[0];
				point_.g = color_vec[1];
				point_.b = color_vec[2];
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}

		//save
		string s_foldername = CTimeString::getTimeString() + "_Output_XYZRGB";
		CTimeString::makenewfolder(dir_, s_foldername);
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			string s_name;
			s_name = filenames_[j].substr(0, filenames_[j].size() - 4) + ".pcd";
			pcl::io::savePCDFile<T_PointType_output>(dir_ + "/" + s_foldername + "/" + s_name, *cloud_vec[j]);
		}

	}

	else if (i_method == EN_XYZI_fromXYZRGB)
	{
		typedef typename pcl::PointXYZRGB T_PointType_input;
		typedef typename pcl::PointXYZI T_PointType_output;

		string s_folder_input = "_XYZRGB";

		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder_input, filenames_, ".pcd");

		vector<pcl::PointCloud<T_PointType_input>::Ptr> cloud_input_vec;
		for (int j = 0; j < filenames_.size(); j++)
		{
			pcl::PointCloud<T_PointType_input>::Ptr cloud_(new pcl::PointCloud<T_PointType_input>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder_input + "/" + filenames_[j], *cloud_);
			cloud_->is_dense = true;
			cloud_input_vec.push_back(cloud_);
		}

		if (cloud_input_vec.size() == 0)
		{
			cout << "ERROR: no file found." << endl;
			throw std::runtime_error("ERROR: no file found.");
		}

		//choosing ch from RGB
		int num_showPoints = 3;
		if (cloud_input_vec[0]->size() < num_showPoints) num_showPoints = cloud_input_vec[0]->size();
		for (int j = 0; j < num_showPoints; j++)
		{
			T_PointType_input point_ = cloud_input_vec[0]->points[j];
			cout << "index:" << j;
			cout << " x:" << point_.x;
			cout << " y:" << point_.y;
			cout << " z:" << point_.z;
			cout << " r:" << point_.r;
			cout << " g:" << point_.g;
			cout << " b:" << point_.b << endl;
		}

		cout << endl;
		cout << "Choose ch  0:R  1:G  2:B" << endl;
		cout << "->";
		int i_ch;
		cin >> i_ch;

		vector<pcl::PointCloud<T_PointType_output>::Ptr> cloud_vec;
		for (int j = 0; j < cloud_input_vec.size(); j++)
		{
			pcl::PointCloud<T_PointType_output>::Ptr cloud_(new pcl::PointCloud<T_PointType_output>());
			for (int i = 0; i < cloud_input_vec[j]->size(); i++)
			{
				T_PointType_output point_;
				point_.x = cloud_input_vec[j]->points[i].x;
				point_.y = cloud_input_vec[j]->points[i].y;
				point_.z = cloud_input_vec[j]->points[i].z;
				if (i_ch == 0)
					point_.intensity = (float)((int)(cloud_input_vec[j]->points[i].r));
				else if (i_ch == 1)
					point_.intensity = (float)((int)(cloud_input_vec[j]->points[i].g));
				else if (i_ch == 2)
					point_.intensity = (float)((int)(cloud_input_vec[j]->points[i].b));
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}

		//save
		string s_foldername = CTimeString::getTimeString() + "_Output_XYZI";
		CTimeString::makenewfolder(dir_, s_foldername);
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			string s_name;
			s_name = filenames_[j].substr(0, filenames_[j].size() - 4) + ".pcd";
			pcl::io::savePCDFile<T_PointType_output>(dir_ + "/" + s_foldername + "/" + s_name, *cloud_vec[j]);
		}
	}

}

void CPointcloudGeneration::NarahaWinter202001(string dir_)
{
	enum
	{
		EN_PCDGeneration,
		EN_Filtering,
		EN_Combination
	};

	cout << EN_PCDGeneration << ": PCDGeneration" << endl;
	cout << EN_Filtering << ": Filtering" << endl;
	cout << EN_Combination << ": Combination of Velodyne and NIR" << endl;

	int i_method;
	cin >> i_method;

	if (i_method == EN_PCDGeneration)
		NarahaWinter202001_PCDGeneration(dir_);

	else if (i_method == EN_Filtering)
		NarahaWinter202001_Filtering(dir_);

	else if (EN_Combination)
		NarahaWinter202001_Combination(dir_);
}

void CPointcloudGeneration::NarahaWinter202001_PCDGeneration(string dir_)
{
	typedef typename pcl::PointXYZI T_PointType;

	enum
	{
		EN_VELO_PCAP,
		EN_VELO_withNIR,
		EN_NIR
	};

	cout << EN_VELO_PCAP << ": VELO_PCAP" << endl;
	cout << EN_VELO_withNIR << ": VELO_withNIR" << endl;
	cout << EN_NIR << ": NIR" << endl;
	cout << "->";

	int i_method;
	cin >> i_method;

	string s_folder_input = "_01Generation";

	//input
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_;
	if (i_method == EN_VELO_PCAP)
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder_input, filenames_, ").csv");
		for (int j = 0; j < filenames_.size(); j++)
		{
			vector<vector<string>> s_vecvec =
				CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder_input + "/" + filenames_[j], ",");
			pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
			for (int i = 0; i < s_vecvec.size(); i++)
			{
				if (i == 0) continue;
				T_PointType point_;
				point_.x = stod(s_vecvec[i][0]);
				point_.y = stod(s_vecvec[i][1]);
				point_.z = stod(s_vecvec[i][2]);
				point_.intensity = stod(s_vecvec[i][6]);
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}
	}
	else if (i_method == EN_VELO_withNIR)
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder_input, filenames_, "velo.csv");
		for (int j = 0; j < filenames_.size(); j++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
			vector<vector<string>> s_vecvec =
				CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder_input + "/" + filenames_[j], " ");
			for (int i = 0; i < s_vecvec.size(); i++)
			{
				T_PointType point_;
				point_.x = stod(s_vecvec[i][0]);
				point_.y = stod(s_vecvec[i][1]);
				point_.z = stod(s_vecvec[i][2]);
				point_.intensity = stod(s_vecvec[i][6]);
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}
	}
	else if (i_method == EN_NIR)
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder_input, filenames_, "nir.csv");
		for (int j = 0; j < filenames_.size(); j++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
			vector<vector<string>> s_vecvec =
				CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder_input + "/" + filenames_[j], " ");
			for (int i = 0; i < s_vecvec.size(); i++)
			{
				T_PointType point_;
				point_.x = stod(s_vecvec[i][0]);
				point_.y = stod(s_vecvec[i][1]);
				point_.z = stod(s_vecvec[i][2]);
				point_.intensity = stod(s_vecvec[i][3]);
				cloud_->push_back(point_);
			}
			cloud_->is_dense = true;
			cloud_vec.push_back(cloud_);
		}
	}
	if (cloud_vec.size() == 0)
	{
		cout << "ERROR: no file found." << endl;
		throw std::runtime_error("ERROR: no file found.");
	}

	//save
	string s_foldername;
	if (i_method == EN_VELO_PCAP)
	{
		s_foldername = s_folder_input + CTimeString::getTimeString() + "_Output_XYZI_VELO_PCAP";
	}
	else if (i_method == EN_VELO_withNIR)
	{
		s_foldername = s_folder_input + CTimeString::getTimeString() + "_Output_XYZI_VELO_withNIR";
	}
	else if (i_method == EN_NIR)
	{
		s_foldername = s_folder_input + CTimeString::getTimeString() + "_Output_XYZI_NIR";
	}
	CTimeString::makenewfolder(dir_, s_foldername);
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		string s_name;
		s_name = filenames_[j].substr(0, filenames_[j].size() - 4) + ".pcd";
		pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_foldername + "/" + s_name, *cloud_vec[j]);
	}

}

void CPointcloudGeneration::NarahaWinter202001_Filtering(string dir_)
{
	enum
	{
		EN_FilteringVelodyne,
		EN_FilteringNIR,
	};

	cout << EN_FilteringVelodyne << ": FilteringVelodyne" << endl;
	cout << EN_FilteringNIR << ": FilteringNIR" << endl;

	int i_method;
	cin >> i_method;

	double pitch_init;
	pitch_init = 24.4 * M_PI / 180.;

	//double th_VGF = 0.01;
	double th_VGF = 0.05;

	float Tolerance_out;
	int MinClusterSize_out;
	Tolerance_out = 1.;
	MinClusterSize_out = 100;
	int Meank_out;
	float StddevMulThresh_out;
	Meank_out = 50;
	StddevMulThresh_out = 0.1;

	bool b_transformToVertical = false;
	b_transformToVertical = true;

	bool b_removeGround = false;
	b_removeGround = true;

	bool b_rejectOutlier = false;
	b_rejectOutlier = true;

	double th_z_velo;
	//th_z_velo = -0.5;
	//th_z_velo = -0.3;
	//th_z_velo = -0.35;
	th_z_velo = -0.4;

	double th_x_nir, th_z_nir;
	th_x_nir = 3.;
	th_z_nir = -1.2;
	//th_x = 10.;
	//th_z = -100.;

	typedef typename pcl::PointXYZI T_PointType;

	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;

	Eigen::Affine3f Trans_ = Eigen::Affine3f::Identity();
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();

	//input
	vector<string> filenames_;
	string s_folder = "_02Filtering";
	if (i_method == EN_FilteringVelodyne)
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_, ".pcd");
		CTimeString::getFileNames_removeByExtension(filenames_, "nir");
	}
	else if (i_method == EN_FilteringNIR)
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_, "nir.pcd");
	}
	for (int j = 0; j < filenames_.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		cout << "reanding: " << filenames_[j] << endl;
		if (-1 == pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_[j], *cloud_))
		{
			cout<<"ERROR: pointcloud couldn't read.";
			throw std::runtime_error("ERROR: pointcloud couldn't read.");
		}
		cloud_->is_dense = true;
		cloud_vec.push_back(cloud_);
	}

	if (cloud_vec.size() == 0)
	{
		cout << "ERROR: no file found." << endl;
		throw std::runtime_error("ERROR: no file found.");
	}

	for (int j = 0; j < cloud_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[j], *cloud_);

		//voxel grid filter
		cout << "name:" << filenames_[j] << "  size:" << cloud_->size() << endl;
		cout << "VGF" << endl;
		pcl::ApproximateVoxelGrid<T_PointType> VGFilter;
		VGFilter.setInputCloud(cloud_);
		VGFilter.setLeafSize(th_VGF, th_VGF, th_VGF);
		VGFilter.filter(*cloud_);
		cout << "name:" << filenames_[j] << "  size:" << cloud_->size() << endl;

		//turn pitch(camera coordinate to robot one)
		if (b_transformToVertical)
		{
			HM_free = Eigen::Matrix4d::Identity();
			HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
			Trans_ = Eigen::Affine3f::Identity();
			Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
			pcl::transformPointCloud(*cloud_, *cloud_, Trans_);
		}

		if (b_removeGround)
		{
			if (i_method == EN_FilteringVelodyne)
			{
				for (int i = cloud_->size() - 1; i >= 0; i--)
				{
					T_PointType point_ = cloud_->points[i];
					if (th_z_velo > cloud_->points[i].z)
						cloud_->erase(cloud_->begin() + i);
				}

			}
			else if (i_method == EN_FilteringNIR)
			{
				for (int i = cloud_->size() - 1; i >= 0; i--)
				{
					T_PointType point_ = cloud_->points[i];
					if (point_.x > th_x_nir || point_.z < th_z_nir)
						cloud_->erase(cloud_->begin() + i);
				}
			}
		}

		if (b_rejectOutlier)
		{
			//rejectOutlier(cloud_, cloud_, Tolerance_out, MinClusterSize_out);
			remove_outliers(cloud_, cloud_, Meank_out, StddevMulThresh_out);
		}

		pcl::copyPointCloud(*cloud_, *cloud_vec[j]);
	}

	//output
	string s_folder_output = s_folder + CTimeString::getTimeString() + "_Output";
	CTimeString::makenewfolder(dir_, s_folder_output);
	for (int j = 0; j < cloud_vec.size(); j++)
		pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_folder_output + "/" + filenames_[j], *cloud_vec[j]);
}

void CPointcloudGeneration::NarahaWinter202001_Combination(string dir_)
{

	enum
	{
		EN_VELO,
		EN_VELONIR,
	};

	cout << EN_VELO << ": Combine Velodyne only" << endl;
	cout << EN_VELONIR << ": Combine Velodyne and NIR" << endl;
	cout << "->";

	int i_method;
	cin >> i_method;

	typedef typename pcl::PointXYZI T_PointType_input;
	typedef typename pcl::PointXYZRGB T_PointType;

	bool b_useRemovingGroundPointcloud = false;
	b_useRemovingGroundPointcloud = true;

	string s_folder = "_03Combination";
	string s_folder_output = s_folder +  "_Output";

	//delete check
	{
		vector<string> temp_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder_output, temp_, ".pcd");
		if (temp_.size() != 0)
		{
			cout << "some .pcd already exist" << endl;
			cout << "select delete them or not  1:yes  0:no" << endl;
			bool b_delete = true;
			cin >> b_delete;
			if (b_delete)
				FileProcess_delete(dir_ + "/" + s_folder_output);
		}
	}

	//input
	vector<string> filenames_velo;
	vector<string> filenames_nir;
	vector<pcl::PointCloud<T_PointType_input>::Ptr> cloud_velo_vec;
	vector<pcl::PointCloud<T_PointType_input>::Ptr> cloud_nir_vec;
	if (i_method == EN_VELO)
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_velo, ").pcd");
		for (int j = 0; j < filenames_velo.size(); j++)
		{
			pcl::PointCloud<T_PointType_input>::Ptr cloud_(new pcl::PointCloud<T_PointType_input>());
			if (-1 == pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_velo[j], *cloud_))
			{
				cout << "ERROR: pointcloud couldn't read." << endl;
				break;
			}
			cloud_->is_dense = true;
			cloud_velo_vec.push_back(cloud_);
		}
	}
	else if (i_method == EN_VELONIR)
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_velo, "velo.pcd");
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_nir, "nir.pcd");
		for (int j = 0; j < filenames_velo.size(); j++)
		{
			pcl::PointCloud<T_PointType_input>::Ptr cloud_(new pcl::PointCloud<T_PointType_input>());
			if (-1 == pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_velo[j], *cloud_))
			{
				cout << "ERROR: pointcloud couldn't read." << endl;
				break;
			}
			cloud_->is_dense = true;
			cloud_velo_vec.push_back(cloud_);
		}
		for (int j = 0; j < filenames_nir.size(); j++)
		{
			pcl::PointCloud<T_PointType_input>::Ptr cloud_(new pcl::PointCloud<T_PointType_input>());
			if (-1 == pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_nir[j], *cloud_))
			{
				cout << "ERROR: pointcloud couldn't read." << endl;
				break;
			}
			cloud_->is_dense = true;
			cloud_nir_vec.push_back(cloud_);
		}

	}

	//combination
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	for (int j = 0; j < cloud_velo_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		cloud_->clear();

		cout << "j:" << j << " calc " << filenames_velo[j] << "..." << endl;
		for (int i = 0; i < cloud_velo_vec[j]->size(); i++)
		{
			T_PointType point_;
			point_.x = cloud_velo_vec[j]->points[i].x;
			point_.y = cloud_velo_vec[j]->points[i].y;
			point_.z = cloud_velo_vec[j]->points[i].z;
			point_.r = 255;
			point_.g = (unsigned char)((int)cloud_velo_vec[j]->points[i].intensity);
			point_.b = 0;
			cloud_->push_back(point_);
		}

		cloud_vec.push_back(cloud_);
	}
	if (i_method == EN_VELONIR)
	{
		for (int j = 0; j < cloud_nir_vec.size(); j++)
		{
			cout << "j:" << j << " calc " << filenames_nir[j] << "..." << endl;
			for (int i = 0; i < cloud_nir_vec[j]->size(); i++)
			{
				T_PointType point_;
				point_.x = cloud_nir_vec[j]->points[i].x;
				point_.y = cloud_nir_vec[j]->points[i].y;
				point_.z = cloud_nir_vec[j]->points[i].z;
				point_.r = (unsigned char)((int)cloud_nir_vec[j]->points[i].intensity);
				point_.g = 255;
				point_.b = 0;
				cloud_vec[j]->push_back(point_);
			}
		}
	}

	//save
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		string filename = filenames_velo[j].substr(0, 3) + "XYZRGB_naraha.pcd";
		pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_folder_output + "/" + filename, *cloud_vec[j]);
	}

}

void CPointcloudGeneration::getCSVFromPointCloud()
{
	cout << "start .csv method" << endl;
	string dir_;
	dir_ = "../../data/process04_CSV_FromPointCloud";
	vector<string> filenames_;
	//CTimeString::getFileNames_extension(file_dir, filenames_, "nir.pcd");
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (int index_ = 0; index_ < filenames_.size(); index_++)
	{
		cout << "reanding: " << filenames_[index_] << endl;
		cloud_->clear();
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[index_], *cloud_);
		vector<vector<double>> data_vec_vec;
		for (int i = 0; i < cloud_->size(); i++)
		{
			vector<double> data_vec;
			data_vec.push_back(cloud_->points[i].x);
			data_vec.push_back(cloud_->points[i].y);
			data_vec.push_back(cloud_->points[i].z);
			//data_vec.push_back(cloud_->points[i].intensity);
			//data_vec.push_back(cloud_->points[i].r);
			data_vec.push_back(cloud_->points[i].g);
			data_vec_vec.push_back(data_vec);
		}
		string filename_save = filenames_[index_].substr(0, filenames_[index_].size() - 4) + "_csv.csv";

		CTimeString::getCSVFromVecVec(data_vec_vec, dir_ + "/_csv/" + filename_save);
	}

	cout << "finish .csv method" << endl;
}

void CPointcloudGeneration::DoSegmentation()
{
	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	string dir_ = "../../data/process08_DoSegmentation";

	float Tolerance;
	cout << "input : th_tolerance" << endl;
	cout << "->";
	cin >> Tolerance;

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	for (int i = 0; i < filenames_.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>);
		pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
		cloud_vec.push_back(cloud);
	}

	CPointVisualization<T_PointType> pv;
	pv.setWindowName("cluster");

	for (int idx = 0; idx < cloud_vec.size(); idx++)
	{
		cout << "i:" << idx;
		cout << " " << filenames_[idx];
		cout << " size:" << cloud_vec[idx]->size() << endl;

		vector <pcl::PointCloud<T_PointType>::Ptr> cloud_cluster_vec;
		pcl::PointCloud<T_PointType>::Ptr cloud_rest(new pcl::PointCloud<T_PointType>);
		cloud_cluster_vec = getSegmentation_robust(cloud_vec[idx], cloud_rest, Tolerance, 100);

		cout << "cloud_cluster_vec.size(): " << cloud_cluster_vec.size() << endl;

		//getRGBwithValuebyHSV
		for (int j = 0; j < cloud_cluster_vec.size(); j++)
		{
			vector<std::uint8_t> color_vec;
			//color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(j, cloud_cluster_vec.size() - 1, 0);
			color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(j, cloud_cluster_vec.size() - 1, 0);
			//cout << "j:" << j << " r:" << (int)color_vec[0] << " g:" << (int)color_vec[1] << " b:" << (int)color_vec[2] << endl;
			for (int i = 0; i < cloud_cluster_vec[j]->size(); i++)
			{
				cloud_cluster_vec[j]->points[i].r = color_vec[0];
				cloud_cluster_vec[j]->points[i].g = color_vec[1];
				cloud_cluster_vec[j]->points[i].b = color_vec[2];
			}
		}

		pcl::PointCloud<T_PointType>::Ptr cloud_sum(new pcl::PointCloud<T_PointType>);
		for (int i = 0; i < cloud_cluster_vec.size(); i++)
		{
			*cloud_sum += *cloud_cluster_vec[i];
		}

		cout << "cloud_sum size:" << cloud_sum->size() << endl;
		cout << "cloud_rest size:" << cloud_rest->size() << endl;

		pv.setPointCloud(cloud_sum);

		while (1)
		{
			pv.updateViewer();
			if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
		}

		pv.setPointCloud(cloud_rest);
		while (1)
		{
			pv.updateViewer();
			if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
		}

	}

	pv.closeViewer();
}

void CPointcloudGeneration::DoOutlierRejector()
{
	string dir_;
	dir_ = "../../data";
	vector<string> dir_folder_vec;
	FileProcess_FolderInFolder(dir_, dir_folder_vec);

	int i_select;
	cout << "select folder (index) ->";
	cin >> i_select;
	cout << i_select << "(" << dir_folder_vec[i_select] << ")" << endl;
	dir_ = dir_ + "/" + dir_folder_vec[i_select];
	cout << endl;

	//typedef typename pcl::PointXYZI PointType_func;
	typedef typename pcl::PointXYZRGB PointType_func;

	bool b_plane = false;
	//b_plane = true;

	CPointVisualization<PointType_func> pv;
	if (typeid(PointType_func) == typeid(pcl::PointXYZI))
		pv.setWindowName("show XYZI");
	else if (typeid(PointType_func) == typeid(pcl::PointXYZRGB))
		pv.setWindowName("show XYZRGB");
	else
		throw std::runtime_error("This PointType is unsupported.");

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");
	cout << "file size: " << filenames_.size() << endl;

	if (filenames_.size() == 0)
	{
		cout << "ERROR: no file found" << endl;
		return;
	}

	vector< pcl::PointCloud<PointType_func>::Ptr> cloud_vec;

	pcl::PointCloud<PointType_func>::Ptr cloud_(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_temp(new pcl::PointCloud<PointType_func>());

	//parameter of outlier rejector
	float Tolerance_out;
	int MinClusterSize_out;
	Tolerance_out = 1.;
	MinClusterSize_out = 100;
	int Meank_out;
	float StddevMulThresh_out;
	Meank_out = 50;
	StddevMulThresh_out = 0.1;

	int index_ = 0;
	cout << "Press space to next" << endl;
	while (1)
	{
		//short key_num = GetAsyncKeyState(VK_SPACE);
		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1)
		{
			if (index_ >= 2 * (filenames_.size()))
			{
				cout << "index over" << endl;
				break;
			}

			if (index_ % 2 == 0)
			{
				int index_temp = (int)(index_ / 2);
				//read file
				cout << "index_: " << index_temp << endl;
				pcl::io::loadPCDFile(dir_ + "/" + filenames_[index_temp], *cloud_);
				cout << "showing:" << filenames_[index_temp] << " size:" << cloud_->size() << endl;
			}
			else
			{
				//use outlier rejector
				cout << "use outlier rejector" << endl;
				remove_outliers(cloud_, cloud_, Meank_out, StddevMulThresh_out);
			}

			//remove ground plane
			if (cloud_->size() != 0 && b_plane)
			{
				detectPlane<PointType_func>(*cloud_, 0.05, true, true);	//velo
				//detectPlane<PointType_func>(*cloud_, 0.01, true, true);	//nir
			}
			pv.setPointCloud(cloud_);
			index_++;
		}

		//escape
		//short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) {
			cout << "toggled!" << endl;
			break;
		}
		pv.updateViewer();
	}

	pv.closeViewer();

}

void CPointcloudGeneration::ThermalCloudGeneration(string dir_)
{
	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".csv");
	typedef typename pcl::PointXYZRGB T_PointType;
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;

	for (int j = 0; j < filenames_.size(); j++)
	{
		cout << "j:" << j << endl;
		vector<vector<string>> data_vec_vec_string;
		data_vec_vec_string = CTimeString::getVecVecFromCSV_string(dir_ + "/" + filenames_[j], " ");
		vector<vector<double>> pc_vecvec;
		for (int i = 0; i < data_vec_vec_string.size(); i++)
		{
			vector<double> pc_vec;
			pc_vec.push_back(stod(data_vec_vec_string[i][0]));
			pc_vec.push_back(stod(data_vec_vec_string[i][1]));
			pc_vec.push_back(stod(data_vec_vec_string[i][2]));
			pc_vec.push_back(stod(data_vec_vec_string[i][3]));
			pc_vec.push_back(stod(data_vec_vec_string[i][4]));
			pc_vecvec.push_back(pc_vec);
		}

		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		cloud_->clear();
		for (int i = 0; i < pc_vecvec.size(); i++)
		{
			T_PointType point_;
			point_.x = pc_vecvec[i][0];
			point_.y = pc_vecvec[i][1];
			point_.z = pc_vecvec[i][2];
			point_.r = pc_vecvec[i][3];
			point_.g = pc_vecvec[i][4];
			point_.b = 0;
			cloud_->push_back(point_);
		}
		cloud_->is_dense = true;
		cout << "cloud_->size():" << cloud_->size() << endl;
		cloud_vec.push_back(cloud_);
		cout << endl;
	}

	if (cloud_vec.size() == 0)
	{
		cout << "ERROR: no file found." << endl;
		throw std::runtime_error("ERROR: no file found.");
	}

	//save
	string s_foldername = CTimeString::getTimeString() + "_Output_samo";
	CTimeString::makenewfolder(dir_, s_foldername);
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		string s_name;
		s_name = filenames_[j].substr(0, filenames_[j].size() - 4) + ".pcd";
		cout << "saving cloud[" << j << "]..." << endl;
		pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_foldername + "/" + s_name, *cloud_vec[j]);
	}

}
