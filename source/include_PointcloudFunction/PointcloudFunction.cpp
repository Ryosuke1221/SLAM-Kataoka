#include "PointcloudFunction.h"

void CPointcloudFunction::all_process()
{
	//moveFile();

	int WhichProcess = 0;
	string filename1, filename2;
	bool b_finish = false;
	enum OPTION {
		EN_escape = 0,
		EN_FreeSpace,
		EN_FileProcess,
		EN_SequentShow,
		EN_handregistration,
		EN_GetPcdFromCSV,
		EN_FilterPointCloud,
		EN_CombinePointCloud,
		EN_CSV_FromPointCloud,
		EN_DynamicTranslation,
		EN_DrawTrajectory,
		EN_Segmentation,
		EN_GR_FPFH_SAC_IA,
		EN_DoOutlierRejector,
		EN_ICP_Proposed_AllFrames,
		EN_Evaluate_ICP_property_Optimization,
		EN_DoMappingFromTrajectory,
		EN_DoDifferential
	};

	while (!b_finish)
	{
		cout << endl;
		cout << "please input process number" << endl;
		cout << " " << EN_escape << ": escape" << endl;
		cout << " " << EN_FreeSpace << ": free space" << endl;
		cout << " " << EN_FileProcess << ": FileProcess" << endl;
		cout << " " << EN_SequentShow << ": sequent show" << endl;
		cout << " " << EN_handregistration << ": hand registration" << endl;
		cout << " " << EN_GetPcdFromCSV << ": get .pcd from .csv" << endl;
		cout << " " << EN_FilterPointCloud << ": filter PointCloud_naraha" << endl;
		cout << " " << EN_CombinePointCloud << ": CombinePointCloud" << endl;
		cout << " " << EN_CSV_FromPointCloud << ": CSV_FromPointCloud" << endl;
		cout << " " << EN_DynamicTranslation << ": DynamicTranslation" << endl;
		cout << " " << EN_DrawTrajectory << ": DrawTrajectory" << endl;
		cout << " " << EN_Segmentation << ": Segmentation" << endl;
		cout << " " << EN_GR_FPFH_SAC_IA << ": GR_FPFH_SAC_IA" << endl;
		cout << " " << EN_DoOutlierRejector << ": DoOutlierRejector" << endl;
		cout << " " << EN_ICP_Proposed_AllFrames << ": ICP_Proposed_AllFrames" << endl;
		cout << " " << EN_Evaluate_ICP_property_Optimization << ": Evaluate_ICP_property_Optimization" << endl;
		cout << " " << EN_DoMappingFromTrajectory << ": DoMappingFromTrajectory" << endl;
		cout << " " << EN_DoDifferential << ": DoDifferential" << endl;

		cout <<"WhichProcess: ";
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
			FreeSpace();
			break;

		case EN_FileProcess:
			FileProcess();
			break;

		case EN_SequentShow:
			show_sequent();
			break;

		case EN_handregistration:
			HandRegistration();
			break;

		case EN_GetPcdFromCSV:
			getPCDFromCSV_naraha();
			break;

		case EN_FilterPointCloud:
			filterNIRPointCloud_naraha();
			break;

		case EN_CSV_FromPointCloud:
			getCSVFromPointCloud();
			break;

		case EN_CombinePointCloud:
			combinePointCloud_naraha();
			break;

		case EN_DynamicTranslation:
			DynamicTranslation();
			break;

		case EN_DrawTrajectory:
			DrawTrajectory();
			break;

		case EN_Segmentation:
			DoSegmentation();
			break;
			
		case EN_GR_FPFH_SAC_IA:
			GlobalRegistration_FPFH_SAC_IA();
			break;

		case EN_DoOutlierRejector:
			DoOutlierRejector();
			break;

		case EN_ICP_Proposed_AllFrames:
			DoICP_proposed_AllFrames();
			break;

		case EN_Evaluate_ICP_property_Optimization:
			DoEvaluation_ICP_property();
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory();
			break;

		case EN_DoDifferential:
			DoDifferential();
			break;

		default:
			break;
		}
	
	}

}

void CPointcloudFunction::show_sequent()
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

	typedef typename pcl::PointXYZI PointType_func;
	//typedef typename pcl::PointXYZRGB PointType_func;

	bool b_useTXT = false;
	//b_useTXT = true;

	bool b_plane = false;
	//b_plane = true;

	bool b_onlyConvergence = false;
	//b_onlyConvergence = true;
	{
		int i_find = dir_.find("GR_FPFH_SAC_IA/2020");
		if (i_find == std::string::npos) b_onlyConvergence = false;
		else b_onlyConvergence = true;
	}

	bool b_select = false;
	//b_select = true;

	bool b_normal = false;
	//b_normal = true;

	CPointVisualization<PointType_func> pv;
	if (typeid(PointType_func) == typeid(pcl::PointXYZI))
		pv.setWindowName("show XYZI");
	else if (typeid(PointType_func) == typeid(pcl::PointXYZRGB))
		pv.setWindowName("show XYZRGB");
	else
		throw std::runtime_error("This PointType is unsupported.");

	//show normal
	if (b_normal)
	{
		cout << "input: normal radius" << endl;
		cout << "->";
		float radius_normal;
		cin >> radius_normal;
		pv.useNormal(radius_normal, 10, 0.1);
	}

	vector<string> filenames_;
	if(b_onlyConvergence) CTimeString::getFileNames_extension(dir_, filenames_, "C1_XYZRGB.pcd");
	else CTimeString::getFileNames_extension(dir_, filenames_,".pcd");
	cout << "file size: " << filenames_.size() << endl;

	if (filenames_.size() == 0)
	{
		cout << "no pointcloud found" << endl;
		pv.closeViewer();
		return;
	}

	//ignore some files
	if(b_select)
	{
		bool b_showAll = true;
		cout << "select: Do you watch all .pcd or not?  Yes:1  No:0" << endl;
		cout << "->";
		cin >> b_showAll;
		if (!b_showAll)
		{
			for (int i = 0; i < filenames_.size(); i++)
			{
				string s_i = to_string(i);
				if (s_i.size() < 3) s_i = " " + s_i;
				if (s_i.size() < 3) s_i = " " + s_i;
				cout << "i:" << s_i << " " << filenames_[i] << endl;

			}
			cout << endl;
			cout << "select(index): files you watch and separeted with spaces" << endl;
			cout << "->";
			vector<string> s_input_vec;
			s_input_vec = CTimeString::inputSomeString();
			vector<string> filenames_temp;
			filenames_temp = filenames_;
			filenames_.clear();
			for (int i = 0; i < s_input_vec.size(); i++)
			{
				int index_temp = stoi(s_input_vec[i]);
				if (!(0 <= index_temp && index_temp < filenames_temp.size()))
				{
					cout << "ERROR: ignored invalud:" << index_temp << endl;
					continue;
				}
				filenames_.push_back(filenames_temp[index_temp]);
				cout << "filenames_.back():" << filenames_.back() << endl;
			}

			if (filenames_.size() == 0) filenames_ = filenames_temp;
		}

	}

	if (filenames_.size() == 0)
	{
		cout << "ERROR: no file found" << endl;
		return ;
	}

	pcl::PointCloud<PointType_func>::Ptr cloud_(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_temp(new pcl::PointCloud<PointType_func>());

	int index_ = 0;

	vector<string> filename_use;
	cout << "Press space to next" << endl;
	while (1)
	{
		//short key_num = GetAsyncKeyState(VK_SPACE);
		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1)
		{
			if (index_ == filenames_.size())
			{
				cout << "index over" << endl;
				break;
			}
			cout << "index_: " << index_ << endl;
			//cout << "reading:" << filenames_[index_] << endl;
			pcl::io::loadPCDFile(dir_ + "/" + filenames_[index_], *cloud_);
			cout << "showing:" << filenames_[index_] <<" size:" << cloud_->size() << endl;
			//pv.setPointCloud(cloud_);
			pv.setPointCloud(cloud_, filenames_[index_]);

			//remove ground plane
			if (cloud_->size() != 0 && b_plane)
			{
				detectPlane<PointType_func>(*cloud_, 0.05, true, true);	//velo
				//detectPlane<PointType_func>(*cloud_, 0.01, true, true);	//nir
			}
			index_++;
		}
		//save
		if ((GetAsyncKeyState(VK_RETURN) & 1) == 1 && b_useTXT)
		{
			filename_use.push_back(filenames_[index_ - 1]);
			cout << "add: " << filenames_[index_ - 1] << endl;
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

	for (int i = 0; i < filename_use.size(); i++)
	{
		//cout << "file " << i << ": " << filename_use[i] << endl;
		cout<< filename_use[i] << endl;
	}

	vector<vector<string>> save_vec_vec;
	for (int i = 0; i < filename_use.size(); i++)
	{
		vector<string> save_vec;
		save_vec.push_back(filename_use[i]);
		save_vec_vec.push_back(save_vec);
	}
	if (b_useTXT)
		CTimeString::getCSVFromVecVec(save_vec_vec, dir_ + "/_usePointCloud.csv");

}

void CPointcloudFunction::getPCDFromCSV_gotFromPCAP(string dir_save, string dir_data, string file_RelativePath_)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	vector<vector<string>> csv_vec_vec_string;
	csv_vec_vec_string = CTimeString::getVecVecFromCSV_string(dir_data + "/" + file_RelativePath_);
	cloud_->clear();
	for (int j = 0; j < csv_vec_vec_string.size(); j++)
	{
		if (j == 0) continue;
		pcl::PointXYZI point_;
		point_.x = stod(csv_vec_vec_string[j][0]);
		point_.y = stod(csv_vec_vec_string[j][1]);
		point_.z = stod(csv_vec_vec_string[j][2]);
		point_.intensity = stod(csv_vec_vec_string[j][6]);
		cloud_->push_back(point_);

	}
	string filename_ = file_RelativePath_.substr(0, file_RelativePath_.size() - 4) + ".pcd";
	pcl::io::savePCDFile<pcl::PointXYZI>(dir_save + "/" + filename_, *cloud_);
	cout << "saved: " << filename_ << endl;
}


void CPointcloudFunction::getPCDFromCSV_naraha()
{
	string file_dir;
	//file_dir = "../../data/temp/02 velo&nir all frame";
	//file_dir = "../../data/temp";
	file_dir = "../../data/process02_GetPcdFromCSV";

	string s_csv = "02 velo&nir all frame";

	{
		vector<string> temp_;
		CTimeString::getFileNames_extension(file_dir,temp_,"pcd");
		if (temp_.size() != 0)
		{
			bool b_delete = true;
			cout << "some .pcd already exist" << endl;
			cout << "select delete them or not  1:yes  0:no" << endl;
			cin >> b_delete;
			if (b_delete)
			{
				FileProcess_delete(file_dir);
			}
		}
	}
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	int i_select;

	//get filenames
	vector<string> filenames;

	cout << "select which .csv data is" << endl;
	cout << "0: velodyne(no NIR), 1:velodyne(NIR), 2:NIR sensor(NIR)" << endl;
	cout << "i_select:";
	cin >> i_select;

	switch (i_select)
	{
	case 0:
		CTimeString::getFileNames_extension(file_dir + "/" + s_csv, filenames, ").csv");
		for (int i = 0; i < filenames.size(); i++)
		{
			cout << "i:" << i << " calc " << file_dir + "/" + s_csv + "/" + filenames[i] << "..." << endl;
			getPCDFromCSV_gotFromPCAP(file_dir, file_dir + "/" + s_csv, filenames[i]);
		}

		break;
	case 1:
		CTimeString::getFileNames_extension(file_dir + "/" + s_csv, filenames, "velo.csv");
		for (int i = 0; i < filenames.size(); i++)
		{
			cout << "i:" << i << " calc " << file_dir + "/" + s_csv + "/" + filenames[i] << "..." << endl;
			vector<vector<string>> csv_vec_vec_string;
			csv_vec_vec_string = CTimeString::getVecVecFromCSV_string(file_dir + "/" + s_csv + "/" + filenames[i], " ");
			//PointCloud
			cloud_->clear();
			for (int j = 0; j < csv_vec_vec_string.size(); j++)
			{
				pcl::PointXYZI point_;
				point_.x = stod(csv_vec_vec_string[j][0]);
				point_.y = stod(csv_vec_vec_string[j][1]);
				point_.z = stod(csv_vec_vec_string[j][2]);
				point_.intensity = stod(csv_vec_vec_string[j][6]);
				cloud_->push_back(point_);
			}
			string filename_ = filenames[i].substr(0, filenames[i].size() - 4) + ".pcd";
			pcl::io::savePCDFile<pcl::PointXYZI>(file_dir + "/" + filename_, *cloud_);
			cout << "saved: " << filename_ << endl;
		}

		break;
	case 2:
		CTimeString::getFileNames_extension(file_dir + "/" + s_csv, filenames, "nir.csv");
		float max_, min_;
		min_ = 255.;
		max_ = 0.;

		for (int i = 0; i < filenames.size(); i++)
		{
			cout << "i:" << i << " calc " << file_dir + "/" + s_csv + "/" + filenames[i] << "..." << endl;
			vector<vector<string>> csv_vec_vec_string;
			csv_vec_vec_string = CTimeString::getVecVecFromCSV_string(file_dir + "/" + s_csv + "/" + filenames[i], " ");
			//PointCloud
			cloud_->clear();
			for (int j = 0; j < csv_vec_vec_string.size(); j++)
			{
				pcl::PointXYZI point_;
				point_.x = stod(csv_vec_vec_string[j][0]);
				point_.y = stod(csv_vec_vec_string[j][1]);
				point_.z = stod(csv_vec_vec_string[j][2]);
				double value_ = stod(csv_vec_vec_string[j][3]);
				point_.intensity = value_;
				cloud_->push_back(point_);
				if (max_ < value_) max_ = value_;
				if (min_ > value_) min_ = value_;
			}
			string filename_ = filenames[i].substr(0, filenames[i].size() - 4) + ".pcd";
			pcl::io::savePCDFile<pcl::PointXYZI>(file_dir + "/" + filename_, *cloud_);
			cout << "saved: " << filename_ << endl;
		}
		cout << "min_ = " << min_ << endl;
		cout << "max_ = " << max_ << endl;
		//0~255
		break;
	}
}

void CPointcloudFunction::FreeSpace()
{
	int i_method;
	//i_method = 0;
	i_method = 1;

	if (i_method == 0)
	{
		typedef typename pcl::FPFHSignature33 FeatureT;
		typedef typename pcl::PointXYZRGB T_PointType;

		int num_nearest = 10;
		//num_nearest = 1;

		string dir_ = "../../data";

		string s_file_0 = "000XYZRGB_naraha.pcd";
		string s_file_1 = "001XYZRGB_naraha.pcd";

		float voxel_size;
		voxel_size = 0.1;

		float radius_normal_FPFH, radius_FPFH;
		radius_normal_FPFH = 0.5;
		radius_FPFH = 1.;


		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile(dir_ + "/" + s_file_0, *cloud_tgt);
		pcl::io::loadPCDFile(dir_ + "/" + s_file_1, *cloud_src);
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

		pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr feature_tree_(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
		feature_tree_->setInputCloud(fpfh_tgt);

		//index_near_vecvec = CFPFH_PCL::getNearestOfFPFH(fpfh_src, num_nearest, feature_tree_, squaredDistance_near_vecvec);
		pcl::Correspondences corrs_;
		corrs_ = CFPFH_PCL::getNearestOfFPFH(fpfh_src, num_nearest, feature_tree_);

		for (int j = 0; j < corrs_.size(); j++)
		{
			if (j % 100 != 0) continue;
			cout << "j:" << j << "  query:" << corrs_[j].index_query << " match:" << corrs_[j].index_match << " distance:" << corrs_[j].distance << endl;
		}

	}

	else if (i_method == 1)
	{
		typedef typename pcl::PointXYZRGB T_PointType;

		string dir_ = "../../data";

		string s_file_0 = "000XYZRGB_naraha.pcd";
		string s_file_1 = "001XYZRGB_naraha.pcd";

		float voxel_size;
		voxel_size = 0.1;

		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile(dir_ + "/" + s_file_0, *cloud_tgt);
		pcl::io::loadPCDFile(dir_ + "/" + s_file_1, *cloud_src);
		cloud_tgt->is_dense = true;
		cloud_src->is_dense = true;

		Eigen::Matrix<float, 3, Eigen::Dynamic> mat_(3, cloud_tgt->size());
		mat_ = CKataokaPCL::calcEigenMatrixFromPointCloud(cloud_tgt);
		Eigen::Matrix<float, 3, 3> mat_cov = CKataokaPCL::calcCovarianceMatrix(mat_);
		cout << "show mat_cov" << endl;
		cout << mat_cov << endl;

	}

}

void CPointcloudFunction::filterNIRPointCloud_naraha()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::ApproximateVoxelGrid<pcl::PointXYZI> VGFilter;
	string dir_ = "../../data/process03_FilterPointCloud";
	string s_subdir = "_before";
	//double th_VGF = 0.01;
	double th_VGF = 0.05;

	{
		vector<string> temp_;
		CTimeString::getFileNames_extension(dir_, temp_, "pcd");
		if (temp_.size() != 0)
		{
			bool b_delete = true;
			cout << "some .pcd already exist" << endl;
			cout << "select delete them or not  1:yes  0:no" << endl;
			cin >> b_delete;
			if (b_delete)
			{
				FileProcess_delete(dir_);
			}
		}
	}


	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();

	//int index_PC_now = 0;
	//bool b_makeNewPC = true;
	//bool b_escaped = false;
	//bool b_break = false;

	bool b_RemoveGround = true;
	bool b_nir = true;

	vector<string> filenames_;
	//CTimeString::getFileNames_extension(foldername_, filenames_, "nir.pcd");
	CTimeString::getFileNames_extension(dir_ + "/" + s_subdir, filenames_, "nir.pcd");

	double pitch_init;
	pitch_init = 24.4 * M_PI / 180.;

	//area filter
	for (int index_ = 0; index_ < filenames_.size(); index_++)
	{
		cout << "reanding: " << filenames_[index_] << endl;
		if (-1 == pcl::io::loadPCDFile(dir_ + "/" + s_subdir + "/" + filenames_[index_], *cloud_))
		{
			cout << "ERROR: pointcloud couldn't read." << endl;
			break;
		}

		//turn pitch(camera coordinate to robot one)
		HM_free = Eigen::Matrix4d::Identity();
		HM_free = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
		Trans_ = Eigen::Affine3f::Identity();
		Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_free);
		pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

		//range
		if (b_nir)
		{
			bool b_modify = false;
			b_modify = true;
			double th_x, th_z;
			th_x = 3.;
			th_z = -1.2;
			//th_x = 10.;
			//th_z = -100.;
			cloud_temp->clear();
			pcl::copyPointCloud(*cloud_, *cloud_temp);
			cloud_->clear();
			for (int i = 0; i < cloud_temp->size(); i++)
			{
				pcl::PointXYZI point_ = cloud_temp->points[i];
				if (point_.x > th_x && b_modify) continue;
				if (point_.z < th_z && b_modify) continue;
				cloud_->push_back(point_);
			}
		}

		//voxel grid filter
		cout << "PC(" << index_ << ") number :" << cloud_->size() << endl;
		cout << "VGF" << endl;
		VGFilter.setInputCloud(cloud_);
		VGFilter.setLeafSize(th_VGF, th_VGF, th_VGF);
		VGFilter.filter(*cloud_);
		cout << "PC(" << index_ << ") number :" << cloud_->size() << endl;

		//-turn pitch(camera coordinate)
		HM_free = Eigen::Matrix4d::Identity();
		HM_free = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., -pitch_init, 0.);
		Trans_ = Eigen::Affine3f::Identity();
		Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_free);
		pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

		string filename_save = filenames_[index_].substr(0, filenames_[index_].size() - 4) + "_filtered_nir.pcd";
		pcl::io::savePCDFile<pcl::PointXYZI>(dir_ + "/" + filename_save, *cloud_);

	}
}

void CPointcloudFunction::getCSVFromPointCloud()
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

void CPointcloudFunction::HandRegistration()
{
	//"delta_T(i) = T(i-1).inverse() * T(i);",
	//because relative displacement can accumulate correctly.
	//When delta_T(i-1) changed, delta_T(i) should be able to work.

	//cout << "HandRegistration started!" << endl;
	//Sleep(1 * 1000);

	string dir_;
	//dir_ = "../../data/temp/_Hand";
	dir_ = "../../data/process01_handregistration";

	bool b_RemoveGround = true;
	//b_RemoveGround = false;


	//typedef typename pcl::PointXYZI PointType_func;
	typedef typename pcl::PointXYZRGB PointType_func;


	CPointVisualization<PointType_func> pv;
	CPointVisualization<PointType_func> pv_2frame;

	if (typeid(PointType_func) == typeid(pcl::PointXYZI))
		pv.setWindowName("show XYZI");
	else if (typeid(PointType_func) == typeid(pcl::PointXYZRGB))
		pv.setWindowName("show XYZRGB");
	else
		throw std::runtime_error("This PointType is unsupported.");

	pv_2frame.setWindowName("show 2 frame");

	pcl::PointCloud<PointType_func>::Ptr cloud_show(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_show_static(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_moving(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_moving_init(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_temp(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_before(new pcl::PointCloud<PointType_func>());//final state in frame before now one
	pcl::PointCloud<PointType_func>::Ptr cloud_show_2frame(new pcl::PointCloud<PointType_func>());

	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d HM_Trans_now = Eigen::Matrix4d::Identity();

	double resolution_translation = 0.;
	double resolution_rotation = 0.;
	resolution_translation = 0.05;
	resolution_rotation = 0.5 * M_PI / 180.;

	int index_PC_now = 0;
	bool b_makeNewPC = true;
	bool b_first = true;
	bool b_escaped = false;
	bool b_break = false;

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	std::string filename_txt;

	vector<Eigen::Matrix4d>	HM_displacement_vec;

	CKataokaPCL kataokaPCL;
	int MaximumIterations;
	double MaxCorrespondenceDistance, EuclideanFitnessEpsilon, TransformationEpsilon;
	kataokaPCL.setMothodInt(0);
	MaximumIterations = 50000;
	MaxCorrespondenceDistance = 1.;
	EuclideanFitnessEpsilon = 1e-5;
	TransformationEpsilon = 1e-6;

	//read txt (initial trajectory)
	{
		vector<Eigen::Vector6d> trajectory_vec_vec;

		//input txt
		vector<string> filenames__txt;
		CTimeString::getFileNames_extension(dir_, filenames__txt, ".csv");

		if (filenames__txt.size() == 0)
		{
			cout << "found no txt file and make it." << endl;

			//generation
			filename_txt = dir_ + "/transformation.txt";
			for (int i = 0; i < filenames_.size(); i++)
			{
				Eigen::Vector6d trajectory_vec = Eigen::Vector6d::Zero();
				trajectory_vec_vec.push_back(trajectory_vec);
			}
		}

		else if (filenames__txt.size() == 1)
		{

			//trajectory_vec_vec = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames__txt[0]);
			vector<vector<double>> trajectory_temp;
			cout << "filename:" << dir_ + "/" + filenames__txt[0] << endl;
			trajectory_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames__txt[0]);
			for (int i = 0; i < trajectory_temp.size(); i++)
			{
				Eigen::Vector6d trajectory_vec = Eigen::Vector6d::Zero();
				trajectory_vec <<
					trajectory_temp[i][1],
					trajectory_temp[i][2],
					trajectory_temp[i][3],
					trajectory_temp[i][4],
					trajectory_temp[i][5],
					trajectory_temp[i][6];
				trajectory_vec_vec.push_back(trajectory_vec);
			}
			filename_txt = dir_ + "/" + filenames__txt[0];
		}

		else
		{
			cout << "error:too many txt files exist." << endl;
			return;
		}

		vector<Eigen::Matrix4d>	HM_trajectory_vec;
		for (int i = 0; i < trajectory_vec_vec.size(); i++)
		{
			Eigen::Matrix4d HM_trajectory = Eigen::Matrix4d::Identity();
			HM_trajectory = CKataokaPCL::calcHomogeneousMatrixFromVector6d(
				trajectory_vec_vec[i](0, 0),
				trajectory_vec_vec[i](1, 0),
				trajectory_vec_vec[i](2, 0),
				trajectory_vec_vec[i](3, 0),
				trajectory_vec_vec[i](4, 0),
				trajectory_vec_vec[i](5, 0));
			HM_trajectory_vec.push_back(HM_trajectory);
		}

		for (int i = 0; i < HM_trajectory_vec.size(); i++)
		{
			if (i == 0)
			{
				HM_free = Eigen::Matrix4d::Identity();
				//HM_displacement_vec.push_back(HM_trajectory_vec[i] * HM_free.inverse());
				HM_displacement_vec.push_back(HM_free.inverse() * HM_trajectory_vec[i]);
			}
			else HM_displacement_vec.push_back(HM_trajectory_vec[i - 1].inverse() * HM_trajectory_vec[i]);
		}
		cout << "HM_displacement_vec size = " << HM_displacement_vec.size() << endl;
	}

	while (1) {
		//input next PointCloud
		if (b_makeNewPC) {

			cloud_moving_init->clear();

			string filename_PC;
			filename_PC = dir_ + "/" + filenames_[index_PC_now];

			if (-1 == pcl::io::loadPCDFile(filename_PC, *cloud_moving_init)) break;

			cout << "i:" << index_PC_now << " number:" << cloud_moving_init->size();
			cout << " finename: " << filenames_[index_PC_now] << endl;
			cout << endl;
			cout << "**********( key option )**********" << endl;
			cout << " +X:1  +Y:2  +Z:3  +Roll:4  +Pitch:5  +Yaw:6" << endl;
			cout << " -X:Q  -Y:W  -Z:E  -Roll:R  -Pitch:T  -Yaw:Y" << endl;
			cout << "Resolution: translation:" << resolution_translation;
			cout << "[m] rotation:" << resolution_rotation * R2D << "[deg]" << endl;
			cout << "calc median: Left SHIFT" << endl;
			cout << "Registration: Left CTRL" << endl;
			cout << "Reset:0" << endl;
			cout << "Next:ENTER" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;

			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();

			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i <= index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_Trans_now = HM_free;
			Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_free);
			pcl::transformPointCloud(*cloud_moving_init, *cloud_moving, Trans_);

			b_makeNewPC = false;

			if (b_RemoveGround)
			{
				double th_height;
				//th_height = -0.1;	//naraha summer
				th_height = -0.3;
				cloud_temp->clear();
				pcl::copyPointCloud(*cloud_moving, *cloud_temp);
				cloud_moving->clear();
				for (size_t i = 0; i < cloud_temp->size(); i++)
				{
					if (th_height > cloud_temp->points[i].z) continue;
					cloud_moving->push_back(cloud_temp->points[i]);
				}
			}

		}

		KEYNUM key_;
		key_ = getKEYNUM();

		if (b_first)
		{
			key_ = NONE;
			b_first = false;
		}
		
		//determine transformation by key input
		if (key_ == X_)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(resolution_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == Y_)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., resolution_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == Z_)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., resolution_translation, 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == ROLL_)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., resolution_rotation, 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == PITCH_)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., resolution_rotation, 0.)
				* HM_Trans_now;
		}
		else if (key_ == YAW_)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., resolution_rotation)
				* HM_Trans_now;
		}
		else if (key_ == X_MINUS)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(-resolution_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == Y_MINUS)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., -resolution_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == Z_MINUS)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., -resolution_translation, 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == ROLL_MINUS)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., -resolution_rotation, 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == PITCH_MINUS)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., -resolution_rotation, 0.)
				* HM_Trans_now;
		}
		else if (key_ == YAW_MINUS)
		{
			HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., -resolution_rotation)
				* HM_Trans_now;
		}
		else if (key_ == ZERO)
		{
			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i <= index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_Trans_now = HM_free;
			cout << "0(numpad) pressed and reset" << endl;
		}
		else if (key_ == ENTER)
		{
			*cloud_show_static += *cloud_moving;
			cloud_before->clear();
			pcl::copyPointCloud(*cloud_moving, *cloud_before);
			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i < index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_displacement_vec[index_PC_now] = HM_free.inverse() * HM_Trans_now;
			index_PC_now++;
			if (index_PC_now == HM_displacement_vec.size()) b_break = true;
			b_makeNewPC = true;
			cout << "ENTER pressed" << endl;
		}
		else if (key_ == ESC)
		{
			cout << "ESC called" << endl;
			b_break = true;
			b_escaped = true;

		}
		else if (key_ == RSHIFT)
		{
			if (cloud_before->size() != 0 && cloud_moving->size() != 0)
			{
				cout << "median of this frame and before frame:";
				cout << CKataokaPCL::getMedianDistance(cloud_before, cloud_moving) << endl;
			}
			else cout << "ERROR: couldn't show median because empty pointcloud" << endl;
		}
		else if (key_ == RCTRL)
		{
			Eigen::Vector6d Registration_Vec = Eigen::Vector6d::Zero();
			//GetAsyncKeyState(VK_RETURN);
			cout << endl;
			cout << "Registration" << endl;
			cout << "select additional option:" << endl;
			cout << " 0:ICP" << endl;
			cout << " 1:ICP(only X,Y,Yaw)" << endl;
			cout << " 2:Global Registration" << endl;
			cout << " 3:Global Registration(only X,Y,Yaw)" << endl;
			cout << " 4:Configuration(ICP parameter)" << endl;
			cout << " 5:Configuration(GR parameter)" << endl;
			cout << "->";
			int i_select;
			cin.clear();
			cin.ignore(1024, '\n');
			cin >> i_select;
			if (i_select == 0 || i_select == 1)
			{
				kataokaPCL.setMaximumIterations(MaximumIterations);
				kataokaPCL.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
				kataokaPCL.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
				kataokaPCL.setTransformationEpsilon(TransformationEpsilon);
				kataokaPCL.setInputSource(cloud_moving);
				kataokaPCL.setInputTarget(cloud_before);
				kataokaPCL.align();
				Registration_Vec = kataokaPCL.getFinalTransformation_Vec();
				if (i_select == 0)
				{
					HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(
						Registration_Vec(0, 0), Registration_Vec(1, 0), Registration_Vec(2, 0),
						Registration_Vec(3, 0), Registration_Vec(4, 0), Registration_Vec(5, 0))
						* HM_Trans_now;
					cout << "X:" << Registration_Vec(0, 0) << " Y:" << Registration_Vec(1, 0) << " Z:" << Registration_Vec(2, 0) << endl;
					cout << "Roll:" << Registration_Vec(3, 0) << " Pitch:" << Registration_Vec(4, 0) << " Yaw:" << Registration_Vec(5, 0) << endl;
				}
				else
				{
					HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(
						Registration_Vec(0, 0), Registration_Vec(1, 0), 0.,
						0., 0., Registration_Vec(5, 0))
						* HM_Trans_now;
					cout << "X:" << Registration_Vec(0, 0) << " Y:" << Registration_Vec(1, 0) << " Yaw:" << Registration_Vec(5, 0) << endl;
				}
			}
			else if (i_select == 2) {}
			else if (i_select == 3) {}
			else if (i_select == 4)
			{
				cout << "input MaxCorrespondenceDistance (double)" << endl;
				cout << "->";
				cin >> MaxCorrespondenceDistance;
			}
			else
			{
				cout << "ERROR: cin cought invalid value" << endl;
			}
			GetAsyncKeyState(VK_RETURN);
			cout << "registration finished." << endl;
			cout << endl;
			cout << "**********( key option )**********" << endl;
			cout << " +X:1  +Y:2  +Z:3  +Roll:4  +Pitch:5  +Yaw:6" << endl;
			cout << " -X:Q  -Y:W  -Z:E  -Roll:R  -Pitch:T  -Yaw:Y" << endl;
			cout << "Resolution: translation:" << resolution_translation;
			cout << "[m] rotation:" << resolution_rotation * R2D << "[deg]" << endl;
			cout << "calc median: Left SHIFT" << endl;
			cout << "Registration: Left CTRL" << endl;
			cout << "Reset:0" << endl;
			cout << "Next:ENTER" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;
		}

		if (!(key_ == NONE || key_ == ENTER)) {
			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();
			Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
			pcl::transformPointCloud(*cloud_moving_init, *cloud_moving, Trans_);
		}
		cloud_show->clear();
		*cloud_show += *cloud_show_static;
		*cloud_show += *cloud_moving;
		cloud_show_2frame->clear();
		*cloud_show_2frame += *cloud_before;
		*cloud_show_2frame += *cloud_moving;

		if (cloud_show->size() != 0) {
			pv.setPointCloud(cloud_show);
			pv.updateViewer();
			pv_2frame.setPointCloud(cloud_show_2frame);
			pv_2frame.updateViewer();
		}

		if (b_break) break;
	}

	bool b_save_txt;
	cout << "Do you save txt?  Yes:1 No:0" << endl;
	cout << "->";
	cin >> b_save_txt;

	//output txt
	if (b_save_txt)
	{
		vector<vector<double>> trajectory_vec_vec;
		HM_free = Eigen::Matrix4d::Identity();
		for (int i = 0; i < HM_displacement_vec.size(); i++)
		{
			HM_free = HM_free * HM_displacement_vec[i];
			Eigen::Vector6d State_;
			State_ = CKataokaPCL::calcVector6dFromHomogeneousMatrix(HM_free);
			vector<double> trajectory_vec;
			trajectory_vec.push_back(i);
			trajectory_vec.push_back(State_[0]);
			trajectory_vec.push_back(State_[1]);
			trajectory_vec.push_back(State_[2]);
			trajectory_vec.push_back(State_[3]);
			trajectory_vec.push_back(State_[4]);
			trajectory_vec.push_back(State_[5]);
			trajectory_vec_vec.push_back(trajectory_vec);
		}
		CTimeString::getCSVFromVecVec(trajectory_vec_vec, filename_txt);
		cout << "file has saved!" << endl;
	}
	else cout << "file did not saved!" << endl;

	pv.closeViewer();
	pv_2frame.closeViewer();
}

CPointcloudFunction::KEYNUM CPointcloudFunction::getKEYNUM()
{
	//http://kts.sakaiweb.com/virtualkeycodes.html

	KEYNUM key_;
	bool b_PChasNUMPAD = true;
	b_PChasNUMPAD = false;

	//https://www.slideshare.net/masafuminoda/pcl-11030703
		//Viewer
		//left drag：rotation of view point
		//Shift+left drag：translation of view point
		//Ctrl+left drag：rotation in display
		//right drag：zoom
		//g：display measure
		//j：save screenshot

		//input key
	short key_num_X_;
	short key_num_Y_;
	short key_num_Z_;
	short key_num_ROLL_;
	short key_num_PITCH_;
	short key_num_YAW_;
	short key_num_X_minus;
	short key_num_Y_minus;
	short key_num_Z_minus;
	short key_num_ROLL_minus;
	short key_num_PITCH_minus;
	short key_num_YAW_minus;
	short key_num_ZERO;
	short key_num_ENTER;
	short key_num_ESC;
	//short key_num_SUBTRACT;
	short key_num_RSHIFT;
	short key_num_RCTRL;

	key_num_X_ = GetAsyncKeyState(0x31);	//1
	key_num_Y_ = GetAsyncKeyState(0x32);	//2
	key_num_Z_ = GetAsyncKeyState(0x33);	//3
	key_num_ROLL_ = GetAsyncKeyState(0x34);	//4
	key_num_PITCH_ = GetAsyncKeyState(0x35);//5
	key_num_YAW_ = GetAsyncKeyState(0x36);	//6
	key_num_X_minus = GetAsyncKeyState(0x51);	//Q
	key_num_Y_minus = GetAsyncKeyState(0x57);	//W
	key_num_Z_minus = GetAsyncKeyState(0x45);	//E
	key_num_ROLL_minus = GetAsyncKeyState(0x52);	//R
	key_num_PITCH_minus = GetAsyncKeyState(0x54);//T
	key_num_YAW_minus = GetAsyncKeyState(0x59);	//Y
	key_num_ZERO = GetAsyncKeyState(0x30);	//0
	key_num_ENTER = GetAsyncKeyState(VK_RETURN);
	key_num_ESC = GetAsyncKeyState(VK_ESCAPE);
	//key_num_SUBTRACT = GetAsyncKeyState(VK_OEM_MINUS);
	key_num_RSHIFT = GetAsyncKeyState(VK_RSHIFT);
	key_num_RCTRL = GetAsyncKeyState(VK_RCONTROL);

	if ((key_num_X_ & 1) == 1) key_ = X_;
	else if ((key_num_Y_ & 1) == 1) key_ = Y_;
	else if ((key_num_Z_ & 1) == 1) key_ = Z_;
	else if ((key_num_ROLL_ & 1) == 1) key_ = ROLL_;
	else if ((key_num_PITCH_ & 1) == 1) key_ = PITCH_;
	else if ((key_num_YAW_ & 1) == 1) key_ = YAW_;
	else if ((key_num_X_minus & 1) == 1) key_ = X_MINUS;
	else if ((key_num_Y_minus & 1) == 1) key_ = Y_MINUS;
	else if ((key_num_Z_minus & 1) == 1) key_ = Z_MINUS;
	else if ((key_num_ROLL_minus & 1) == 1) key_ = ROLL_MINUS;
	else if ((key_num_PITCH_minus & 1) == 1) key_ = PITCH_MINUS;
	else if ((key_num_YAW_minus & 1) == 1) key_ = YAW_MINUS;
	else if ((key_num_ZERO & 1) == 1) key_ = ZERO;
	else if ((key_num_ENTER & 1) == 1) key_ = ENTER;
	//else if ((key_num_SUBTRACT & 1) == 1) key_ = SUBTRACT;
	else if ((key_num_ESC & 1) == 1) key_ = ESC;
	else if ((key_num_RSHIFT & 1) == 1) key_ = RSHIFT;
	else if ((key_num_RCTRL & 1) == 1) key_ = RCTRL;
	//{
	//	cout << "ESC called" << endl;
	//	b_escaped = true;
	//	break;
	//}
	else key_ = NONE;

	return key_;
}


void CPointcloudFunction::combinePointCloud_naraha()
{
	string dir_;
	dir_ = "../../data/process05_CombinePointCloud";
	string subdir_;
	subdir_ = "_before";

	{
		vector<string> temp_;
		CTimeString::getFileNames_extension(dir_, temp_, "pcd");
		if (temp_.size() != 0)
		{
			bool b_delete = true;
			cout << "some .pcd already exist" << endl;
			cout << "select delete them or not  1:yes  0:no" << endl;
			cin >> b_delete;
			if (b_delete)
			{
				FileProcess_delete(dir_);
			}
		}
	}

	vector<string> filenames_velo_nonir;
	vector<string> filenames_velo;
	vector<string> filenames_nir;
	//CTimeString::getFileNames_extension(file_dir, filenames_, "nir.pcd");
	CTimeString::getFileNames_extension(dir_ + "/" + subdir_, filenames_velo_nonir, ").pcd");
	CTimeString::getFileNames_extension(dir_ + "/" + subdir_, filenames_velo, "velo.pcd");
	CTimeString::getFileNames_extension(dir_ + "/" + subdir_, filenames_nir, "nir.pcd");

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_velo(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_nir(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_save(new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_XYZI(new pcl::PointCloud<pcl::PointXYZI>());

	bool b_transform = false;
	b_transform = true;
	bool b_removeGround = true;

	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();
	double pitch_init;
	//pitch_init = 22. * M_PI / 180.;
	//pitch_init = 23.5 * M_PI / 180.;
	//pitch_init = 23. * M_PI / 180.;
	pitch_init = 24.4 * M_PI / 180.;
	double th_z_velo;
	//th_z_velo = -0.5;
	//th_z_velo = -0.3;
	//th_z_velo = -0.35;
	th_z_velo = -0.4;

	//outlier rejector
	bool b_rejectOutlier = false;
	//b_rejectOutlier = true;
	float Tolerance_out;
	int MinClusterSize_out;
	Tolerance_out = 1.;
	MinClusterSize_out = 100;
	int Meank_out;
	float StddevMulThresh_out;
	Meank_out = 50;
	StddevMulThresh_out = 0.1;
	cout << "select: do you use outlier rejector?   Yes:1  No 0" << endl;
	cout << "->";
	cin >> b_rejectOutlier;

	{
		for (int index_ = 0; index_ < filenames_velo_nonir.size(); index_++)
		{
			cloud_velo->clear();
			cout << "reading:" << filenames_velo_nonir[index_] << endl;
			pcl::io::loadPCDFile(dir_ + "/" + subdir_ + "/" + filenames_velo_nonir[index_], *cloud_velo);

			if (b_transform)
			{
				//turn pitch(camera coordinate to robot one)
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_velo, *cloud_velo, Trans_);
				if (b_removeGround)
				{
					cloud_temp_XYZI->clear();
					pcl::copyPointCloud(*cloud_velo, *cloud_temp_XYZI);
					cloud_velo->clear();
					for (int i = 0; i < cloud_temp_XYZI->size(); i++)
					{
						if (th_z_velo > cloud_temp_XYZI->points[i].z) continue;
						cloud_velo->push_back(cloud_temp_XYZI->points[i]);
					}
				}
			}

			cloud_save->clear();
			for (int i = 0; i < cloud_velo->size(); i++)
			{
				pcl::PointXYZRGB point_;
				point_.x = cloud_velo->points[i].x;
				point_.y = cloud_velo->points[i].y;
				point_.z = cloud_velo->points[i].z;
				point_.r = 255;
				point_.g = (unsigned char)((int)cloud_velo->points[i].intensity);
				point_.b = 0;
				cloud_save->push_back(point_);
			}

			if (b_rejectOutlier)
			{
				//CKataokaPCL::rejectOutlier(cloud_save, cloud_save, Tolerance_out, MinClusterSize_out);
				CKataokaPCL::Remove_outliers(cloud_save, cloud_save, Meank_out, StddevMulThresh_out);
			}
			string filename_save = filenames_velo_nonir[index_].substr(0, 3) + "XYZRGB_naraha.pcd";
			pcl::io::savePCDFile<pcl::PointXYZRGB>
				(dir_ + "/" + filename_save, *cloud_save);
		}

		for (int index_ = 0; index_ < filenames_velo.size(); index_++)
		{
			if (filenames_velo.size() == 0)
			{
				cout << "ERROR: no velodyne(NIR) pointcloud found" << endl;
				return;
			}

			if (filenames_nir.size() == 0)
			{
				cout << "ERROR: no NIR pointcloud found" << endl;
				return;
			}
			cloud_velo->clear();
			cout << "reading:" << filenames_velo[index_] << endl;
			pcl::io::loadPCDFile(dir_ + "/" + subdir_ + "/" + filenames_velo[index_], *cloud_velo);
			cloud_nir->clear();
			cout << "reading:" << filenames_nir[index_] << endl;
			pcl::io::loadPCDFile(dir_ + "/" + subdir_ + "/" + filenames_nir[index_], *cloud_nir);

			if (b_transform)
			{
				//turn pitch(camera coordinate to robot one)
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_velo, *cloud_velo, Trans_);
				if (b_removeGround)
				{
					cloud_temp_XYZI->clear();
					pcl::copyPointCloud(*cloud_velo, *cloud_temp_XYZI);
					cloud_velo->clear();
					for (int i = 0; i < cloud_temp_XYZI->size(); i++)
					{
						if (th_z_velo > cloud_temp_XYZI->points[i].z) continue;
						cloud_velo->push_back(cloud_temp_XYZI->points[i]);
					}
				}

			}

			if (b_transform)
			{
				//turn pitch(camera coordinate to robot one)
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_nir, *cloud_nir, Trans_);
			}

			cloud_save->clear();
			for (int i = 0; i < cloud_velo->size(); i++)
			{
				pcl::PointXYZRGB point_;
				point_.x = cloud_velo->points[i].x;
				point_.y = cloud_velo->points[i].y;
				point_.z = cloud_velo->points[i].z;
				point_.r = 255;
				point_.g = (unsigned char)((int)cloud_velo->points[i].intensity);
				point_.b = 0;
				cloud_save->push_back(point_);
			}

			for (int i = 0; i < cloud_nir->size(); i++)
			{
				pcl::PointXYZRGB point_;
				point_.x = cloud_nir->points[i].x;
				point_.y = cloud_nir->points[i].y;
				point_.z = cloud_nir->points[i].z;
				point_.r = (unsigned char)((int)cloud_nir->points[i].intensity);
				point_.g = 255;
				point_.b = 0;
				cloud_save->push_back(point_);
			}

			if (b_rejectOutlier)
			{
				//CKataokaPCL::rejectOutlier(cloud_save, cloud_save, Tolerance_out, MinClusterSize_out);
				CKataokaPCL::Remove_outliers(cloud_save, cloud_save, Meank_out, StddevMulThresh_out);
			}
			string filename_save = filenames_velo[index_].substr(0, 3) + "XYZRGB_naraha.pcd";
			pcl::io::savePCDFile<pcl::PointXYZRGB>
				(dir_ + "/" + filename_save, *cloud_save);
		}

	}
	
}

void CPointcloudFunction::changeColor_plane(pcl::PointXYZRGB &point_)
{
	point_.r = 0;
	point_.g = 0;
	point_.b = 255;
}

void CPointcloudFunction::changeColor_plane(pcl::PointXYZI &point_)
{
	point_.intensity = 210;
}

void CPointcloudFunction::DynamicTranslation()
{
	//bool b_saveByTXT = false;
	bool b_inputTranslation = false;

	string dir_;
	dir_ = "../../data/process06_DynamicTranslation";

	typedef typename pcl::PointXYZI PointType_func;
	//typedef typename pcl::PointXYZRGB PointType_func;


	CPointVisualization<PointType_func> pv;

	if (typeid(PointType_func) == typeid(pcl::PointXYZI))
		pv.setWindowName("show XYZI");
	else if (typeid(PointType_func) == typeid(pcl::PointXYZRGB))
		pv.setWindowName("show XYZRGB");
	else
		throw std::runtime_error("This PointType is unsupported.");

	//pcl::PointCloud<PointType_func>::Ptr cloud_show(new pcl::PointCloud<PointType_func>());
	//pcl::PointCloud<PointType_func>::Ptr cloud_show_static(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_moving(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_moving_before(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_temp(new pcl::PointCloud<PointType_func>());

	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d HM_Trans_now = Eigen::Matrix4d::Identity();

	int index_PC_now = 0;
	bool b_makeNewPC = true;
	bool b_first = true;
	bool b_escaped = false;
	bool b_break = false;

	enum KEYNUM {
		KEY_NONE,
		KEY_TRANSLATION,
		//ENTER,
		KEY_SPACE,
		KEY_SUBTRACT,
		//KEY_CTRL
		KEY_SHIFT

	};
	KEYNUM key_;

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	while (1) {
		//input next PointCloud
		if (b_makeNewPC) {

			cloud_moving_before->clear();
			cloud_moving->clear();

			string filename_PC;
			filename_PC = filenames_[index_PC_now];

			if (-1 == pcl::io::loadPCDFile(dir_ + "/" + filename_PC, *cloud_moving_before)) break;

			cout << "PC(" << index_PC_now << "): ";
			cout << filename_PC + " ";
			cout << "number:" << cloud_moving->size() << endl;

			//plane color
			if (cloud_moving_before->size() != 0)
			{
				detectPlane<PointType_func>(*cloud_moving_before, 0.05, true, true);	//velo
				//detectPlane<PointType_func>(*cloud_moving_before, 0.01, true, true);	//nir
			}

			pcl::copyPointCloud(*cloud_moving_before, *cloud_moving);

			cout << endl;
			cout << "**********( key option )**********" << endl;
			cout << "Reset:-(numpad)" << endl;
			cout << "Switch:SPACE" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;

			HM_Trans_now = Eigen::Matrix4d::Identity();

			b_makeNewPC = false;

		}

		//https://www.slideshare.net/masafuminoda/pcl-11030703
		//Viewer
		//left drag：rotation of view point
		//Shift+left drag：translation of view point
		//Ctrl+left drag：rotation in display
		//right drag：zoom
		//g：display measure
		//j：save screenshot

		short key_num_space = GetAsyncKeyState(VK_SPACE);
		short key_num_escape = GetAsyncKeyState(VK_ESCAPE);
		short key_num_subt_numpad = GetAsyncKeyState(VK_SUBTRACT);
		//short key_num_t = GetAsyncKeyState(0x54);//T
		short key_num_alt = GetAsyncKeyState(VK_MENU);//T
		//short key_num_ctrl = GetAsyncKeyState(VK_CONTROL);//T
		short key_num_shift = GetAsyncKeyState(VK_SHIFT);//T

		if ((key_num_space & 1) == 1) key_ = KEY_SPACE;
		else if ((key_num_subt_numpad & 1) == 1) key_ = KEY_SUBTRACT;
		else if ((key_num_alt & 1) == 1) key_ = KEY_TRANSLATION;
		else if ((key_num_escape & 1) == 1)
		{
			cout << "ESC called" << endl;
			b_escaped = true;
			break;
		}
		else if ((key_num_shift & 1) == 1) key_ = KEY_SHIFT;
		else key_ = KEY_NONE;

		//GetAsyncKeyState(0x54);
		//Sleep(0.5 * 1000);
		//cout << " " << endl;

		if (b_first)
		{
			key_ = KEY_NONE;
			b_first = false;
		}

		//determine transformation by key input
		if (key_ == KEY_SPACE)
		{
			//*cloud_show_static += *cloud_moving;
			//HM_free = Eigen::Matrix4d::Identity();
			//for (int i = 0; i < index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			//HM_displacement_vec[index_PC_now] = HM_free.inverse() * HM_Trans_now;
			index_PC_now++;
			if (index_PC_now == filenames_.size()) b_break = true;
			b_makeNewPC = true;
			cout << "ENTER pressed" << endl;
		}
		else if (key_ == KEY_SUBTRACT)
		{
			HM_Trans_now = Eigen::Matrix4d::Identity();
			cout << "-(numpad) pressed" << endl;
		}
		else if (key_ == KEY_SHIFT)
		{
			//calc plane parameter
			bool b_remove_plane = false;

			cout << "removing plane? 1:Yes 0:No ->";
			cin >> b_remove_plane;
			//b_remove_plane = true;

			if (b_remove_plane)
			{
				if (cloud_moving->size() != 0)
				{
					detectPlane<PointType_func>(*cloud_moving_before, 0.05, true, b_remove_plane);	//velo
					//detectPlane<PointType_func>(*cloud_moving_before, 0.01, true, b_remove_plane);	//nir
				}

				cloud_moving->clear();
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
				pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);
			}
			else
			{
				if (cloud_moving->size() != 0)
				{
					detectPlane<PointType_func>(*cloud_moving_before, 0.05, true, b_remove_plane);	//velo
					//detectPlane<PointType_func>(*cloud_moving_before, 0.01, true, b_remove_plane);	//nir
				}

			}
		}
		else if (key_ == KEY_TRANSLATION)
		{
			enum Variable {
				X_vr,
				Y_vr,
				Z_vr,
				Roll_vr,
				Pitch_vr,
				Yaw_vr,
				ESC_vr,
				Other_vr
			};
			Variable var;

			//input variable
			do
			{
				string s_input;
				//cout << endl;
				//cout.flush();
				//cout << " " << endl;
				cout << "select: x y z roll pitch yaw, or ESC" << endl;
				cout << "->";
				cin >> s_input;
				if (s_input == "x") var = X_vr;
				else if (s_input == "y") var = Y_vr;
				else if (s_input == "z") var = Z_vr;
				else if (s_input == "roll") var = Roll_vr;
				else if (s_input == "pitch") var = Pitch_vr;
				else if (s_input == "yaw") var = Yaw_vr;
				else if (s_input == "ESC") var = ESC_vr;
				else var = Other_vr;
				GetAsyncKeyState(VK_RETURN);
				cout << "s_input: " << s_input << endl;
			} while (var == Other_vr);
			if (var == ESC_vr)
			{
				cout << "ESC called" << endl;
				continue;
			}

			//input value
			string s_value_;
			double d_value_ = 0.;;
			cout << endl;
			cout << "input value: translation[m] or rotation[deg]" << endl;
			cout << "->";
			cin >> s_value_;
			d_value_ = stod(s_value_);		//should detect invarid value
			GetAsyncKeyState(VK_RETURN);

			//move pointcloud
			switch (var)
			{
			case X_vr:
				HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(d_value_, 0., 0., 0., 0., 0.)
					* HM_Trans_now;
				break;
			case Y_vr:
				HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., d_value_, 0., 0., 0., 0.)
					* HM_Trans_now;
				break;
			case Z_vr:
				HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., d_value_, 0., 0., 0.)
					* HM_Trans_now;
				break;
			case Roll_vr:
				HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., d_value_ * M_PI / 180., 0., 0.)
					* HM_Trans_now;
				break;
			case Pitch_vr:
				HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., d_value_ * M_PI / 180., 0.)
					* HM_Trans_now;
				break;
			case Yaw_vr:
				HM_Trans_now = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., d_value_ * M_PI / 180.)
					* HM_Trans_now;
				break;
			default:
				break;
			}

			cout << endl;
			cout << "**********( key option )**********" << endl;
			cout << "Reset:-(numpad)" << endl;
			cout << "Switch:ENTER" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;
		}
			
		if (!(key_ == KEY_NONE || key_ == KEY_SPACE)) {
			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();
			Trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
			pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);
		}

		if (cloud_moving->size() != 0) {
			pv.setPointCloud(cloud_moving);
			pv.updateViewer();
		}

		if (b_break) break;
	}

	int i_save_txt = 0;
	cout << "Do you save txt?  Yes:0 No:1" << endl;
	cout << "->";
	cin >> i_save_txt;
	bool b_save_txt;
	if (i_save_txt == 0)
		b_save_txt = true;
	else
		b_save_txt = false;

	pv.closeViewer();
}

void CPointcloudFunction::FileProcess()
{

	string dir_;
	//dir_ = "../../data/temp/_DynamicTranslation";
	dir_ = "../../data";

	vector<string> filenames_folder;

	enum Process
	{
		EN_SHOW_DEEPLY,
		EN_COPY,
		EN_DELETE,
		EN_EVACUATE,
		EN_ESPACE
	};

	////debug
	//{
	//	vector<string> files_temp;
	//	//CTimeString::getFileNames(dir_, files_temp, false, true, false);
	//	CTimeString::getFileNames(dir_, files_temp, true, true, true);
	//	cout << "files_temp.size():"<< files_temp.size() << endl;
	//}


	while (1)
	{
		//show get folder name
		FileProcess_FolderInFolder(dir_, filenames_folder);

		cout << endl;
		cout << "select:" << endl;
		cout << "  show deeply:" << EN_SHOW_DEEPLY << endl;
		cout << "  copy file(.pcd):" << EN_COPY << endl;
		cout << "  delete file(.pcd):" << EN_DELETE << endl;
		cout << "  evacuate file(.pcd) in folder A to new folder B in A:" << EN_EVACUATE << endl;
		cout << "  escape this function:" << EN_ESPACE << endl;
		cout << "->";
		int i_select = 0;
		cin >> i_select;
		cout << endl;

		if (i_select == EN_SHOW_DEEPLY)
		{
			for (int i = 0; i < filenames_folder.size(); i++)
			{
				string s_filename = dir_ + "/" + filenames_folder[i];
				vector<string> temp_;
				CTimeString::getFileNames(s_filename, temp_, false, true, false);
				cout << endl;
				cout << s_filename << endl;
				for (int j = 0; j < temp_.size(); j++)
					cout << "     " << temp_[j] << endl;
			}
		}

		else if (i_select == EN_COPY)
		{
			int i_copy_to = -1;
			int i_copy_from = -1;

			cout << endl;

			cout << "copy from (index) ->";
			cin >> i_copy_from;
			//if (!(-1 < i_copy_from && i_copy_from < filenames_folder.size())) return;
			cout << i_copy_from << "(" << filenames_folder[i_copy_from] << ")" << endl;

			cout << "copy to (index) ->";
			cin >> i_copy_to;
			//if (!(-1 < i_copy_to && i_copy_to < filenames_folder.size())) return;
			cout << i_copy_to << "(" << filenames_folder[i_copy_to] << ")" << endl;

			FileProcess_copy(
				dir_ + "/" + filenames_folder[i_copy_from], 
				dir_ + "/" + filenames_folder[i_copy_to]);

		}

		else if (i_select == EN_DELETE)
		{
			int i_select_2;
			cout << "delete file in folder (index) ->";
			cin >> i_select_2;
			cout << i_select_2 << "(" << filenames_folder[i_select_2] << ")" << endl;
			FileProcess_delete(dir_ + "/" + filenames_folder[i_select_2]);
		}

		else if (i_select == EN_EVACUATE)
		{
			int i_select_2;
			cout << "evaculate files in folder (index) ->";
			cin >> i_select_2;
			cout << i_select_2 << "(" << filenames_folder[i_select_2] << ")" << endl;
			FileProcess_evacuate(dir_ + "/" + filenames_folder[i_select_2]);
		}

		else if (i_select == EN_ESPACE) break;

		else return;
	}

}

void CPointcloudFunction::FileProcess_copy(string dir_from, string dir_to)
{
	vector<string> filenames_copy;
	//check it can copy file
	{
		vector<string> filenames_from;
		CTimeString::getFileNames_extension(dir_from, filenames_from, ".pcd");
		vector<string> filenames_to;
		CTimeString::getFileNames_extension(dir_to, filenames_to, ".pcd");

		for (int j = 0; j < filenames_from.size(); j++)
		{
			bool b_copy = true;
			for (int i = 0; i < filenames_to.size(); i++)
			{
				if (filenames_from[j] == filenames_to[i])
				{
					cout << "ALERT: " << filenames_from[j] << " exist in each folder" << endl;
					b_copy = false;
					break;
				}
			}
			if (b_copy) filenames_copy.push_back(filenames_from[j]);
		}
	}


	//copy
	for (int i = 0; i < filenames_copy.size(); i++)
	{
		string s_filefrom = dir_from + "/" + filenames_copy[i];
		string s_fileto = dir_to + "/" + filenames_copy[i];
		cout << "s_filefrom: " << s_filefrom << endl;
		cout << "s_fileto: " << s_fileto << endl;
		CTimeString::copyfile(s_filefrom, s_fileto);

		//check whether copying succeeded
		vector<string> check_vec;
		bool b_succeeded = false;
		CTimeString::getFileNames_extension(dir_to, check_vec, filenames_copy[i]);
		if (check_vec.size() == 1) b_succeeded = true;
		if (!b_succeeded)
		{
			cout << "ERROR: " << filenames_copy[i] << " was failed to copy" << endl;
			return;
		}
	}
}

void CPointcloudFunction::FileProcess_delete(string dir)
{

	//check
	bool b_canDelete = true;
	vector<string> filenames_delete;
	CTimeString::getFileNames_extension(dir, filenames_delete, ".pcd");
	if (filenames_delete.size() == 0) b_canDelete = false;
	if (!b_canDelete)
	{
		cout << "ERROR: " << dir << " has no file" << endl;
		return;
	}

	{
		cout << "Do you delete it realy?  yes:1  no:0" << endl;
		cout << "->";
		bool b_decision = false;
		cin >> b_decision;
		if (!b_decision) return;
	}

	//delete
	for (int i = 0; i < filenames_delete.size(); i++)
	{
		string s_filedelete = dir + "/" + filenames_delete[i];
		cout << "s_filedelete: " << s_filedelete << endl;
		CTimeString::deletefile(s_filedelete);

		//check whether deleting succeeded
		vector<string> check_vec;
		bool b_succeeded = false;
		CTimeString::getFileNames_extension(dir, check_vec, filenames_delete[i]);
		if (check_vec.size() == 0) b_succeeded = true;
		if (!b_succeeded)
		{
			cout << "ERROR: " << filenames_delete[i] << " was failed to delete" << endl;
			return;
		}
	}
}

void CPointcloudFunction::FileProcess_evacuate(string dir)
{
	vector<string> filenames_main;
	CTimeString::getFileNames_extension(dir, filenames_main, ".pcd");
	{
		vector<string> filenames_show;
		CTimeString::getFileNames_extension(dir, filenames_show, ".pcd");
		cout << "show files" << endl;
		for (int i = 0; i < filenames_show.size(); i++) cout << filenames_show[i] << endl;
	}

	//make new folder
	string s_newfoldername;
	cout << endl;
	cout << "write new folder name (don't use SPACE)" << endl;
	cout << "->";
	cin >> s_newfoldername;
	CTimeString::makenewfolder(dir, s_newfoldername);

	//copy
	for (int i = 0; i < filenames_main.size(); i++)
	{
		string s_filefrom = dir + "/" + filenames_main[i];
		string s_fileto = dir + "/" + s_newfoldername + "/" + filenames_main[i];
		cout << "s_filefrom: " << s_filefrom << endl;
		cout << "s_fileto: " << s_fileto << endl;
		CTimeString::copyfile(s_filefrom, s_fileto);
		//check whether copying succeeded
		vector<string> check_vec;
		bool b_succeeded = false;
		CTimeString::getFileNames_extension(dir + "/" + s_newfoldername, check_vec, filenames_main[i]);
		if (check_vec.size() == 1) b_succeeded = true;
		if (!b_succeeded)
		{
			cout << "ERROR: " << filenames_main[i] << " was failed to copy" << endl;
			return;
		}
	}

	//delete
	for (int i = 0; i < filenames_main.size(); i++)
	{
		string s_filedelete = dir + "/" + filenames_main[i];
		cout << "s_filedelete: " << s_filedelete << endl;
		CTimeString::deletefile(s_filedelete);
		//check whether deleting succeeded
		vector<string> check_vec;
		bool b_succeeded = false;
		CTimeString::getFileNames_extension(dir, check_vec, filenames_main[i]);
		if (check_vec.size() == 0) b_succeeded = true;
		if (!b_succeeded)
		{
			cout << "ERROR: " << filenames_main[i] << " was failed to delete" << endl;
			return;
		}
	}
}

void CPointcloudFunction::FileProcess_FolderInFolder(string dir_, vector<string> &folder_vec)
{
	folder_vec.clear();

	vector<string> filenames_folder_temp;
	CTimeString::getFileNames_folder(dir_, filenames_folder_temp);

	////debug
	//cout << "size: " << filenames_folder_temp.size() << endl;

	int num_folder = 0;
	string s_index_folder;
	cout << endl;
	for (int j = 0; j < filenames_folder_temp.size(); j++)
	{
		folder_vec.push_back(filenames_folder_temp[j]);
		s_index_folder = to_string(num_folder);
		if (s_index_folder.size() != 2) s_index_folder = " " + s_index_folder;
		cout << "i:" << s_index_folder;
		cout << " " << filenames_folder_temp[j];
		num_folder++;
		{
			vector<string> filenames_folder_temp2;
			CTimeString::getFileNames(dir_ + "/" + filenames_folder_temp[j], filenames_folder_temp2, false, true, false);
			if (filenames_folder_temp2.size() == 0)
			{
				cout << "  <-(blank)" << endl;
				continue;
			}
			cout << endl;
		}
		vector<string> filenames_folder_folder;
		CTimeString::getFileNames_folder(dir_ + "/" + filenames_folder_temp[j], filenames_folder_folder);

		for (int i = 0; i < filenames_folder_folder.size(); i++)
		{
			folder_vec.push_back(
				filenames_folder_temp[j] + "/" + filenames_folder_folder[i]);

			s_index_folder = to_string(num_folder);
			if (s_index_folder.size() != 2) s_index_folder = " " + s_index_folder;
			cout << "i:" << s_index_folder;
			cout << " " << filenames_folder_temp[j] + "/" + filenames_folder_folder[i];
			vector<string> filenames_folder_folder_folder;
			CTimeString::getFileNames(
				dir_ + "/" + filenames_folder_temp[j] + "/" + filenames_folder_folder[i],
				filenames_folder_folder_folder, false, true, false);
			if (filenames_folder_folder_folder.size() == 0) cout << "  <-(blank)";
			cout << endl;
			num_folder++;
		}
	}
}

void CPointcloudFunction::DrawTrajectory()
{
	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	//read file name
	string dir = "../../data/process07_DrawTrajectory";
	vector<string> filenames_txt;
	CTimeString::getFileNames_extension(dir, filenames_txt, ".csv");

	if (filenames_txt.size() == 0)
	{
		cout << "ERROR: no file has found" << endl;
		return;
	}

	cout << "select file:" << endl;
	//int i_readfile = 0;
	vector<int> i_readfile_vec;
	for (int i = 0; i < filenames_txt.size(); i++)
		cout << " " << i << ": " << filenames_txt[i] << endl;
	cout << "->";
	{
		vector<string> s_vec;
		s_vec = CTimeString::inputSomeString();
		for (int i = 0; i < s_vec.size(); i++)
		{
			int i_value = stoi(s_vec[i]);
			if(!(0 <= i_value && i_value < filenames_txt.size()))
				throw std::runtime_error("ERROR: Invalid trajectories readed.");
			i_readfile_vec.push_back(i_value);
		}
		if (!(1 <= i_readfile_vec.size() && i_readfile_vec.size() <= 2))
			throw std::runtime_error("ERROR: Invalid trajectories readed.");
	}

	vector<Eigen::Vector6d> trajectory_vec_vec;
	vector<Eigen::Vector6d> trajectory_vec_vec2;

	//input trajectory 0
	{
		vector<vector<double>> trajectory_temp;
		trajectory_temp = CTimeString::getVecVecFromCSV(dir + "/" + filenames_txt[i_readfile_vec[0]]);
		for (int i = 0; i < trajectory_temp.size(); i++)
		{
			Eigen::Vector6d trajectory_vec = Eigen::Vector6d::Zero();
			trajectory_vec <<
				trajectory_temp[i][1],
				trajectory_temp[i][2],
				trajectory_temp[i][3],
				trajectory_temp[i][4],
				trajectory_temp[i][5],
				trajectory_temp[i][6];
			trajectory_vec_vec.push_back(trajectory_vec);
		}
	}

	bool b_show_sequently = false;
	if (i_readfile_vec.size() == 1)
	{
		cout << "input: show trajectory in   1:sequently  or  0:single frame" << endl;
		cout << "->";
		cin >> b_show_sequently;
	}
	else if (i_readfile_vec.size() == 2)
	{
		//input trajectory 1
		vector<vector<double>> trajectory_temp;
		trajectory_temp = CTimeString::getVecVecFromCSV(dir + "/" + filenames_txt[i_readfile_vec[1]]);
		for (int i = 0; i < trajectory_temp.size(); i++)
		{
			Eigen::Vector6d trajectory_vec = Eigen::Vector6d::Zero();
			trajectory_vec <<
				trajectory_temp[i][1],
				trajectory_temp[i][2],
				trajectory_temp[i][3],
				trajectory_temp[i][4],
				trajectory_temp[i][5],
				trajectory_temp[i][6];
			trajectory_vec_vec2.push_back(trajectory_vec);
		}
	}

	typedef typename CPointVisualization<T_PointType> CPV;
	CPV pv;
	pv.setWindowName("trajectory");

	if (i_readfile_vec.size() == 2)
	{
		pv.drawTrajectory(trajectory_vec_vec, trajectory_vec_vec.size(), true, true, "trajectory0");
		pv.drawTrajectory(trajectory_vec_vec2, trajectory_vec_vec.size(), true, true, "trajectory1");
	}

	else if (i_readfile_vec.size() == 1 && !b_show_sequently)
		pv.drawTrajectory(trajectory_vec_vec, trajectory_vec_vec.size(), true, true, "trajectory0");

	else if (i_readfile_vec.size() == 1 && b_show_sequently){}

	int index_ = 0;
	while (1)
	{
		if(b_show_sequently && GetAsyncKeyState(VK_SPACE) & 1)
		{
			pv.drawTrajectory(trajectory_vec_vec, index_, true, true, "trajectory0");
			index_++;
			if (index_ >= trajectory_vec_vec.size()) cout << "press ESC to escape" << endl;
		}
		pv.updateViewer();
		if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
	}

	pv.closeViewer();
}

void CPointcloudFunction::DoSegmentation()
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
		cout<<" size:"<< cloud_vec[idx]->size() << endl;

		vector <pcl::PointCloud<T_PointType>::Ptr> cloud_cluster_vec;
		pcl::PointCloud<T_PointType>::Ptr cloud_rest(new pcl::PointCloud<T_PointType>);
		cloud_cluster_vec = CKataokaPCL::getSegmentation_robust(cloud_vec[idx], cloud_rest, Tolerance, 100);

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

void CPointcloudFunction::GlobalRegistration_FPFH_SAC_IA()
{
	string dir_;
	dir_ = "../../data/process09_GR_FPFH_SAC_IA";

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

void CPointcloudFunction::GR_FPFH_getResultAnalysis(string dir_, string s_folder)
{
	cout << "calc " << s_folder << endl;
	GR_FPFH_makeFusionCSV(dir_, s_folder);

	GR_FPFH_makeSuccessEstimation(dir_ + "/" + s_folder);
	cout << endl;
}

void CPointcloudFunction::GR_FPFH_makeFusionCSV(string dir_, string s_folder)
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

vector<vector<string>> CPointcloudFunction::GR_FPFH_makeMatrix(vector<vector<int>> int_vecvec)
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

void CPointcloudFunction::GR_FPFH_makeSuccessEstimation(string dir_)
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

void CPointcloudFunction::GR_FPFH_getResultOfPatterns(string dir_)
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
		if(!b_useCSV) s_input_vec = CTimeString::inputSomeString();
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

vector<string> CPointcloudFunction::GR_FPFH_getResultOfOnePattern(string dir_, string s_folder)
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
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_,"_SucEst.csv");
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

void CPointcloudFunction::GR_FPFH_SAC_IA_2frames(string dir_, vector<float> parameter_vec)
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
			Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(transform_);
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

void CPointcloudFunction::GR_FPFH_SAC_IA_Allframes(string dir_, vector<float> parameter_vec, bool b_changeParameter)
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

	if(b_changeParameter)
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
		fpfh = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF,cloud_vec[i], radius_normal_FPFH, radius_FPFH);
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
		s_result_vec = GR_FPFH_SAC_IA_Allframes_OnePair(dir_ + "/" + s_newfoldername, parameter_vec,i_tgt,i_src, cloud_vec,fpfh_vec,
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
	if(b_useClusterNotification)
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
			CTimeString::getCSVFromVecVec(s_temp, dir_ + "/" + s_newfoldername +"_hasClusterSize" + to_string(cluster_vecvec[0].size()) +  ".csv");
		}
	}

	cout << endl;
}

vector<string> CPointcloudFunction::GR_FPFH_SAC_IA_Allframes_OnePair(string dir_, vector<float> parameter_vec, int i_tgt, int i_src,
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
		T_src_TRUE = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]).inverse()
			* CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);

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
		Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(transform_);
		pcl::transformPointCloud(*cloud_src, *cloud_src, Trans_temp);
	}
	//distance
	double distance_ = 0.;
	{
		Eigen::Matrix4d T_i_src = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d T_i1_tgt = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d T_i_GL = Eigen::Matrix4d::Identity();
		//T_i_src = T_i1_tgt * T_i_GL
		T_i_src = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
		T_i1_tgt = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
		T_i_GL = T_i1_tgt.inverse() * T_i_src;
		Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
		Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(T_i_GL);
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
	double median_ = CKataokaPCL::getMedianDistance(cloud_src, cloud_tgt);
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
	transform_vec = CKataokaPCL::calcVector6dFromHomogeneousMatrix(transform_);
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

vector<pair<int, int>> CPointcloudFunction::GR_FPFH_SAC_IA_get_frame_pair_vec(string dir_)
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

void CPointcloudFunction::GR_addToOutputString_OutputHeader_FPFH(vector<vector<string>> &s_output_vecvec)
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

void CPointcloudFunction::GR_FPFH_error(string dir_, vector<float> parameter_vec)
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
			T_i_src = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
			T_i1_tgt = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
			T_i_GL = T_i1_tgt.inverse() * T_i_src;
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(T_i_GL);
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
		//variance_vec = CKataokaPCL::getFPFHVariance(fpfh_src);
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

void CPointcloudFunction::GR_FPFH_error_AllFrames(string dir_, vector<float> parameter_vec, bool b_changeParameter)
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

	if(b_changeParameter)
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
				T_i_src = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
				T_i1_tgt = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
				T_i_GL = T_i1_tgt.inverse() * T_i_src;
				Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
				Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(T_i_GL);
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

void CPointcloudFunction::DoOutlierRejector()
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
				CKataokaPCL::Remove_outliers(cloud_, cloud_, Meank_out, StddevMulThresh_out);
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

void CPointcloudFunction::GR_FPFH_variance_AllFrames(string dir_, vector<float> parameter_vec, bool b_changeParameter)
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

void CPointcloudFunction::GR_FPFH_varyParameter(string dir_, vector<float> parameter_vec_arg)
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

void CPointcloudFunction::GR_FPFH_FixFusion(string dir_, string s_folder)
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

void CPointcloudFunction::DoICP_addToOutputString_OutputHeader(vector<vector<string>> &s_output_vecvec)
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

void CPointcloudFunction::DoICP_proposed_AllFrames()
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	int i_method;
	cout << "select: single parameter:0  vary parameters:1  merge results:2" << endl;
	cout << "->";
	cin >> i_method;

	string dir_ = "../../data/process10_DoICP_proposed_AllFrames";
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

void CPointcloudFunction::DoICP_proposed_givenParameter(string dir_, string filename_csv, vector<float> parameter_vec)
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

void CPointcloudFunction::DoICP_proposed_only1method(
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
			chara_vec = CKataokaPCL::ICP_Chara_GetCharaData(cloud_vec[i]);
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
			Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(initPos_vec[j].Init_Vector));
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
			Registration_Vector = CKataokaPCL::calcVector6dFromHomogeneousMatrix(
				align_ICP.getFinalTransformation().cast<double>());
			fitnessscore = align_ICP.getFitnessScore();
		}
		else if (i_method == 1)
		{
			CKataokaPCL align_ICP_proposed;
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
			Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(Registration_Vector));
			pcl::transformPointCloud(*cloud_src_transformed, *cloud_src_transformed, Trans_temp);
		}

		//inlier rate
		double rate_inlier;
		if (cloud_src->size() != 0)
		{
			pcl::Correspondences corr;
			corr = CKataokaPCL::determineCorrespondences_output(cloud_src_transformed, cloud_tgt, MaxCorrespondenceDistance);
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
			T_i_src = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_src]);
			T_i1_tgt = CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectory_vec[i_tgt]);
			T_i_TRUE = T_i1_tgt.inverse() * T_i_src;
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(T_i_TRUE);
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
		median_ = CKataokaPCL::getMedianDistance(cloud_src_transformed, cloud_tgt);
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
		transform_vec = CKataokaPCL::calcVector6dFromHomogeneousMatrix(
			CKataokaPCL::calcHomogeneousMatrixFromVector6d(Registration_Vector)
			*CKataokaPCL::calcHomogeneousMatrixFromVector6d(initPos_vec[j].Init_Vector));
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

void CPointcloudFunction::DoICP_proposed_varyParameters(string dir_, string filename_csv)
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

void CPointcloudFunction::DoICP_proposed_mergeResult()
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
		if(b_useMedian)
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

vector<string> CPointcloudFunction::DoICP_proposed_mergeResult_OnePattern(string dir_, string s_folder, bool b_useMedian)
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
		if(b_useMedian)
			s_vec_output.push_back(to_string(compare_median));
		if (b_useTransformation)
			s_vec_output.push_back(to_string(compare_transformation));
	}

	return s_vec_output;
}

void CPointcloudFunction::DoEvaluation_ICP_property()
{
	int i_method;
	cout << "select: optimization:0  mergeResult:1" << endl;
	cout << "->";
	cin >> i_method;

	string dir_ = "../../data/process11_DoEvaluation_ICP_property";

	if(i_method == 0)
		DoEvaluation_ICP_property_files(dir_);
	else if(i_method == 1)
		DoEvaluation_ICP_property_mergeResult(dir_);
}

void CPointcloudFunction::DoEvaluation_ICP_property_files(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	vector<string> filenames_input;
	{
		vector<string> filenames_temp;
		CTimeString::getFileNames_extension(dir_,filenames_temp, "_optimization.csv");
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

void CPointcloudFunction::DoEvaluation_ICP_property_addToFile(string dir_,
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
	for(int j=0;j< trajectoryVector_vec_ICP_proposed.size();j++)
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

void CPointcloudFunction::DoEvaluation_ICP_property_calculation(string dir_, string s_folder, vector<Eigen::Vector6d> trajectoryVector_vec,
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
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(
					trajectoryVector_vec_TRUE[frames_ajusted[j - 1]]).inverse()
				* CKataokaPCL::calcHomogeneousMatrixFromVector6d(
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
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(
					trajectoryVector_vec[frames_ajusted[j - 1]]).inverse()
				* CKataokaPCL::calcHomogeneousMatrixFromVector6d(
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
		Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
			displacementMat_vector[j]);
		pcl::transformPointCloud(*cloud_src, *cloud_src, Trans_temp);
		frameCloudMedian_vec.push_back(CKataokaPCL::getMedianDistance(cloud_src, cloud_tgt));
	}

	//map
	pcl::PointCloud<T_PointType>::Ptr cloud_map_TRUE(new pcl::PointCloud<T_PointType>());
	for (int j = 0; j < frames_ajusted.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[frames_ajusted[j]], *cloud_);
		Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
		Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(displacementMat_vector_TRUE[j]);
		pcl::transformPointCloud(*cloud_, *cloud_, Trans_temp);
		*cloud_map_TRUE += *cloud_;
	}
	pcl::PointCloud<T_PointType>::Ptr cloud_map_(new pcl::PointCloud<T_PointType>());
	for (int j = 0; j < frames_ajusted.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[frames_ajusted[j]], *cloud_);
		Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
		Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(displacementMat_vector[j]);
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

void CPointcloudFunction::DoEvaluation_ICP_property_mergeResult(string dir_)
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

vector<string> CPointcloudFunction::DoEvaluation_ICP_property_mergeResult_OnePattern(string dir_, string s_folder)
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

void CPointcloudFunction::DoMappingFromTrajectory()
{
	typedef pcl::PointXYZRGB T_PointType;

	string dir_ = "../../data/process12_DoMappingFromTrajectory";

	bool b_showNumber = false;
	b_showNumber = true;

	bool b_showArrow = false;
	b_showArrow = true;

	bool b_useGrid = false;
	b_useGrid = true;

	float yaw_trans = 0.;
	yaw_trans = -7.* D2R;

	//input trajectory
	string trajectory_csv = "trajectory.csv";
	vector<Eigen::Vector6d> trajectoryVector_vec;
	vector<int> frames_all;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + trajectory_csv);

		for (int j = 1; j < s_temp_vecvec.size(); j++)
		{
			Eigen::Vector6d trajectoryVector_ = Eigen::Vector6d::Zero();
			trajectoryVector_ <<
				stod(s_temp_vecvec[j][1]),
				stod(s_temp_vecvec[j][2]),
				stod(s_temp_vecvec[j][3]),
				stod(s_temp_vecvec[j][4]),
				stod(s_temp_vecvec[j][5]),
				stod(s_temp_vecvec[j][6]);
			//yaw
			Eigen::Matrix4d yaw_Mat = Eigen::Matrix4d::Identity();
			yaw_Mat = CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., yaw_trans);
			yaw_Mat = yaw_Mat * CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectoryVector_);
			trajectoryVector_ = CKataokaPCL::calcVector6dFromHomogeneousMatrix(yaw_Mat);
			trajectoryVector_vec.push_back(trajectoryVector_);
			if (stoi(s_temp_vecvec[j][7]) == 1) frames_all.push_back(-1);
			else frames_all.push_back(j - 1);
		}
	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	{
		vector<string> cloud_folder;
		CTimeString::getFileNames_folder(dir_, cloud_folder);
		for (int i = 0; i < cloud_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << cloud_folder[i] << endl;
		}
		cout << "select folder of input point cloud" << endl;
		cout << "->";
		int i_folder;
		cin >> i_folder;

		vector<string> cloud_filenames;
		CTimeString::getFileNames_extension(dir_ + "/" + cloud_folder[i_folder], cloud_filenames, ".pcd");
		for (int i = 0; i < cloud_filenames.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + cloud_folder[i_folder] + "/" + cloud_filenames[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}

		if (cloud_vec.size() == 0)
		{
			cout << "ERROR: no point cloud found." << endl;
			return;
		}

	}

	bool b_show_sequently = false;
	cout << "input: show map in   1:sequently  or  0:single frame" << endl;
	cout << "->";
	cin >> b_show_sequently;

	for (int j = frames_all.size() - 1; j >= 0; j--)
	{
		if (frames_all[j] != -1) continue;
		trajectoryVector_vec.erase(trajectoryVector_vec.begin() + j);
		cloud_vec.erase(cloud_vec.begin() + j);
	}

	typedef typename CPointVisualization<T_PointType> CPV;
	CPV pv;
	pv.setWindowName("MAP");
	if(b_useGrid) pv.drawGrid(30., 30., -30., -30.);

	pcl::PointCloud<T_PointType>::Ptr cloud_map(new pcl::PointCloud<T_PointType>());

	//show point cloud
	{
		int index_cloud = 0;
		while (1)
		{

			bool b_next = (GetAsyncKeyState(VK_SPACE) & 1) == 1;
			//if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && (index_cloud != cloud_vec.size()))
			if ((b_next || !b_show_sequently) && (index_cloud != cloud_vec.size()))
			{
				pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
				pcl::copyPointCloud(*cloud_vec[index_cloud], *cloud);
				//transformation
				{
					auto trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
						CKataokaPCL::calcHomogeneousMatrixFromVector6d(trajectoryVector_vec[index_cloud])
					);
					pcl::transformPointCloud(*cloud, *cloud, trans_);
				}
				*cloud_map += *cloud;
				pv.setPointCloud(cloud_map);
				pv.drawTrajectory(trajectoryVector_vec, index_cloud, b_showNumber, b_showArrow, "trajectory0");
				cout << "index:" << index_cloud << endl;
				index_cloud++;
			}
			if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
			pv.updateViewer();
		}

		cout << "escaped !" << endl;
	}

	pv.closeViewer();

	bool b_savePointCloud = false;
	cout << "Do you save .pcd of the Map?   Yes:1  No:0" << endl;
	cout << "->";
	cin >> b_savePointCloud;

	if (b_savePointCloud)
	{
		string s_time = CTimeString::getTimeString();
		string s_filename_save = "map_" + CTimeString::getTimeString() + ".pcd";
		pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_filename_save, *cloud_map);
	}

}

void CPointcloudFunction::DoDifferential()
{
	string dir_ = "../../data/process13_DoDifferential";

	int i_method;
	cout << "select method:" << endl;
	cout << "0: SomePointclouds" << endl;
	cout << "1: DoDifferential_showFeatureValue" << endl;
	cout << "2: FPFH_unique" << endl;
	cout << "3: RigidTransformation_FPFH_Features" << endl;
	cout << "4: RigidTransformation_FPFH_Features_allFrames" << endl;
	cout << "5: DoDifferential_PairEvaluation" << endl;
	cout << "->";
	cin >> i_method;

	if (i_method == 0)
		DoDifferential_SomePointclouds(dir_);
	else if (i_method == 1)
		DoDifferential_showFeatureValue(dir_);
	else if (i_method == 2)
		FPFH_unique(dir_);
	else if (i_method == 3)
		DoDifferential_RigidTransformation_FPFH_Features_new(dir_);
	else if (i_method == 4)
		DoDifferential_RigidTransformation_FPFH_Features_allFrames(dir_);
	else if (i_method == 5)
		DoDifferential_PairEvaluation(dir_);
}

void CPointcloudFunction::DoDifferential_1pointcloud(string dir_)
{
	typedef typename pcl::PointXYZRGB T_PointType;

	string s_file_0 = "000XYZRGB_naraha.pcd";
	string s_file_1 = "005XYZRGB_naraha.pcd";

	float radius_differential = 0.5;
	//{
	//	cout << "input radius ->";
	//	float radius_temp;
	//	cin >> radius_temp;
	//	if (0 < radius_temp && radius_temp < 3) radius_differential = radius_temp;
	//	cout << "radius_differential:" << radius_differential << endl;
	//}

	pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_colored(new pcl::PointCloud<T_PointType>());
	//pcl::io::loadPCDFile(dir_ + "/" + s_file_0, *cloud_);
	pcl::io::loadPCDFile(dir_ + "/" + s_file_1, *cloud_);
	cloud_->is_dense = true;

	pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	kdtree_->setInputCloud(cloud_);
	vector<float> feature_vec;

	cout << "select feature:  Velodyne:1  NIR:0" << endl;
	bool b_useVelodyneFeature;
	cin >> b_useVelodyneFeature;

	for (int j = 0; j < cloud_->size(); j++)
	{
		if (!b_useVelodyneFeature)
			feature_vec.push_back((float)((int)cloud_->points[j].r));
		else
			feature_vec.push_back((float)((int)cloud_->points[j].g));
	}

	vector<float> featureDivergence_vec;
	featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_, feature_vec, kdtree_, radius_differential, 0.25);

	//for (int j = 0; j < featureDivergence_vec.size(); j++)
	//{
	//	if (j % 100 != 0) continue;
	//	cout << "j:" << j << "  divergence:" << featureDivergence_vec[j] << endl;
	//}
	//cout << endl;

	pcl::copyPointCloud(*cloud_, *cloud_colored);

	vector<int> hist_vec = CTimeString::getHistogram(featureDivergence_vec, 10, true);

	float feature_min = std::numeric_limits<float>::max();
	float feature_max = -std::numeric_limits<float>::max();
	float feature_mean = 0.;
	float feature_median;
	float feature_first_quartile;
	float feature_third_quartile;

	for (int j = 0; j < featureDivergence_vec.size(); j++)
	{
		if (feature_min > featureDivergence_vec[j]) feature_min = featureDivergence_vec[j];
		if (feature_max < featureDivergence_vec[j]) feature_max = featureDivergence_vec[j];
	}
	//calc median and quartile
	{
		vector<float> temp_vec = CTimeString::getMedian_Quartile(featureDivergence_vec);
		feature_first_quartile = temp_vec[0];
		feature_median = temp_vec[1];
		feature_third_quartile = temp_vec[2];
	}
	//calc mean
	{
		for (int j = 0; j < featureDivergence_vec.size(); j++)
			feature_mean += featureDivergence_vec[j];
		if (featureDivergence_vec.size() != 0) feature_mean /= (float)featureDivergence_vec.size();
		else feature_mean = 10000.;
	}

	cout << "feature_min:" << feature_min << endl;
	cout << "feature_max:" << feature_max << endl;
	cout << "feature_mean:" << feature_mean << endl;
	cout << "feature_first_quartile:" << feature_first_quartile << endl;
	cout << "feature_median:" << feature_median << endl;
	cout << "feature_third_quartile:" << feature_third_quartile << endl;


	float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	th_velodyne_min_color = -20.;
	//cout << "input th_velodyne_min_color ->";
	//cin >> th_velodyne_min_color;

	float th_velodyne_max_color;
	//th_velodyne_max_color = 50.;
	//th_velodyne_max_color = 20.;
	//th_velodyne_max_color = 120.;
	th_velodyne_max_color = 80.;
	//cout << "input th_velodyne_max_color ->";
	//cin >> th_velodyne_max_color;

	float th_nir_min_color;


	float th_nir_max_color;
	th_nir_max_color = 100.;

	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
		for (int j = 0; j < s_temp_vecvec.size(); j++)
			cout << "j:" << j << " " << "s_temp_vecvec[j].size():" << s_temp_vecvec[j].size() << endl;
		th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
		th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
		th_nir_max_color = stof(s_temp_vecvec[5][3]);
	}

	for (int j = 0; j < cloud_colored->size(); j++)
	{
		//if (fabs(featureDivergence_vec[j]) > 50.)
		//{
		//	cloud_colored->points[j].r = 0;
		//	cloud_colored->points[j].g = 255;
		//	cloud_colored->points[j].b = 0;
		//}
		//else
		//{
		//	cloud_colored->points[j].r = 255;
		//	cloud_colored->points[j].g = 255;
		//	cloud_colored->points[j].b = 255;
		//}


		vector<std::uint8_t> color_vec;
		//color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[j], feature_max, feature_min);

		if (b_useVelodyneFeature)
			color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[j], th_velodyne_max_color, th_velodyne_min_color);
		else
			color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[j], th_nir_max_color, feature_min);

		cloud_colored->points[j].r = color_vec[0];
		cloud_colored->points[j].g = color_vec[1];
		cloud_colored->points[j].b = color_vec[2];
	}

	for (int j = 0; j < cloud_colored->size(); j++)
	{
		if (fabs(featureDivergence_vec[j]) == 0.)
		{
			cloud_colored->points[j].r = 255;
			cloud_colored->points[j].g = 255;
			cloud_colored->points[j].b = 255;
		}

		if (b_useVelodyneFeature)
		{
			if (th_velodyne_min_color > featureDivergence_vec[j])
			{
				cloud_colored->points[j].r = 255;
				cloud_colored->points[j].g = 255;
				cloud_colored->points[j].b = 0;
			}
			else if (th_velodyne_max_color < featureDivergence_vec[j])
			{
				cloud_colored->points[j].r = 0;
				cloud_colored->points[j].g = 0;
				cloud_colored->points[j].b = 255;
			}
		}
		else
		{
			//th_velodyne_max_color
			if (th_nir_max_color < featureDivergence_vec[j])
			{
				cloud_colored->points[j].r = 255;
				cloud_colored->points[j].g = 130;
				cloud_colored->points[j].b = 0;
			}

		}
	}

	Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
		CKataokaPCL::calcHomogeneousMatrixFromVector6d(40., 0., 0., 0., 0., 0.));
	pcl::transformPointCloud(*cloud_colored, *cloud_colored, trans_);
	*cloud_colored += *cloud_;
	//pcl::io::savePCDFile<T_PointType>(dir_save + "/cloud_colored.pcd", *cloud_colored);

	typedef typename CPointVisualization<T_PointType> CPV;

	CPV pv;
	pv.setWindowName("differential");

	while (1)
	{
		pv.setPointCloud(cloud_colored);
		pv.updateViewer();
		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
	}

	pv.closeViewer();
}

void CPointcloudFunction::DoDifferential_SomePointclouds(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	cout << "select feature:  NIR:0   Velodyne:1" << endl;
	bool b_useVelodyneFeature;
	cin >> b_useVelodyneFeature;

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	float th_velodyne_min_color;
	th_velodyne_min_color = -4.;
	float th_velodyne_max_color;
	th_velodyne_max_color = 3.6;
	float th_nir_min_color;
	th_nir_min_color = -20;
	float th_nir_max_color;
	th_nir_max_color = 70.;

	float sigma_weight;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
		radius_differential = stof(s_temp_vecvec[1][3]);
		th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
		th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
		th_nir_min_color = stof(s_temp_vecvec[4][3]);
		th_nir_max_color = stof(s_temp_vecvec[5][3]);
		sigma_weight = stof(s_temp_vecvec[6][3]);
	}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;

	//feature
	vector<vector<float>> feature_vecvec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		vector<float> feature_vec;
		for (int i = 0; i < cloud_vec[j]->size(); i++)
		{
			if (!b_useVelodyneFeature)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			else
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
		}
		feature_vecvec.push_back(feature_vec);
	}

	//calc
	vector<vector<float>> featureDivergence_vecvec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		//cout << "j:" << j << endl;
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
		kdtree_->setInputCloud(cloud_vec[j]);
		vector<float> featureDivergence_vec;
		featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec[j], kdtree_, radius_differential, sigma_weight, true);
		featureDivergence_vecvec.push_back(featureDivergence_vec);
	}

	float value_max_all = -std::numeric_limits<float>::max();
	float value_min_all = std::numeric_limits<float>::max();
	for (int j = 0; j < featureDivergence_vecvec.size(); j++)
	{
		for (int i = 0; i < featureDivergence_vecvec[j].size(); i++)
		{
			if (value_max_all < featureDivergence_vecvec[j][i]) value_max_all = featureDivergence_vecvec[j][i];
			if (value_min_all > featureDivergence_vecvec[j][i]) value_min_all = featureDivergence_vecvec[j][i];
		}
	}

	vector<vector<int>> index_valid_vecvec;
	index_valid_vecvec = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec, num_bin_hist, true);

	////botsu?
	//vector<pair<int, int>> corr_vec = CKataokaPCL::determineCorrespondences_feature_histogram(featureDivergence_vecvec[1],
	//	featureDivergence_vecvec[0], num_bin_hist, value_max_hist, value_min_hist, range_hist, true);
	//cout << "corr_vec.size():" << corr_vec.size() << endl;

	pcl::Correspondences corr_new;
	vector<pcl::Correspondences> corr_new_vec;
	{
		int i_src = 1;
		int i_tgt = 0;
		int num_nearest = 10;
		//corr_new = CKataokaPCL::determineCorrespondences_feature(featureDivergence_vecvec[i_src], featureDivergence_vecvec[i_tgt], num_nearest);
		corr_new = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec[i_src], featureDivergence_vecvec[i_tgt],
			index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], num_nearest);
		cout << "corr_new.size():" << corr_new.size() << endl;
		corr_new_vec = CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt(cloud_vec[i_src], cloud_vec[i_tgt], corr_new, 0.9);
		for (int j = 0; j < corr_new_vec.size(); j++)
			cout << "j:" << j << " corr_new_vec[j].size():" << corr_new_vec[j].size() << endl;
	}

	//show corr
	{
		//showing
		CPointVisualization<T_PointType> pv;
		pv.setWindowName("Pairs");
		pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[1], *cloud_src);
		pcl::copyPointCloud(*cloud_vec[0], *cloud_tgt);
		{
			Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
		}
		*cloud_show += *cloud_src;
		*cloud_show += *cloud_tgt;
		pv.setPointCloud(cloud_show);
		vector<std::uint8_t> color_vec;
		color_vec.push_back(100);
		color_vec.push_back(100);
		color_vec.push_back(100);
		cout << "corr_new.size():" << corr_new.size() << endl;
		//for (int j = corr_new.size() - 1; j >= 0; j--)
		//	if (j > 100) corr_new.erase(corr_new.begin() + j);
		cout << "corr_new.size():" << corr_new.size() << endl;
		int index_corr = 0;
		while (1)
		{
			if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_corr < corr_new_vec.size())
			{
				pcl::Correspondences corr_show;
				pv.drawCorrespondance(cloud_src, cloud_tgt, corr_new_vec[index_corr], color_vec);
				index_corr++;
			}

			if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
			pv.updateViewer();
		}
		pv.closeViewer();
	}

	//give color to pointcloud
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_colored(new pcl::PointCloud<T_PointType>());
		//pcl::copyPointCloud(*cloud_vec[j], *cloud_colored);
		for (int i = 0; i < index_valid_vecvec[j].size(); i++)
			cloud_colored->push_back(cloud_vec[j]->points[index_valid_vecvec[j][i]]);
		pcl::PointCloud<T_PointType>::Ptr cloud_ZValue(new pcl::PointCloud<T_PointType>());
		vector<float> feature_vec;
		for (int i = 0; i < index_valid_vecvec[j].size(); i++)
			feature_vec.push_back(featureDivergence_vecvec[j][index_valid_vecvec[j][i]]);

		for (int i = 0; i < index_valid_vecvec[j].size(); i++)
		{
			vector<std::uint8_t> color_vec;
			//color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[i], feature_max, feature_min);

			//if (b_useVelodyneFeature)
			//	color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], th_velodyne_max_color, th_velodyne_min_color);
			//else
			//	color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], th_nir_max_color, th_nir_min_color);
			if (b_useVelodyneFeature)
				color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], value_max_all, value_min_all);
			else
				color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], value_max_all, value_min_all);

			cloud_colored->points[i].r = color_vec[0];
			cloud_colored->points[i].g = color_vec[1];
			cloud_colored->points[i].b = color_vec[2];

			if (fabs(feature_vec[i]) == 0.)
			{
				cloud_colored->points[i].r = 255;
				cloud_colored->points[i].g = 255;
				cloud_colored->points[i].b = 255;
			}

			//if (b_useVelodyneFeature)
			//{
			//	if (th_velodyne_min_color > feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 100;
			//		cloud_colored->points[i].g = 100;
			//		cloud_colored->points[i].b = 100;
			//	}
			//	else if (th_velodyne_max_color < feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 255;
			//		cloud_colored->points[i].g = 255;
			//		cloud_colored->points[i].b = 0;
			//	}
			//}
			//else
			//{
			//	if (th_nir_min_color > feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 100;
			//		cloud_colored->points[i].g = 100;
			//		cloud_colored->points[i].b = 100;
			//	}
			//	else if (th_nir_max_color < feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 255;
			//		cloud_colored->points[i].g = 255;
			//		cloud_colored->points[i].b = 0;
			//	}
			//}
		}

		//give z by features
		pcl::copyPointCloud(*cloud_colored, *cloud_ZValue);
		*cloud_ZValue = *CKataokaPCL::getPointCloud_ZAaxisByFeature(cloud_ZValue, feature_vec, 10.);

		{
			Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(40., 0., 0., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_colored, *cloud_colored, trans_);
			*cloud_vec[j] += *cloud_colored;
		}

		{
			Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(40., 0., 15., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_ZValue, *cloud_ZValue, trans_);
			*cloud_vec[j] += *cloud_ZValue;
		}
	}

	//showing
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("Differential");

	int index_cloud = 0;
	while (1)
	{

		if (((GetAsyncKeyState(VK_SPACE) & 1) == 1) && (index_cloud != cloud_vec.size()))
		{
			pv.setPointCloud(cloud_vec[index_cloud], filenames_cloud[index_cloud]);
			cout << "index:" << index_cloud << endl;
			index_cloud++;
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;

		pv.updateViewer();
	}
	pv.closeViewer();

}

void CPointcloudFunction::DoDifferential_showFeatureValue(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	cout << "select feature:  NIR:0   Velodyne:1" << endl;
	bool b_useVelodyneFeature;
	cin >> b_useVelodyneFeature;

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	//remove 255 in nir
	if (!b_useVelodyneFeature)
	{
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			for (int i = cloud_vec[j]->size() - 1; i >= 0 ; i--)
			{
				if (cloud_vec[j]->points[i].r == 255)
					cloud_vec[j]->points.erase(cloud_vec[j]->points.begin() + i);
			}
		}
	}

	//feature
	vector<vector<float>> feature_vecvec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		vector<float> feature_vec;
		for (int i = 0; i < cloud_vec[j]->size(); i++)
		{
			if (!b_useVelodyneFeature)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			else
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
		}
		feature_vecvec.push_back(feature_vec);
	}
	
	//calc
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		cout << "j:" << j << endl;
		vector<float> feature_calcHistogram;
		for (int i = 0; i < feature_vecvec[j].size(); i++)
			feature_calcHistogram.push_back(feature_vecvec[j][i]);
		vector<int> hist_vec = CTimeString::getHistogram(feature_calcHistogram, 80, true);
	}

	float feature_all_min = std::numeric_limits<float>::max();
	float feature_all_max = -std::numeric_limits<float>::max();
	{
		cout << "show histogram of all features" << endl;
		vector<float> features_all;
		for (int j = 0; j < feature_vecvec.size(); j++)
			features_all.insert(features_all.end(), feature_vecvec[j].begin(), feature_vecvec[j].end());
		vector<float> feature_calcHistogram;

		for (int j = 0; j < features_all.size(); j++)
		{
			feature_calcHistogram.push_back(features_all[j]);
			if (feature_all_min > features_all[j]) feature_all_min = features_all[j];
			if (feature_all_max < features_all[j]) feature_all_max = features_all[j];

		}
		vector<int> hist_vec = CTimeString::getHistogram(feature_calcHistogram, 80, true);
	}

	//give color (and z by features) to pointcloud
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_colored(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[j], *cloud_colored);

		pcl::PointCloud<T_PointType>::Ptr cloud_ZValue(new pcl::PointCloud<T_PointType>());

		for (int i = 0; i < cloud_vec[j]->size(); i++)
		{
			vector<std::uint8_t> color_vec;
			color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vecvec[j][i], feature_all_max, feature_all_min);
			cloud_colored->points[i].r = color_vec[0];
			cloud_colored->points[i].g = color_vec[1];
			cloud_colored->points[i].b = color_vec[2];
		}

		//give z by features
		pcl::copyPointCloud(*cloud_colored, *cloud_ZValue);
		*cloud_ZValue = *CKataokaPCL::getPointCloud_ZAaxisByFeature(cloud_ZValue, feature_vecvec[j], 3.);

		{
			Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(40., 0., 0., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_colored, *cloud_colored, trans_);
			*cloud_vec[j] += *cloud_colored;

		}

		{
			Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(40., 0., 15., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_ZValue, *cloud_ZValue, trans_);
			*cloud_vec[j] += *cloud_ZValue;
		}

	}

	//showing
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("differential");

	int index_cloud = 0;
	while (1)
	{

		if (((GetAsyncKeyState(VK_SPACE) & 1) == 1) && (index_cloud != cloud_vec.size()))
		{
			pv.setPointCloud(cloud_vec[index_cloud], filenames_cloud[index_cloud]);
			cout << "index:" << index_cloud << endl;
			index_cloud++;

		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;

		pv.updateViewer();
	}
	pv.closeViewer();

}

void CPointcloudFunction::FPFH_unique(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	//debug
	//for (int j = cloud_vec.size() - 1; j >=1 ; j--)
	//	cloud_vec.erase(cloud_vec.begin() + j);

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;

	//calc normal
	//0.5
	//float voxel_size;
	//voxel_size = 0.1;

	float radius_normal_FPFH;
	radius_normal_FPFH = 0.5;

	const pcl::search::KdTree<T_PointType>::Ptr kdtree_(new pcl::search::KdTree<T_PointType>);
	const auto view_point = T_PointType(0.0, 10.0, 10.0);
	vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
		ne->setInputCloud(cloud_vec[j]);
		ne->setRadiusSearch(radius_normal_FPFH);
		ne->setSearchMethod(kdtree_);
		ne->setViewPoint(view_point.x, view_point.y, view_point.z);
		ne->compute(*normals);
		normals_vec.push_back(normals);
	}

	vector<vector<int>> index_vecvec;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	float radius_FPFH_center = 1.;
	index_vecvec = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
	for (int j = 0; j < index_vecvec.size(); j++)
	{
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_temp(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh_temp = CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center);
		vector<bool> b_valid_vec;
		b_valid_vec.resize(cloud_vec[j]->size());
		fill(b_valid_vec.begin(), b_valid_vec.end(), false);
		for (int i = 0; i < index_vecvec[j].size(); i++)
			b_valid_vec[index_vecvec[j][i]] = true;
		for (int i = index_vecvec[j].size() - 1; i >= 0; i--)
			if (!b_valid_vec[i]) fpfh_temp->erase(fpfh_temp->begin() + i);
		fpfh_vec.push_back(fpfh_temp);
	}

	pcl::Correspondences corrs_;
	{
		int i_tgt = 0;
		int i_src = 1;
		int num_near = 10;
		corrs_ = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
			index_vecvec[i_src], index_vecvec[i_tgt], num_near);

		for (int j = 0; j < corrs_.size(); j++)
		{
			if (j % 1000 != 0) continue;
			cout << "j:" << j << "  query:" << corrs_[j].index_query << " match:" << corrs_[j].index_match << " distance:" << corrs_[j].distance << endl;
		}

	}
}

void CPointcloudFunction::DoDifferential_RigidTransformation_FPFH_Features(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	{
		vector<vector<float>> feature_vecvec_nir;
		vector<vector<float>> feature_vecvec_velodyne;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			feature_vecvec_nir.push_back(feature_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
			feature_vecvec_velodyne.push_back(feature_vec);
		}
		//calc Laplacian
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
			kdtree_->setInputCloud(cloud_vec[j]);
			vector<float> featureDivergence_vec;
			featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
			kdtree_->setInputCloud(cloud_vec[j]);
			vector<float> featureDivergence_vec;
			featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
		}
		bool b_showHistogram = false;
		b_showHistogram = true;
		index_valid_vecvec_nir = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	}


	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	string ts_start = CTimeString::getTimeString();
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);
		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));

	}
	string ts_end = CTimeString::getTimeString();
	cout << "elapsed time:" << CTimeString::getTimeElapsefrom2Strings(ts_start, ts_end) << endl;

	pcl::Correspondences corrs_nir;
	pcl::Correspondences corrs_velodyne;
	pcl::Correspondences corrs_fpfh;

	int i_tgt;
	int i_src;
	i_tgt = 0;
	i_src = 1;
	//i_tgt = 5;
	//i_src = 6;
	int num_nearest = 10;
	float th_nearest_nir = 0.5;
	float th_nearest_velodyne = 0.05;
	float th_nearest_fpfh = 1800.;

	//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
	//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
	//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
	//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

	//corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
	//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
	corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

	//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
	//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove_value(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

	vector<pcl::Correspondences> corrs_vec;
	{
		pcl::Correspondences corrs_temp;
		//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
		corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
		//corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
		corrs_vec = CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
		//for (int j = 0; j < corrs_vec.size(); j++)
		//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	}

	//select biggest 10 corr
	{
		int num_select = 10;
		vector<vector<int>> index_biggest_vecvec;
		for (int j = 0; j < num_select; j++)
		{
			vector<int> index_biggest_vec;
			index_biggest_vec.push_back(0);
			index_biggest_vec.push_back(0);
			index_biggest_vecvec.push_back(index_biggest_vec);
		}
		for (int j = 0; j < corrs_vec.size(); j++)
		{
			vector<int> index_biggest_vec;
			index_biggest_vec.push_back(j);
			index_biggest_vec.push_back(corrs_vec[j].size());
			index_biggest_vecvec.push_back(index_biggest_vec);
			CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
			index_biggest_vecvec.pop_back();
		}
		vector<pcl::Correspondences> corrs_vec_new;
		for (int j = 0; j < index_biggest_vecvec.size(); j++)
			corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
		corrs_vec = corrs_vec_new;
		for (int j = 0; j < corrs_vec.size(); j++)
			cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	}

	//show corr
	{
		//showing
		CPointVisualization<T_PointType> pv;
		pv.setWindowName("Pairs");
		pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
		pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);
		{
			Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
				CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
		}
		*cloud_show += *cloud_src;
		*cloud_show += *cloud_tgt;
		pv.setPointCloud(cloud_show);
		vector<std::uint8_t> color_vec;
		color_vec.push_back(100);
		color_vec.push_back(100);
		color_vec.push_back(100);
		int index_corr = 0;
		while (1)
		{
			if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_corr < corrs_vec.size())
			{
				cout << "show corrs_vec[" << index_corr << "]" << endl;
				pcl::Correspondences corr_show;
				pv.drawCorrespondance(cloud_src, cloud_tgt, corrs_vec[index_corr], color_vec);
				for (int i = 0; i < corrs_vec[index_corr].size(); i++)
					cout << "i:" << i << " distance:" << corrs_vec[index_corr][i].distance << endl;
				cout << endl;
				index_corr++;
			}

			if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
			pv.updateViewer();
		}
		pv.closeViewer();
	}

	////transformation
	//for (int j = 0; j < corrs_vec.size(); j++)
	//{
	//	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	//	CKataokaPCL::estimateRigidTransformation_static(cloud_vec[i_src], cloud_vec[i_tgt], corrs_vec[j], transformation_matrix);

	//}

}

void CPointcloudFunction::DoDifferential_RigidTransformation_FPFH_Features_new(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	//{
	//	vector<vector<float>> feature_vecvec_nir;
	//	vector<vector<float>> feature_vecvec_velodyne;
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
	//		feature_vecvec_nir.push_back(feature_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
	//		feature_vecvec_velodyne.push_back(feature_vec);
	//	}
	//	//calc Laplacian
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
	//	}
	//	bool b_showHistogram = false;
	//	b_showHistogram = true;
	//	index_valid_vecvec_nir = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
	//	index_valid_vecvec_velodyne = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	//}


	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	//vector<Eigen::Matrix4f> transformation_vec;
	vector<pcl::Correspondences> corr_biggest_vec;
	vector<pair<int, int>> index_pair_vec;

	//calc
	{
		int i_tgt = 0;
		int i_src = 1;

		int num_nearest = 10;
		float th_nearest_nir = 0.5;
		float th_nearest_velodyne = 0.05;
		float th_nearest_fpfh = 1800.;
		index_pair_vec.push_back(make_pair(i_tgt, i_src));
		cout << "i_tgt:" << i_tgt << endl;
		cout << "i_src:" << i_src << endl;
		//cout << "fpfh_vec[i_src]->size():" << fpfh_vec[i_src]->size() << endl;
		//cout << "fpfh_vec[i_tgt]->size():" << fpfh_vec[i_tgt]->size() << endl;
		//{
		//	int i_max = 0;
		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_src].size(); j++)
		//		if (i_max < index_valid_vecvec_FPFH[i_src][j]) i_max = index_valid_vecvec_FPFH[i_src][j];
		//	cout << "i_src max:" << i_max << endl;
		//}
		//{
		//	int i_max = 0;
		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_tgt].size(); j++)
		//		if (i_max < index_valid_vecvec_FPFH[i_tgt][j]) i_max = index_valid_vecvec_FPFH[i_tgt][j];
		//	cout << "i_tgt max:" << i_max << endl;
		//}

		pcl::Correspondences corrs_nir;
		pcl::Correspondences corrs_velodyne;
		pcl::Correspondences corrs_fpfh;

		//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
		//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

		//corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
		//corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
		corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove_value(fpfh_vec[i_src], fpfh_vec[i_tgt],
			index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

		vector<pcl::Correspondences> corrs_vec;
		{
			pcl::Correspondences corrs_temp;
			//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
			//corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
			corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
			corrs_vec = CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
			//for (int j = 0; j < corrs_vec.size(); j++)
			//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
		}

		//select biggest 10 corr
		{
			int num_select = 10;
			vector<vector<int>> index_biggest_vecvec;
			for (int j = 0; j < num_select; j++)
			{
				vector<int> index_biggest_vec;
				index_biggest_vec.push_back(0);
				index_biggest_vec.push_back(0);
				index_biggest_vecvec.push_back(index_biggest_vec);
			}
			for (int j = 0; j < corrs_vec.size(); j++)
			{
				vector<int> index_biggest_vec;
				index_biggest_vec.push_back(j);
				index_biggest_vec.push_back(corrs_vec[j].size());
				index_biggest_vecvec.push_back(index_biggest_vec);
				CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
				index_biggest_vecvec.pop_back();
			}
			//if (corrs_vec.size() == 0) continue;
			vector<pcl::Correspondences> corrs_vec_new;
			for (int j = 0; j < index_biggest_vecvec.size(); j++)
				corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
			corrs_vec = corrs_vec_new;
			for (int j = 0; j < corrs_vec.size(); j++)
				cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
			corr_biggest_vec.push_back(corrs_vec[0]);
		}

	}

	cout << "show" << endl;

	//vector<pcl::Correspondences> corr_biggest_vec;
	//vector<pair<int, int>> index_pair_vec;
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("Pairs");
	pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

	vector<std::uint8_t> color_corr_vec;
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);

	int index_framePair = 0;
	while (1)
	{
		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corr_biggest_vec.size())
		{
			int i_tgt = index_pair_vec[index_framePair].first;
			int i_src = index_pair_vec[index_framePair].second;
			pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
			pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

			//showing
			{
				Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
					CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
				pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
			}
			*cloud_show += *cloud_src;
			*cloud_show += *cloud_tgt;
			pv.setPointCloud(cloud_show);
			vector<std::uint8_t> color_corr_vec;
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);

			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			pcl::Correspondences corr_show;
			pv.drawCorrespondance(cloud_src, cloud_tgt, corr_biggest_vec[index_framePair], color_corr_vec);
			for (int i = 0; i < corr_biggest_vec[index_framePair].size(); i++)
				cout << "i:" << i << " distance:" << corr_biggest_vec[index_framePair][i].distance << endl;
			cout << endl;
			index_framePair++;
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
		pv.updateViewer();

	}
	pv.closeViewer();

}

void CPointcloudFunction::DoDifferential_RigidTransformation_FPFH_Features_allFrames(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	//{
	//	vector<vector<float>> feature_vecvec_nir;
	//	vector<vector<float>> feature_vecvec_velodyne;
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
	//		feature_vecvec_nir.push_back(feature_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
	//		feature_vecvec_velodyne.push_back(feature_vec);
	//	}
	//	//calc Laplacian
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
	//	}
	//	bool b_showHistogram = false;
	//	b_showHistogram = true;
	//	index_valid_vecvec_nir = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
	//	index_valid_vecvec_velodyne = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	//}


	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	//vector<Eigen::Matrix4f> transformation_vec;
	vector<pcl::Correspondences> corr_biggest_vec;
	vector<pair<int, int>> index_pair_vec;
	for (int i_tgt = 0; i_tgt < cloud_vec.size(); i_tgt++)
	{
		for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
		{
			int num_nearest = 10;
			index_pair_vec.push_back(make_pair(i_tgt, i_src));
			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			//cout << "fpfh_vec[i_src]->size():" << fpfh_vec[i_src]->size() << endl;
			//cout << "fpfh_vec[i_tgt]->size():" << fpfh_vec[i_tgt]->size() << endl;
			//{
			//	int i_max = 0;
			//	for (int j = 0; j < index_valid_vecvec_FPFH[i_src].size(); j++)
			//		if (i_max < index_valid_vecvec_FPFH[i_src][j]) i_max = index_valid_vecvec_FPFH[i_src][j];
			//	cout << "i_src max:" << i_max << endl;
			//}
			//{
			//	int i_max = 0;
			//	for (int j = 0; j < index_valid_vecvec_FPFH[i_tgt].size(); j++)
			//		if (i_max < index_valid_vecvec_FPFH[i_tgt][j]) i_max = index_valid_vecvec_FPFH[i_tgt][j];
			//	cout << "i_tgt max:" << i_max << endl;
			//}

			pcl::Correspondences corrs_nir;
			pcl::Correspondences corrs_velodyne;
			pcl::Correspondences corrs_fpfh;

			//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
			//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
			//corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
			//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
			corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
				index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);

			vector<pcl::Correspondences> corrs_vec;
			{
				pcl::Correspondences corrs_temp;
				//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
				//corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
				corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
				corrs_vec = CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
				//for (int j = 0; j < corrs_vec.size(); j++)
				//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
			}

			//select biggest 10 corr
			{
				int num_select = 10;
				vector<vector<int>> index_biggest_vecvec;
				for (int j = 0; j < num_select; j++)
				{
					vector<int> index_biggest_vec;
					index_biggest_vec.push_back(0);
					index_biggest_vec.push_back(0);
					index_biggest_vecvec.push_back(index_biggest_vec);
				}
				for (int j = 0; j < corrs_vec.size(); j++)
				{
					vector<int> index_biggest_vec;
					index_biggest_vec.push_back(j);
					index_biggest_vec.push_back(corrs_vec[j].size());
					index_biggest_vecvec.push_back(index_biggest_vec);
					CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
					index_biggest_vecvec.pop_back();
				}
				if (corrs_vec.size() == 0) continue;
				vector<pcl::Correspondences> corrs_vec_new;
				for (int j = 0; j < index_biggest_vecvec.size(); j++)
					corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
				corrs_vec = corrs_vec_new;
				for (int j = 0; j < corrs_vec.size(); j++)
					cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
				corr_biggest_vec.push_back(corrs_vec[0]);
			}

		}
	}

	cout << "show" << endl;

	//vector<pcl::Correspondences> corr_biggest_vec;
	//vector<pair<int, int>> index_pair_vec;
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("Pairs");
	pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

	vector<std::uint8_t> color_corr_vec;
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);

	int index_framePair = 0;
	while (1)
	{
		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corr_biggest_vec.size())
		{
			int i_tgt = index_pair_vec[index_framePair].first;
			int i_src = index_pair_vec[index_framePair].second;
			pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
			pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

			//showing
			{
				Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
					CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
				pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
			}
			*cloud_show += *cloud_src;
			*cloud_show += *cloud_tgt;
			pv.setPointCloud(cloud_show);
			vector<std::uint8_t> color_corr_vec;
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);

			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			pcl::Correspondences corr_show;
			pv.drawCorrespondance(cloud_src, cloud_tgt, corr_biggest_vec[index_framePair], color_corr_vec);
			for (int i = 0; i < corr_biggest_vec[index_framePair].size(); i++)
				cout << "i:" << i << " distance:" << corr_biggest_vec[index_framePair][i].distance << endl;
			cout << endl;
			index_framePair++;
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
		pv.updateViewer();

	}
	pv.closeViewer();

}

void CPointcloudFunction::DoDifferential_PairEvaluation(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	{
		vector<vector<float>> feature_vecvec_nir;
		vector<vector<float>> feature_vecvec_velodyne;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			feature_vecvec_nir.push_back(feature_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
			feature_vecvec_velodyne.push_back(feature_vec);
		}
		//calc Laplacian
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
			kdtree_->setInputCloud(cloud_vec[j]);
			vector<float> featureDivergence_vec;
			featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
			kdtree_->setInputCloud(cloud_vec[j]);
			vector<float> featureDivergence_vec;
			featureDivergence_vec = CKataokaPCL::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
		}
		bool b_showHistogram = false;
		//b_showHistogram = true;
		index_valid_vecvec_nir = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CKataokaPCL::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	}


	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	//vector<Eigen::Matrix4f> transformation_vec;
	vector<pcl::Correspondences> corr_biggest_vec;
	vector<pair<int, int>> index_pair_vec;

	vector<pcl::Correspondences> corrs_nir_vec;
	vector<pcl::Correspondences> corrs_velodyne_vec;
	vector<pcl::Correspondences> corrs_fpfh_vec;

	vector<vector<pair<float, float>>> compare_nir_vecvec;
	vector<vector<pair<float, float>>> compare_velodyne_vecvec;
	vector<vector<pair<float, float>>> compare_fpfh_vecvec;

	{
		int i_tgt = 0;
		int i_src = i_tgt + 1;

		int num_nearest = 10;
		float th_nearest_nir = 0.5;
		float th_nearest_velodyne = 0.05;
		float th_nearest_fpfh = 1800.;
		index_pair_vec.push_back(make_pair(i_tgt, i_src));
		cout << "i_tgt:" << i_tgt << endl;
		cout << "i_src:" << i_src << endl;

		pcl::Correspondences corrs_nir;
		pcl::Correspondences corrs_velodyne;
		pcl::Correspondences corrs_fpfh;

		//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
		corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
			index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

		//corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
		corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
			index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove_value(fpfh_vec[i_src], fpfh_vec[i_tgt],
		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

		corrs_nir_vec.push_back(corrs_nir);
		corrs_velodyne_vec.push_back(corrs_velodyne);
		corrs_fpfh_vec.push_back(corrs_fpfh);

		vector<pair<float, float>> compare_nir_vec;
		vector<pair<float, float>> compare_velodyne_vec;
		vector<pair<float, float>> compare_fpfh_vec;

		compare_nir_vec = CKataokaPCL::calcCompareValueOfFeature_scalar(corrs_nir,
			cloud_vec[i_src], cloud_vec[i_tgt], featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
			index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);
		for (int j = 0; j < compare_nir_vec.size(); j++)
			cout << "j:" << j << "  query:" << compare_nir_vec[j].first << " match:" << compare_nir_vec[j].second << endl;

		compare_velodyne_vec = CKataokaPCL::calcCompareValueOfFeature_scalar(corrs_velodyne, 
			cloud_vec[i_src], cloud_vec[i_tgt], featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
			index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);
		for (int j = 0; j < compare_velodyne_vec.size(); j++)
			cout << "j:" << j << "  query:" << compare_velodyne_vec[j].first << " match:" << compare_velodyne_vec[j].second << endl;
	
		compare_nir_vecvec.push_back(compare_nir_vec);
		compare_velodyne_vecvec.push_back(compare_velodyne_vec);
	}

	//vector<pcl::Correspondences> corrs_nir_vec;
	//vector<pcl::Correspondences> corrs_velodyne_vec;
	//vector<pcl::Correspondences> corrs_fpfh_vec;

	//vector<vector<pair<float, float>>> compare_nir_vecvec;
	//vector<vector<pair<float, float>>> compare_velodyne_vecvec;
	//vector<vector<pair<float, float>>> compare_fpfh_vecvec;

	//ranking
	{
		vector<vector<float>> ranking_vecvec;
		for (int j = 0; j < corrs_nir_vec.size(); j++)
		{
			for (int i = 0; i < corrs_nir_vec[j].size(); i++)
			{
				vector<float> ranking_vec_query;
				ranking_vec_query.push_back((float)(j));						//frame
				ranking_vec_query.push_back((float)(i * 2 + 0));				//0:query, 1:match
				ranking_vec_query.push_back(compare_nir_vecvec[j][i].first);	//evaluation
				ranking_vecvec.push_back(ranking_vec_query);
				vector<float> ranking_vec_match;
				ranking_vec_match.push_back((float)(j));						//frame
				ranking_vec_match.push_back((float)(i * 2 + 1));				//0:query, 1:match
				ranking_vec_match.push_back(compare_nir_vecvec[j][i].second);	//evaluation
				ranking_vecvec.push_back(ranking_vec_match);
			}
		}
		CTimeString::sortVector2d(ranking_vecvec, 2);
		for (int j = 0; j < ranking_vecvec.size(); j++)
		{
			//if (j % 100 == 0)
			{
				cout << "j:" << j;
				cout << " frame:" << ranking_vecvec[j][0];
				cout << " corr_index:" << ((int)ranking_vecvec[j][1]) / 2;
				cout<<" query or match:"<< ((int)ranking_vecvec[j][1]) % 2;
				cout << " evaluation:" << ranking_vecvec[j][2] << endl;
			}
		}
		cout << endl;
	}
	//ranking
	{
		vector<vector<float>> ranking_vecvec;
		for (int j = 0; j < corrs_velodyne_vec.size(); j++)
		{
			for (int i = 0; i < corrs_velodyne_vec[j].size(); i++)
			{
				vector<float> ranking_vec_query;
				ranking_vec_query.push_back((float)(j));						//frame
				ranking_vec_query.push_back((float)(i * 2 + 0));					//0:query, 1:match
				ranking_vec_query.push_back(compare_velodyne_vecvec[j][i].first);	//evaluation
				ranking_vecvec.push_back(ranking_vec_query);
				vector<float> ranking_vec_match;
				ranking_vec_match.push_back((float)(j));						//frame
				ranking_vec_match.push_back((float)(i * 2 + 1));					//0:query, 1:match
				ranking_vec_match.push_back(compare_velodyne_vecvec[j][i].second);	//evaluation
				ranking_vecvec.push_back(ranking_vec_match);
			}
		}
		CTimeString::sortVector2d(ranking_vecvec, 2);
		for (int j = 0; j < ranking_vecvec.size(); j++)
		{
			//if (j % 100 == 0)
			{
				cout << "j:" << j;
				cout << " frame:" << ranking_vecvec[j][0];
				cout << " corr_index:" << ((int)ranking_vecvec[j][1]) / 2;
				cout << " query or match:" << ((int)ranking_vecvec[j][1]) % 2;
				cout << " evaluation:" << ranking_vecvec[j][2] << endl;
			}
		}
	}

	//for (int i_tgt = 0; i_tgt < cloud_vec.size(); i_tgt++)
	//{
	//	for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
	//	{
	//		int num_nearest = 10;
	//		float th_nearest_nir = 0.5;
	//		float th_nearest_velodyne = 0.05;
	//		float th_nearest_fpfh = 1800.;
	//		index_pair_vec.push_back(make_pair(i_tgt, i_src));
	//		cout << "i_tgt:" << i_tgt << endl;
	//		cout << "i_src:" << i_src << endl;

	//		pcl::Correspondences corrs_nir;
	//		pcl::Correspondences corrs_velodyne;
	//		pcl::Correspondences corrs_fpfh;

	//		//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
	//		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
	//		//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
	//		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

	//		//corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
	//		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
	//		corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove_value(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
	//			index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

	//		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
	//		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove_value(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

	//		vector<pair<float, float>> compare_nir_vec;
	//		vector<pair<float, float>> compare_velodyne_vec;
	//		vector<pair<float, float>> compare_fpfh_vec;

	//	}
	//}

	//for (int i_tgt = 0; i_tgt < cloud_vec.size(); i_tgt++)
	//{
	//	for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
	//	{
	//		int num_nearest = 10;
	//		index_pair_vec.push_back(make_pair(i_tgt, i_src));
	//		cout << "i_tgt:" << i_tgt << endl;
	//		cout << "i_src:" << i_src << endl;
	//		//cout << "fpfh_vec[i_src]->size():" << fpfh_vec[i_src]->size() << endl;
	//		//cout << "fpfh_vec[i_tgt]->size():" << fpfh_vec[i_tgt]->size() << endl;
	//		//{
	//		//	int i_max = 0;
	//		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_src].size(); j++)
	//		//		if (i_max < index_valid_vecvec_FPFH[i_src][j]) i_max = index_valid_vecvec_FPFH[i_src][j];
	//		//	cout << "i_src max:" << i_max << endl;
	//		//}
	//		//{
	//		//	int i_max = 0;
	//		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_tgt].size(); j++)
	//		//		if (i_max < index_valid_vecvec_FPFH[i_tgt][j]) i_max = index_valid_vecvec_FPFH[i_tgt][j];
	//		//	cout << "i_tgt max:" << i_max << endl;
	//		//}

	//		pcl::Correspondences corrs_nir;
	//		pcl::Correspondences corrs_velodyne;
	//		pcl::Correspondences corrs_fpfh;

	//		//corrs_nir = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
	//		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
	//		//corrs_velodyne = CKataokaPCL::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
	//		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
	//		corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//			index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);

	//		vector<pcl::Correspondences> corrs_vec;
	//		{
	//			pcl::Correspondences corrs_temp;
	//			//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
	//			//corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
	//			corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
	//			corrs_vec = CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
	//			//for (int j = 0; j < corrs_vec.size(); j++)
	//			//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	//		}

	//		//select biggest 10 corr
	//		{
	//			int num_select = 10;
	//			vector<vector<int>> index_biggest_vecvec;
	//			for (int j = 0; j < num_select; j++)
	//			{
	//				vector<int> index_biggest_vec;
	//				index_biggest_vec.push_back(0);
	//				index_biggest_vec.push_back(0);
	//				index_biggest_vecvec.push_back(index_biggest_vec);
	//			}
	//			for (int j = 0; j < corrs_vec.size(); j++)
	//			{
	//				vector<int> index_biggest_vec;
	//				index_biggest_vec.push_back(j);
	//				index_biggest_vec.push_back(corrs_vec[j].size());
	//				index_biggest_vecvec.push_back(index_biggest_vec);
	//				CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
	//				index_biggest_vecvec.pop_back();
	//			}
	//			if (corrs_vec.size() == 0) continue;
	//			vector<pcl::Correspondences> corrs_vec_new;
	//			for (int j = 0; j < index_biggest_vecvec.size(); j++)
	//				corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
	//			corrs_vec = corrs_vec_new;
	//			for (int j = 0; j < corrs_vec.size(); j++)
	//				cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	//			corr_biggest_vec.push_back(corrs_vec[0]);
	//		}

	//	}
	//}

	//cout << "show" << endl;

	////vector<pcl::Correspondences> corr_biggest_vec;
	////vector<pair<int, int>> index_pair_vec;
	//CPointVisualization<T_PointType> pv;
	//pv.setWindowName("Pairs");
	//pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
	//pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	//pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

	//vector<std::uint8_t> color_corr_vec;
	//color_corr_vec.push_back(100);
	//color_corr_vec.push_back(100);
	//color_corr_vec.push_back(100);

	//int index_framePair = 0;
	//while (1)
	//{
	//	if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corr_biggest_vec.size())
	//	{
	//		int i_tgt = index_pair_vec[index_framePair].first;
	//		int i_src = index_pair_vec[index_framePair].second;
	//		pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
	//		pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

	//		//showing
	//		{
	//			Eigen::Affine3f trans_ = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(
	//				CKataokaPCL::calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
	//			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
	//		}
	//		*cloud_show += *cloud_src;
	//		*cloud_show += *cloud_tgt;
	//		pv.setPointCloud(cloud_show);
	//		vector<std::uint8_t> color_corr_vec;
	//		color_corr_vec.push_back(100);
	//		color_corr_vec.push_back(100);
	//		color_corr_vec.push_back(100);

	//		cout << "i_tgt:" << i_tgt << endl;
	//		cout << "i_src:" << i_src << endl;
	//		pcl::Correspondences corr_show;
	//		pv.drawCorrespondance(cloud_src, cloud_tgt, corr_biggest_vec[index_framePair], color_corr_vec);
	//		for (int i = 0; i < corr_biggest_vec[index_framePair].size(); i++)
	//			cout << "i:" << i << " distance:" << corr_biggest_vec[index_framePair][i].distance << endl;
	//		cout << endl;
	//		index_framePair++;
	//	}

	//	if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
	//	pv.updateViewer();

	//}
	//pv.closeViewer();

}
