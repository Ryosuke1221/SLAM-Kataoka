#include "PointcloudFunction.h"

Eigen::Matrix4d CPointcloudFuction::calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
	double Roll_, double Pitch_, double Yaw_) 
{
	Eigen::Matrix4d	transformation_Position = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Roll_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Pitch_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Yaw_mat = Eigen::Matrix4d::Identity();
	T_mat(0, 3) = X_;
	T_mat(1, 3) = Y_;
	T_mat(2, 3) = Z_;
	Roll_mat(1, 1) = cos(Roll_);
	Roll_mat(1, 2) = -sin(Roll_);
	Roll_mat(2, 1) = sin(Roll_);
	Roll_mat(2, 2) = cos(Roll_);
	Pitch_mat(0, 0) = cos(Pitch_);
	Pitch_mat(2, 0) = -sin(Pitch_);
	Pitch_mat(0, 2) = sin(Pitch_);
	Pitch_mat(2, 2) = cos(Pitch_);
	Yaw_mat(0, 0) = cos(Yaw_);
	Yaw_mat(0, 1) = -sin(Yaw_);
	Yaw_mat(1, 0) = sin(Yaw_);
	Yaw_mat(1, 1) = cos(Yaw_);
	transformation_Position = T_mat * Yaw_mat * Pitch_mat * Roll_mat;
	return transformation_Position;
}

Eigen::Affine3f CPointcloudFuction::calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat)
{
	Eigen::Affine3f Trans_Affine = Eigen::Affine3f::Identity();
	Eigen::Vector6d Trans_Vec = Eigen::Vector6d::Identity();
	Trans_Vec = calcVector6dFromHomogeneousMatrix(input_Mat);
	Trans_Affine.translation() << Trans_Vec(0, 0), Trans_Vec(1, 0), Trans_Vec(2, 0);
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(5, 0), Eigen::Vector3f::UnitZ()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(4, 0), Eigen::Vector3f::UnitY()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(3, 0), Eigen::Vector3f::UnitX()));
	return Trans_Affine;
}

Eigen::Vector6d CPointcloudFuction::calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat) 
{
	Eigen::Vector6d XYZRPY = Eigen::Vector6d::Zero();
	double X_, Y_, Z_, Roll_, Pitch_, Yaw_;
	X_ = input_Mat(0, 3);
	Y_ = input_Mat(1, 3);
	Z_ = input_Mat(2, 3);
	if (input_Mat(2, 0) == -1.) {
		Pitch_ = M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(input_Mat(1, 2), input_Mat(1, 1));
	}
	else if (input_Mat(2, 0) == 1.) {
		Pitch_ = -M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(-input_Mat(1, 2), input_Mat(1, 1));
	}
	else {
		Yaw_ = atan2(input_Mat(1, 0), input_Mat(0, 0));
		Roll_ = atan2(input_Mat(2, 1), input_Mat(2, 2));
		double cos_Pitch;
		if (cos(Yaw_) == 0.) {
			cos_Pitch = input_Mat(0, 0) / sin(Yaw_);
		}
		else 	cos_Pitch = input_Mat(0, 0) / cos(Yaw_);

		Pitch_ = atan2(-input_Mat(2, 0), cos_Pitch);
	}
	if (!(-M_PI < Roll_)) Roll_ += M_PI;
	else if (!(Roll_ < M_PI)) Roll_ -= M_PI;
	if (!(-M_PI < Pitch_)) Pitch_ += M_PI;
	else if (!(Pitch_ < M_PI)) Pitch_ -= M_PI;
	if (!(-M_PI < Yaw_)) Yaw_ += M_PI;
	else if (!(Yaw_ < M_PI)) Yaw_ -= M_PI;

	XYZRPY << X_, Y_, Z_,
		Roll_, Pitch_, Yaw_;
	//cout << "Roll_ = " << Roll_ << endl;
	//cout << "Pitch_ = " << Pitch_ << endl;
	//cout << "Yaw_ = " << Yaw_ << endl;

	return XYZRPY;
}

void CPointcloudFuction::all_process()
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
		EN_GR_FPFH_SAC_IA
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
			GR_FPFH_SAC_IA();
			break;

		default:
			break;
		}
	
	}

	//cout << "process finished (press:ESC)" << endl;
	//GetAsyncKeyState(VK_ESCAPE);
	//while (1)
	//{
	//	if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
	//}

}

void CPointcloudFuction::show_sequent()
{
	string foldername_;
	foldername_ = "../../data/process_show_sequent";

	//typedef typename pcl::PointXYZI PointType_func;
	typedef typename pcl::PointXYZRGB PointType_func;

	bool b_useTXT = false;
	//b_useTXT = true;

	bool b_plane = true;
	b_plane = false;

	CPointVisualization<PointType_func> pv;
	if (typeid(PointType_func) == typeid(pcl::PointXYZI))
		pv.setWindowName("show XYZI");
	else if (typeid(PointType_func) == typeid(pcl::PointXYZRGB))
		pv.setWindowName("show XYZRGB");
	else
		throw std::runtime_error("This PointType is unsupported.");

	////show normal
	//pv.useNormal(0.3, 10, 0.1);

	vector<string> filenames_;
	CTimeString::getFileNames_extension(foldername_, filenames_,".pcd");
	cout << "file size: " << filenames_.size() << endl;

	if (filenames_.size() == 0)
	{
		cout << "ERROR: no file found" << endl;
		return ;
	}

	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<PointType_func>::Ptr cloud_(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_temp(new pcl::PointCloud<PointType_func>());

	//Eigen::Affine3f Trans_;
	//Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();

	int index_ = 0;

	vector<string> filename_use;

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
			pcl::io::loadPCDFile(foldername_ + "/" + filenames_[index_], *cloud_);
			cout << "showing:" << filenames_[index_] << endl;
			cout << "size: " << cloud_->size() << endl;

			pv.setPointCloud(cloud_);

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
		CTimeString::getCSVFromVecVec(save_vec_vec, foldername_ + "/_usePointCloud.csv");

}

void CPointcloudFuction::getPCDFromCSV_gotFromPCAP(string dir_save, string dir_data, string file_RelativePath_)
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


void CPointcloudFuction::getPCDFromCSV_naraha()
{
	string file_dir;
	//file_dir = "../../data/temp/02 velo&nir all frame";
	//file_dir = "../../data/temp";
	file_dir = "../../data/process_GetPcdFromCSV";

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

void CPointcloudFuction::FreeSpace()
{
	
}

void CPointcloudFuction::filterNIRPointCloud_naraha()
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::ApproximateVoxelGrid<pcl::PointXYZI> VGFilter;
	string dir_ = "../../data/process_FilterPointCloud";
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
		HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
		Trans_ = Eigen::Affine3f::Identity();
		Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
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
		HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., -pitch_init, 0.);
		Trans_ = Eigen::Affine3f::Identity();
		Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
		pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

		string filename_save = filenames_[index_].substr(0, filenames_[index_].size() - 4) + "_filtered_nir.pcd";
		pcl::io::savePCDFile<pcl::PointXYZI>(dir_ + "/" + filename_save, *cloud_);

	}
}

void CPointcloudFuction::getCSVFromPointCloud()
{
	cout << "start .csv method" << endl;
	string dir_;
	dir_ = "../../data/temp";
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

void CPointcloudFuction::HandRegistration()
{
	//"delta_T(i) = T(i-1).inverse() * T(i);",
	//because relative displacement can accumulate correctly.
	//When delta_T(i-1) changed, delta_T(i) should be able to work.

	//cout << "HandRegistration started!" << endl;
	//Sleep(1 * 1000);

	string dir_;
	//dir_ = "../../data/temp/_Hand";
	dir_ = "../../data/process_handregistration";

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
			HM_trajectory = calcHomogeneousMatrixFromVector6d(
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
			Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
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
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(resolution_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == Y_)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., resolution_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == Z_)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., resolution_translation, 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == ROLL_)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., resolution_rotation, 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == PITCH_)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., resolution_rotation, 0.)
				* HM_Trans_now;
		}
		else if (key_ == YAW_)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., resolution_rotation)
				* HM_Trans_now;
		}
		else if (key_ == X_MINUS)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(-resolution_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;

		}
		else if (key_ == Y_MINUS)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., -resolution_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == Z_MINUS)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., -resolution_translation, 0., 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == ROLL_MINUS)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., -resolution_rotation, 0., 0.)
				* HM_Trans_now;
		}
		else if (key_ == PITCH_MINUS)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., -resolution_rotation, 0.)
				* HM_Trans_now;
		}
		else if (key_ == YAW_MINUS)
		{
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., -resolution_rotation)
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
				cout << CKataokaPCL::getMedianDistance(*cloud_before, *cloud_moving) << endl;
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
					HM_Trans_now = calcHomogeneousMatrixFromVector6d(
						Registration_Vec(0, 0), Registration_Vec(1, 0), Registration_Vec(2, 0),
						Registration_Vec(3, 0), Registration_Vec(4, 0), Registration_Vec(5, 0))
						* HM_Trans_now;
					cout << "X:" << Registration_Vec(0, 0) << " Y:" << Registration_Vec(1, 0) << " Z:" << Registration_Vec(2, 0) << endl;
					cout << "Roll:" << Registration_Vec(3, 0) << " Pitch:" << Registration_Vec(4, 0) << " Yaw:" << Registration_Vec(5, 0) << endl;
				}
				else
				{
					HM_Trans_now = calcHomogeneousMatrixFromVector6d(
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
			cout << endl;
		}

		if (!(key_ == NONE || key_ == ENTER)) {
			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();
			Trans_ = calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
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
			State_ = calcVector6dFromHomogeneousMatrix(HM_free);
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

CPointcloudFuction::KEYNUM CPointcloudFuction::getKEYNUM()
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


void CPointcloudFuction::combinePointCloud_naraha()
{
	string dir_;
	dir_ = "../../data/process_CombinePointCloud";
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
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());

	int i_select;
	bool b_transform = true;
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

	cout << "select no_nir_frame or nir_frame" << endl;
	cout << "0: velodyne(no NIR), 1:velodyne(NIR)" << endl;
	cout << "i_select:";
	cin >> i_select;

	switch (i_select)
	{
	case 0:
		for (int index_ = 0; index_ < filenames_velo_nonir.size(); index_++)
		{
			cloud_velo->clear();
			cout << "reading:" << filenames_velo_nonir[index_] << endl;
			pcl::io::loadPCDFile(dir_ + "/" + subdir_ + "/" + filenames_velo_nonir[index_], *cloud_velo);

			if (b_transform)
			{
				//turn pitch(camera coordinate to robot one)
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_velo, *cloud_velo, Trans_);
				if (b_removeGround)
				{
					cloud_temp->clear();
					pcl::copyPointCloud(*cloud_velo, *cloud_temp);
					cloud_velo->clear();
					for (int i = 0; i < cloud_temp->size(); i++)
					{
						if (th_z_velo > cloud_temp->points[i].z) continue;
						cloud_velo->push_back(cloud_temp->points[i]);
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
			
			string filename_save = filenames_velo_nonir[index_].substr(0, 3) + "XYZRGB_naraha.pcd";
			pcl::io::savePCDFile<pcl::PointXYZRGB>
				(dir_  + "/" + filename_save, *cloud_save);
		}
		break;

	case 1:
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
				HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_velo, *cloud_velo, Trans_);
				if (b_removeGround)
				{
					cloud_temp->clear();
					pcl::copyPointCloud(*cloud_velo, *cloud_temp);
					cloud_velo->clear();
					for (int i = 0; i < cloud_temp->size(); i++)
					{
						if (th_z_velo > cloud_temp->points[i].z) continue;
						cloud_velo->push_back(cloud_temp->points[i]);
					}
				}

			}

			if (b_transform)
			{
				//turn pitch(camera coordinate to robot one)
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
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

			string filename_save = filenames_velo[index_].substr(0,3) +"XYZRGB_naraha.pcd";
			pcl::io::savePCDFile<pcl::PointXYZRGB>
				(dir_ + "/" + filename_save, *cloud_save);
		}
		break;
	}
}

void CPointcloudFuction::changeColor_plane(pcl::PointXYZRGB &point_)
{
	point_.r = 0;
	point_.g = 0;
	point_.b = 255;
}

void CPointcloudFuction::changeColor_plane(pcl::PointXYZI &point_)
{
	point_.intensity = 210;
}

void CPointcloudFuction::DynamicTranslation()
{
	//bool b_saveByTXT = false;
	bool b_inputTranslation = false;

	string dir_;
	dir_ = "../../data/temp/_DynamicTranslation";

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
				Trans_ = calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
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
				HM_Trans_now = calcHomogeneousMatrixFromVector6d(d_value_, 0., 0., 0., 0., 0.)
					* HM_Trans_now;
				break;
			case Y_vr:
				HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., d_value_, 0., 0., 0., 0.)
					* HM_Trans_now;
				break;
			case Z_vr:
				HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., d_value_, 0., 0., 0.)
					* HM_Trans_now;
				break;
			case Roll_vr:
				HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., d_value_ * M_PI / 180., 0., 0.)
					* HM_Trans_now;
				break;
			case Pitch_vr:
				HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., d_value_ * M_PI / 180., 0.)
					* HM_Trans_now;
				break;
			case Yaw_vr:
				HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., d_value_ * M_PI / 180.)
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
			Trans_ = calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
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

void CPointcloudFuction::FileProcess()
{

	string dir_;
	//dir_ = "../../data/temp/_DynamicTranslation";
	dir_ = "../../data";

	vector<string> filenames_folder;

	string s_index;

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
		{
			//filenames_folder

			vector<string> filenames_folder_temp;
			CTimeString::getFileNames_folder(dir_, filenames_folder_temp);

			////debug
			//cout << "size: " << filenames_folder_temp.size() << endl;

			int num_folder = 0;
			cout << endl;
			for (int i = 0; i < filenames_folder_temp.size(); i++)
			{
				filenames_folder.push_back(filenames_folder_temp[i]);
				s_index = to_string(num_folder);
				if (s_index.size() != 2) s_index = " " + s_index;
				cout << "i:" << s_index;
				cout << " " << filenames_folder_temp[i];
				vector<string> filenames_folder_folder;
				CTimeString::getFileNames_folder(dir_ + "/" + filenames_folder_temp[i], filenames_folder_folder);
				//if (filenames_folder_folder.size() == 0) cout << " (blank)" << endl;
				//else cout << endl;
				cout << endl;
				num_folder++;

				for (int j = 0; j < filenames_folder_folder.size(); j++)
				{
					filenames_folder.push_back(
						filenames_folder_temp[i] + "/" + filenames_folder_folder[j]);

					s_index = to_string(num_folder);
					if (s_index.size() != 2) s_index = " " + s_index;
					cout << "i:" << s_index;
					cout << " " << filenames_folder_temp[i] + "/" + filenames_folder_folder[j];
					vector<string> filenames_folder_folder_folder;
					CTimeString::getFileNames(
						dir_ + "/" + filenames_folder_temp[i] + "/" + filenames_folder_folder[j],
						filenames_folder_folder_folder,false,true,false);
					if (filenames_folder_folder_folder.size() == 0) cout << " (blank)";
					cout << endl;
					num_folder++;
				}
			}
		}

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

			cout << "copy to (index) ->";
			cin >> i_copy_to;
			//if (!(-1 < i_copy_to && i_copy_to < filenames_folder.size())) return;
			cout << i_copy_to << "(" << filenames_folder[i_copy_to] << ")" << endl;

			cout << "copy from (index) ->";
			cin >> i_copy_from;
			//if (!(-1 < i_copy_from && i_copy_from < filenames_folder.size())) return;
			cout << i_copy_from << "(" << filenames_folder[i_copy_from] << ")" << endl;

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

void CPointcloudFuction::FileProcess_copy(string dir_from, string dir_to)
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

void CPointcloudFuction::FileProcess_delete(string dir)
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

void CPointcloudFuction::FileProcess_evacuate(string dir)
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

void CPointcloudFuction::DrawTrajectory()
{
	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	//read file name
	string dir = "../../data/process_DrawTrajectory";
	vector<string> filenames_txt;
	CTimeString::getFileNames_extension(dir, filenames_txt, ".csv");

	if (filenames_txt.size() == 0)
	{
		cout << "ERROR: no file has found" << endl;
		return;
	}

	//select file
	cout << "select file:" << endl;
	int i_readfile = 0;
	for (int i = 0; i < filenames_txt.size(); i++)
		cout << " " << i << ": " << filenames_txt[i] << endl;
	cout << "->";
	cin >> i_readfile;

	//input trajectory
	vector<Eigen::Vector6d> trajectory_vec_vec;
	{
		vector<vector<double>> trajectory_temp;
		trajectory_temp = CTimeString::getVecVecFromCSV(dir + "/" + filenames_txt[i_readfile]);

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

	//bool read_sequently = true;
	//cout << "select: read_sequently  1:YES  0:NO" << endl;
	//cout << "->";
	//cin >> read_sequently;

	typedef typename CPointVisualization<T_PointType> CPV;
	CPV pv;
	pv.setWindowName("trajectory: " + filenames_txt[i_readfile]);

	cout << "show in one frame or frames  one frame:1  frames:0" << endl;
	cout << "->";
	bool b_mode_1frame;
	cin >> b_mode_1frame;

	//draw trajectory
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	for (int i = 0; i < trajectory_vec_vec.size(); i++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
		cloud_->clear();
		//draw arraw
		pcl::PointXYZRGB point_pose_arraw;
		point_pose_arraw.x = trajectory_vec_vec[i](0, 0);
		point_pose_arraw.y = trajectory_vec_vec[i](1, 0);
		point_pose_arraw.z = trajectory_vec_vec[i](2, 0);
		point_pose_arraw.r = 255;
		point_pose_arraw.g = 150;
		point_pose_arraw.b = 0;
		*cloud_ += *CPV::drawArrow(point_pose_arraw,
			trajectory_vec_vec[i](3, 0), trajectory_vec_vec[i](4, 0), trajectory_vec_vec[i](5, 0));
		//draw frame number
		pcl::PointXYZRGB point_frame;
		point_frame = point_pose_arraw;
		point_frame.z += 1.;
		point_frame.r = 255;
		point_frame.g = 255;
		point_frame.b = 0;
		*cloud_ += *pv.drawNumber(point_frame, i);
		if (i != 0)
		{
			//draw line
			pcl::PointXYZRGB point_pose_current;
			point_pose_current = point_pose_arraw;
			point_pose_current.r = 255;
			point_pose_current.g = 0;
			point_pose_current.b = 255;
			pcl::PointXYZRGB point_pose_before;
			point_pose_before = point_pose_current;
			point_pose_before.x = trajectory_vec_vec[i - 1](0, 0);
			point_pose_before.y = trajectory_vec_vec[i - 1](1, 0);
			point_pose_before.z = trajectory_vec_vec[i - 1](2, 0);
			*cloud_ += *CPV::drawLine(point_pose_before, point_pose_current);
		}
		cloud_vec.push_back(cloud_);
	}

	int index = 0;
	pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());

	if(!b_mode_1frame) cout << "Press SPACE then skip" << endl;
	while (1)
	{
		if (index < cloud_vec.size())
		{
			if (b_mode_1frame || (GetAsyncKeyState(VK_SPACE) & 1))
			{
				//cout << "index:" << index << endl;
				*cloud_show += *cloud_vec[index];
				index++;
			}
		}
		pv.setPointCloud(cloud_show);
		pv.updateViewer();
		if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
	}
	pv.closeViewer();
}

void CPointcloudFuction::DoSegmentation()
{
	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	// Read in the cloud data
	pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>);
	string filename_PC = "../../data/000XYZRGB_naraha.pcd";
	pcl::io::loadPCDFile(filename_PC, *cloud);

	// Create the filtering object: downsample the dataset
	pcl::ApproximateVoxelGrid<T_PointType> VGFilter;
	float leaf_ = 0.01;
	VGFilter.setInputCloud(cloud);
	VGFilter.setLeafSize(leaf_, leaf_, leaf_);
	VGFilter.filter(*cloud);

	detectPlane<T_PointType>(*cloud, 0.05, false, true);	//velo
	//detectPlane<T_PointType>(*cloud, 0.01, false, true);	//nir

	cout << "PointCloud size:" << cloud->size() << endl;


	vector < pcl::PointCloud<T_PointType>::Ptr > cloud_cluster_vec;
	pcl::PointCloud<T_PointType>::Ptr cloud_rest(new pcl::PointCloud<T_PointType>);
	//cloud_cluster_vec = getSegmentation(cloud, 3.);
	//cloud_cluster_vec = getSegmentation(cloud, 1.);
	//cloud_cluster_vec = getSegmentation(cloud, 0.5);
	//cloud_cluster_vec = getSegmentation_rest(cloud, cloud_rest, 0.5);
	cloud_cluster_vec = getSegmentation_rest(cloud, cloud_rest, 1.);

	//next clustering
	{
		vector < pcl::PointCloud<T_PointType>::Ptr > cloud_cluster_vec_next;
		pcl::PointCloud<T_PointType>::Ptr cloud_rest_next(new pcl::PointCloud<T_PointType>);
		cloud_cluster_vec_next = getSegmentation_rest(cloud_rest, cloud_rest_next, 2.);
		for (int i = 0; i < cloud_cluster_vec_next.size(); i++)
			cloud_cluster_vec.push_back(cloud_cluster_vec_next[i]);
		cloud_rest->clear();
		pcl::copyPointCloud(*cloud_rest_next, *cloud_rest);
	}

	cout << "cloud_cluster_vec.size(): " << cloud_cluster_vec.size() << endl;
	for (int i = 0; i < cloud_cluster_vec.size(); i++)
	{
		cout << "i:" << i << " cloud size:" << cloud_cluster_vec[i]->size() << endl;
	}

	//getRGBwithValuebyHSV
	for (int j = 0; j < cloud_cluster_vec.size(); j++)
	{
		vector<std::uint8_t> color_vec;
		color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(j, cloud_cluster_vec.size(), 0);
		cout << "j:" << j << " r:" << (int)color_vec[0] << " g:" << (int)color_vec[1] << " b:" << (int)color_vec[2] << endl;
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

	cout << "cloud size:" << cloud->size() << endl;
	cout << "cloud_sum size:" << cloud_sum->size() << endl;
	cout << "cloud_rest size:" << cloud_rest->size() << endl;

	CPointVisualization<T_PointType> pv;
	int index_cluster = 0;
	//cout << "Press SPACE then show PointCloud" << endl;
	//while (1)
	//{
	//	if ((GetAsyncKeyState(VK_SPACE) & 1) && (cloud_cluster_vec.size() - 1 > index_cluster) && cloud_cluster_vec.size() != 0)
	//	{
	//		pv.setPointCloud(cloud_cluster_vec[index_cluster]);
	//		cout << "showing index:" << index_cluster << " size:" << cloud_cluster_vec[index_cluster]->size() << endl;
	//		index_cluster++;
	//	}
	//	pv.updateViewer();
	//	if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
	//}

	cout << "show sum pointcloud" << endl;
	while (1)
	{
		if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
		pv.setPointCloud(cloud_sum);
		pv.updateViewer();
	}

	cout << "show rest pointcloud" << endl;
	while (1)
	{
		if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
		pv.setPointCloud(cloud_rest);
		pv.updateViewer();
	}

	pv.closeViewer();
}

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> CPointcloudFuction::getSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, double th_tolerance)
{
	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>);
	pcl::copyPointCloud(*cloud_arg, *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	std::vector<pcl::PointIndices> cluster_indices;
	cluster_indices = getSegmentation_indices(cloud_arg, th_tolerance);

	vector < pcl::PointCloud<T_PointType>::Ptr > cloud_cluster_vec;
	////old
	//int index_ = 0;
	//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	//{
	//	pcl::PointCloud<T_PointType>::Ptr cloud_cluster(new pcl::PointCloud<T_PointType>);
	//	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
	//		cloud_cluster->points.push_back(cloud->points[*pit]); //*
	//	cloud_cluster->width = cloud_cluster->points.size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;

	//	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
	//	//std::stringstream ss;
	//	//ss << "cloud_cluster_" << index_ << ".pcd";
	//	//writer.write<PointType_func>(ss.str(), *cloud_cluster, false); //*
	//	index_++;
	//	cloud_cluster_vec.push_back(cloud_cluster);
	//}

	//new
	for (int j = 0; j < cluster_indices.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_cluster(new pcl::PointCloud<T_PointType>);
		for (int i = 0; i < cluster_indices[j].indices.size(); i++)
			cloud_cluster->push_back(cloud->points[cluster_indices[j].indices[i]]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		cloud_cluster_vec.push_back(cloud_cluster);
	}

	return cloud_cluster_vec;
}

vector<pcl::PointIndices> CPointcloudFuction::getSegmentation_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, double th_tolerance)
{
	//https://vml.sakura.ne.jp/koeda/PCL/tutorials/html/cluster_extraction.html#cluster-extraction

	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>);
	pcl::copyPointCloud(*cloud_arg, *cloud);
	std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl; //*

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<T_PointType>::Ptr tree(new pcl::search::KdTree<T_PointType>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<T_PointType> ec;
	ec.setClusterTolerance(th_tolerance);
	ec.setMinClusterSize(100);	//threshold; distance of clusters 
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	return cluster_indices;
}

vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> CPointcloudFuction::getSegmentation_rest(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rest, double th_tolerance)
{
	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>);
	pcl::copyPointCloud(*cloud_arg, *cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	cluster_indices = getSegmentation_indices(cloud_arg, th_tolerance);

	vector<bool> b_rest_vec;

	for (int i = 0; i < cloud_arg->size(); i++)
		b_rest_vec.push_back(true);

	for (int j = 0; j < cluster_indices.size(); j++)
	{
		for (int i = 0; i < cluster_indices[j].indices.size(); i++)
		{
			b_rest_vec[cluster_indices[j].indices[i]] = false;
		}
	}

	vector < pcl::PointCloud<T_PointType>::Ptr > cloud_cluster_vec;

	//new
	for (int j = 0; j < cluster_indices.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_cluster(new pcl::PointCloud<T_PointType>);
		for (int i = 0; i < cluster_indices[j].indices.size(); i++)
			cloud_cluster->push_back(cloud->points[cluster_indices[j].indices[i]]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
		cloud_cluster_vec.push_back(cloud_cluster);
	}

	cloud_rest->clear();
	for (int i = 0; i < cloud_arg->size(); i++)
	{
		if (b_rest_vec[i] == true)
		{
			auto point = cloud_arg->points[i];
			cloud_rest->push_back(point);
		}
	}

	return cloud_cluster_vec;
}

void CPointcloudFuction::GR_FPFH_SAC_IA()
{
	//typedef pcl::PointXYZ T_PointType;
	typedef pcl::PointXYZRGB T_PointType;

	int i_method;
	i_method = 0;//registration of 2 cloud
	i_method = 1;//registration of all frames and output files
	//i_method = 2;//debug

	//0.15	0
	//0.1	29 / 50
	//0.125	1 / 50


	if (i_method == 0)
	{
		pcl::PointCloud<T_PointType>::Ptr scene1(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr scene2(new pcl::PointCloud<T_PointType>());
		pcl::io::loadPCDFile("../../data/008XYZRGB_naraha.pcd", *scene1);
		//pcl::io::loadPCDFile("../../data/008XYZRGB_naraha.pcd", *scene2);
		pcl::io::loadPCDFile("../../data/009XYZRGB_naraha.pcd", *scene2);

		if (typeid(T_PointType) == typeid(pcl::PointXYZRGB))
		{
			for (int i = 0; i < scene1->size(); i++)
			{
				scene1->points[i].r = 0;
				scene1->points[i].g = 0;
				scene1->points[i].b = 255;
			}

			for (int i = 0; i < scene2->size(); i++)
			{
				scene2->points[i].r = 255;
				scene2->points[i].g = 255;
				scene2->points[i].b = 0;
			}

		}

		// scene2 を適当に回転・並進
		Eigen::Matrix4f transform_matrix;
		transform_matrix <<
			1.0, 0.0, 0.0, -0.1,
			0.0, 0.0, -1.0, 0.1,
			0.0, 1.0, 0.0, -0.1,
			0.0, 0.0, 0.0, 1;
		//pcl::transformPointCloud(*scene2, *scene2, transform_matrix);

		//visualization
		pcl::PointCloud<T_PointType>::Ptr cloud_show_init(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_show_global(new pcl::PointCloud<T_PointType>());

		CPointVisualization<T_PointType> pv_init;
		pv_init.setWindowName("Initial");
		CPointVisualization<T_PointType> pv_global;
		pv_global.setWindowName("Global");

		*cloud_show_init += *scene1;
		*cloud_show_init += *scene2;

		float voxel_size;
		//voxel_size = 0.01;
		//voxel_size = 0.05;
		voxel_size = 0.1;

		float radius_normal_FPFH, radius_FPFH;
		radius_normal_FPFH = voxel_size * 2.0;
		radius_FPFH = voxel_size * 5.0;

		float MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
		MaxCorrespondenceDistance_SAC = voxel_size * 2.5;
		int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;
		//MaximumIterations_SAC = 500000;
		//MaximumIterations_SAC = 50;	//8 & 8
		//MaximumIterations_SAC = 1000;
		MaximumIterations_SAC = 500;
		//NumberOfSamples_SAC = 4;//8 & 8
		//NumberOfSamples_SAC = 10;
		NumberOfSamples_SAC = 100;
		//CorrespondenceRandomness_SAC = 2;
		CorrespondenceRandomness_SAC = 10;
		//SimilarityThreshold_SAC = 0.9f;
		SimilarityThreshold_SAC = 0.01f;
		//InlierFraction_SAC = 0.25f;
		InlierFraction_SAC = 0.15f;
		cout << "fill InlierFraction_SAC ->";
		cin >> InlierFraction_SAC;

		Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh_src = CKataokaPCL::computeFPFH<T_PointType>(*scene1, voxel_size, radius_normal_FPFH, radius_FPFH);
		fpfh_tgt = CKataokaPCL::computeFPFH<T_PointType>(*scene2, voxel_size, radius_normal_FPFH, radius_FPFH);

		cout << "RANSAC" << endl;
		int max_RANSAC = 50;
		int index_RANSAC = 0;
		int frame_failed = 0;
		vector<pair<float, Eigen::Matrix4d>> output_vec;

		bool b_hasConverged = false;
		vector<int> inlier_;
		float fitnessscore;

		b_hasConverged = CKataokaPCL::align_SAC_AI_RANSAC<T_PointType>(transform_, inlier_, fitnessscore, frame_failed,
			*scene1, *fpfh_src, *scene2, *fpfh_tgt,
			voxel_size, MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC, 
			MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC, max_RANSAC);

		//transform_ = CKataokaPCL::align_FPFH_SAC_AI<T_PointType>(*scene1, *scene2);

		Eigen::Affine3f Trans_ = Eigen::Affine3f::Identity();
		Trans_ = calcAffine3fFromHomogeneousMatrix(transform_);
		pcl::transformPointCloud(*scene1, *scene1, Trans_);
		*cloud_show_global += *scene1;
		*cloud_show_global += *scene2;

		pv_init.setPointCloud(cloud_show_init);
		pv_global.setPointCloud(cloud_show_global);

		while (1)
		{
			pv_init.updateViewer();
			pv_global.updateViewer();
			if (GetAsyncKeyState(VK_ESCAPE) & 1) break;
		}
		pv_init.closeViewer();
		pv_global.closeViewer();

		return;
	}
	else if (i_method == 1)
	{
		string dir_;
		dir_ = "../../data/process_GR_FPFH_SAC_IA";

		string time_start = CTimeString::getTimeString();
		string time_regular = time_start;
		cout << "time_start:" << time_start << endl;

		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

		//make new folder
		string s_newfoldername = time_start;
		CTimeString::makenewfolder(dir_, s_newfoldername);

		vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
		vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
		vector<vector<string>> s_output_vecvec;

		float voxel_size;
		//voxel_size = 0.01;
		//voxel_size = 0.05;
		voxel_size = 0.1;

		float radius_normal_FPFH, radius_FPFH;
		radius_normal_FPFH = voxel_size * 2.0;
		radius_FPFH = voxel_size * 5.0;

		float MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
		MaxCorrespondenceDistance_SAC = voxel_size * 2.5;
		int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;
		//MaximumIterations_SAC = 500000;
		//MaximumIterations_SAC = 50;	//8 & 8
		//MaximumIterations_SAC = 1000;
		MaximumIterations_SAC = 500;
		//NumberOfSamples_SAC = 4;//8 & 8
		//NumberOfSamples_SAC = 10;
		NumberOfSamples_SAC = 100;
		//CorrespondenceRandomness_SAC = 2;
		CorrespondenceRandomness_SAC = 10;
		//SimilarityThreshold_SAC = 0.9f;
		SimilarityThreshold_SAC = 0.01f;
		//InlierFraction_SAC = 0.25f;
		InlierFraction_SAC = 0.15f;

		int max_RANSAC = 50;

		//s_output_vecvec
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("Parameter");
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("voxel_size");
			s_temp_vec.push_back(""); s_temp_vec.push_back(""); s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(voxel_size));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("radius_normal_FPFH");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(radius_normal_FPFH));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("radius_FPFH");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(radius_FPFH));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("MaxCorrespondenceDistance_SAC");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(MaxCorrespondenceDistance_SAC));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("SimilarityThreshold_SAC");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(SimilarityThreshold_SAC));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("InlierFraction_SAC");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(InlierFraction_SAC));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("MaximumIterations_SAC");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(MaximumIterations_SAC));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("NumberOfSamples_SAC");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(NumberOfSamples_SAC));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("CorrespondenceRandomness_SAC");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(CorrespondenceRandomness_SAC));
			s_output_vecvec.push_back(s_temp_vec);
		}
		{
			vector<string> s_temp_vec;
			s_temp_vec.push_back("max_RANSAC");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back("");
			s_temp_vec.push_back(to_string(max_RANSAC));
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

		for (int i = 0; i < filenames_.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + filenames_[i], *cloud);
			cloud_vec.push_back(cloud);
		}

		for (int i = 0; i < filenames_.size(); i++)
		{
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
			fpfh = CKataokaPCL::computeFPFH<T_PointType>(*cloud_vec[i], voxel_size, radius_normal_FPFH, radius_FPFH);
			fpfh_vec.push_back(fpfh);
		}

		string time_end_FPFH = CTimeString::getTimeString();
		cout << "time_end_FPFH:" << time_end_FPFH << endl;

		const int i_tgt_start = 0;
		//const int i_src_start = 1;

		for (int i_tgt = i_tgt_start; i_tgt < cloud_vec.size() - 1; i_tgt++)
		{
			for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
			{
				if (i_tgt == i_src) continue;

				string time_start_frame = CTimeString::getTimeString();

				vector<pair<float, Eigen::Matrix4d>> output_vec;

				bool b_hasConverged = false;
				vector<int> inlier_;
				float fitnessscore;
				int frame_failed = 0;
				Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();

				cout << "i_tgt:" << i_tgt << " i_src" << i_src << endl;

				b_hasConverged = CKataokaPCL::align_SAC_AI_RANSAC<T_PointType>(transform_, inlier_, fitnessscore, frame_failed,
					*cloud_vec[i_src], *fpfh_vec[i_src], *cloud_vec[i_tgt], *fpfh_vec[i_tgt],
					voxel_size, MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC,	InlierFraction_SAC,
					MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC, max_RANSAC);

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
				s_temp_vec.push_back(to_string((float)inlier_.size()/ (float)fpfh_vec[i_src]->size()));
				s_temp_vec.push_back(to_string((int)b_hasConverged));
				s_temp_vec.push_back(to_string(max_RANSAC - frame_failed) + "(/" + to_string(max_RANSAC) + ")");
				s_temp_vec.push_back(to_string(fitnessscore));
				s_temp_vec.push_back(time_elapsed_frame);
				s_temp_vec.push_back(to_string(transform_vec(0, 0)));	//X
				s_temp_vec.push_back(to_string(transform_vec(1, 0)));	//Y
				s_temp_vec.push_back(to_string(transform_vec(2, 0)));	//Z
				s_temp_vec.push_back(to_string(transform_vec(3, 0)));	//ROLL
				s_temp_vec.push_back(to_string(transform_vec(4, 0)));	//PITCH
				s_temp_vec.push_back(to_string(transform_vec(5, 0)));	//YAW

				s_output_vecvec.push_back(s_temp_vec);

				//save pointcloud
				pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
				pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
				//VGF
				const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
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
				{
					Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
					Trans_temp = calcAffine3fFromHomogeneousMatrix(transform_);
					pcl::transformPointCloud(*cloud_src, *cloud_src, Trans_temp);
				}
				//add
				*cloud_tgt += *cloud_src;
				//filename
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
					s_filename_output = "t" + s_tgt + "s" + s_src + "XYZRGB.pcd";
				}
				//save
				pcl::io::savePCDFile<T_PointType>(dir_ + "/" + s_newfoldername + "/" + s_filename_output, *cloud_tgt);
				cout << endl;
				cout << endl;

				//regular saving csv
				{
					int elapsed_millisec = CTimeString::getTimeElapsefrom2Strings_millisec(time_regular, time_end_frame);
					int elapsed_minute =(int) (((float)elapsed_millisec / 1000.) / 60.);
					const int th_minute = 5;
					if (elapsed_minute >= th_minute)
					{
						//save
						CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_regular + "_output.csv");
						time_regular = CTimeString::getTimeString();
						//clear s_output_vecvec
						s_output_vecvec.clear();
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
				}
				//time_regular, time_end_frame
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

		//CTimeString::getCSVFromVecVec(s_output_vecvec, "../../data/process_GR_FPFH_SAC_IA/"+ time_start+ "_output.csv");
		CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_end + "_output.csv");

		return;
	}
	else if (i_method == 2)
	{
		string aa;
		vector<vector<string>> s_vecvec;

		for (int j = 0; j < 3 ; j++)
		{
			vector<string> s_vec;
			s_vec.push_back("A");
			s_vec.push_back("B");
			s_vec.push_back("C");
			s_vecvec.push_back(s_vec);
		}

		CTimeString::getCSVFromVecVec(s_vecvec,"../../data/process_GR_FPFH_SAC_IA/test.csv");

	}


 

	
}