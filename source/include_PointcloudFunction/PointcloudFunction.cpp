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
		EN_SequentShow,
		EN_handregistration,
		EN_GetPcdFromCSV,
		EN_FilterPointCloud,
		EN_CombinePointCloud,
		EN_CSV_FromPointCloud
	};

	while (!b_finish)
	{
		cout << endl;
		cout << "please input process number" << endl;
		cout << EN_escape << ": escape" << endl;
		cout << EN_FreeSpace << ": free space" << endl;
		cout << EN_SequentShow << ": sequent show" << endl;
		cout << EN_handregistration << ": hand registration" << endl;
		cout << EN_GetPcdFromCSV << ": get .pcd from .csv" << endl;
		cout << EN_FilterPointCloud << ": filter PointCloud_naraha" << endl;
		cout << EN_CombinePointCloud << ": CombinePointCloud" << endl;
		cout << EN_CSV_FromPointCloud << ": CSV_FromPointCloud" << endl;

		cout <<"WhichProcess: ";
		cin >> WhichProcess;
		switch (WhichProcess)
		{
		case EN_escape:
			//escape
			b_finish = true;
			break;

		case EN_FreeSpace:
			FreeSpace();
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
	//foldername_ = "../../data/temp";
	foldername_ = "../../data/temp/_XYZRGB";

	//typedef typename pcl::PointXYZI PointType_func;
	typedef typename pcl::PointXYZRGB PointType_func;

	bool b_useTXT = false;
	//b_useTXT = true;

	CPointVisualization<PointType_func> pv;
	if (typeid(PointType_func) == typeid(pcl::PointXYZI))
		pv.setWindowName("show XYZI");
	else if (typeid(PointType_func) == typeid(pcl::PointXYZRGB))
		pv.setWindowName("show XYZRGB");
	else
		throw std::runtime_error("This PointType is unsupported.");

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


			//remove ground plane
			if (cloud_->size() != 0)
			{
				detectPlane<PointType_func>(*cloud_);

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

		pv.setPointCloud(cloud_);
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

void CPointcloudFuction::moveFile()
{
	//string foldername_;
	//foldername_ = "../../data/temp";
	//vector<string> filenames_;

	////{
	////	vector<vector<string>>
	////}
	////CTimeString::getVecVecFromCSV();

	//CPointVisualization<pcl::PointXYZI> pv;
	//pv.setWindowName("show XYZI");

	//CTimeString::getFileNames_extension(foldername_, filenames_, ".pcd");
	//cout << "file size: " << filenames_.size() << endl;

	//if (filenames_.size() == 0)
	//{
	//	cout << "ERROR: no file found" << endl;
	//	return;
	//}

	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());

	////Eigen::Affine3f Trans_;
	////Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();

	//int index_ = 0;

	//vector<string> filename_use;

	//while (1)
	//{
	//	//short key_num = GetAsyncKeyState(VK_SPACE);
	//	if ((GetAsyncKeyState(VK_SPACE) & 1) == 1)
	//	{
	//		if (index_ == filenames_.size())
	//		{
	//			cout << "index over" << endl;
	//			break;
	//		}

	//		cout << "index_: " << index_ << endl;
	//		//cout << "reading:" << filenames_[index_] << endl;
	//		pcl::io::loadPCDFile(foldername_ + "/" + filenames_[index_], *cloud_);
	//		cout << "showing:" << filenames_[index_] << endl;

	//		index_++;

	//	}

	//	//save
	//	if ((GetAsyncKeyState(VK_RETURN) & 1) == 1)
	//	{
	//		filename_use.push_back(filenames_[index_ - 1]);
	//		cout << "add: " << filenames_[index_ - 1] << endl;
	//	}


	//	//escape
	//	//short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
	//	if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) {
	//		cout << "toggled!" << endl;
	//		break;
	//	}

	//	pv.setPointCloud(cloud_);
	//	pv.updateViewer();

	//}

	//pv.closeViewer();

	//for (int i = 0; i < filename_use.size(); i++)
	//{
	//	//cout << "file " << i << ": " << filename_use[i] << endl;
	//	cout << filename_use[i] << endl;
	//}

	//vector<vector<string>> save_vec_vec;
	//for (int i = 0; i < filename_use.size(); i++)
	//{
	//	vector<string> save_vec;
	//	save_vec.push_back(filename_use[i]);
	//	save_vec_vec.push_back(save_vec);
	//}
	//CTimeString::getCSVFromVecVec(save_vec_vec, foldername_ + "/_usePointCloud.csv");
}

void CPointcloudFuction::getPCDFromCSV_gotFromPCAP(string dir_, string file_RelativePath_)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
	vector<vector<string>> csv_vec_vec_string;
	csv_vec_vec_string = CTimeString::getVecVecFromCSV_string(dir_ + "/" + file_RelativePath_);
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
	pcl::io::savePCDFile<pcl::PointXYZI>(dir_ + "/_pointcloud/" + filename_, *cloud_);
	cout << "saved: " << filename_ << endl;
}


void CPointcloudFuction::getPCDFromCSV_naraha()
{
	string file_dir;
	file_dir = "../../data/temp/02 velo&nir all frame";
	//file_dir = "../../data/temp";
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
		CTimeString::getFileNames_extension(file_dir, filenames, ").csv");
		for (int i = 0; i < filenames.size(); i++)
		{
			cout << "i:" << i << " calc " << file_dir + "/" + filenames[i] << "..." << endl;
			getPCDFromCSV_gotFromPCAP(file_dir, filenames[i]);
		}

		break;
	case 1:
		CTimeString::getFileNames_extension(file_dir, filenames, "velo.csv");
		for (int i = 0; i < filenames.size(); i++)
		{
			cout << "i:" << i << " calc " << file_dir + "/" + filenames[i] << "..." << endl;
			vector<vector<string>> csv_vec_vec_string;
			csv_vec_vec_string = CTimeString::getVecVecFromCSV_string(file_dir + "/" + filenames[i], " ");
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
			pcl::io::savePCDFile<pcl::PointXYZI>(file_dir + "/_pointcloud/" + filename_, *cloud_);
			cout << "saved: " << filename_ << endl;
		}

		break;
	case 2:
		CTimeString::getFileNames_extension(file_dir, filenames, "nir.csv");
		float max_, min_;
		min_ = 255.;
		max_ = 0.;

		for (int i = 0; i < filenames.size(); i++)
		{
			cout << "i:" << i << " calc " << file_dir + "/" + filenames[i] << "..." << endl;
			vector<vector<string>> csv_vec_vec_string;
			csv_vec_vec_string = CTimeString::getVecVecFromCSV_string(file_dir + "/" + filenames[i], " ");
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
			pcl::io::savePCDFile<pcl::PointXYZI>(file_dir + "/_pointcloud/" + filename_, *cloud_);
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
	string dir_ = "../../data/temp";
	double th_VGF = 0.01;

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
	CTimeString::getFileNames_extension(dir_, filenames_, "nir.pcd");

	double pitch_init;
	pitch_init = 22. * M_PI / 180.;

	//area filter
	for (int index_ = 0; index_ < filenames_.size(); index_++)
	{
		cout << "reanding: " << filenames_[index_] << endl;
		if (-1 == pcl::io::loadPCDFile(dir_ + "/" + filenames_[index_], *cloud_))
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
		pcl::io::savePCDFile<pcl::PointXYZI>(dir_ + "/_filtered/" + filename_save, *cloud_);

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

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>());
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
			data_vec.push_back(cloud_->points[i].intensity);
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
	dir_ = "../../data/temp/_Hand";

	bool b_RemoveGround = true;
	//b_RemoveGround = false;


	//typedef typename pcl::PointXYZI PointType_func;
	typedef typename pcl::PointXYZRGB PointType_func;


	CPointVisualization<PointType_func> pv;

	if (typeid(PointType_func) == typeid(pcl::PointXYZI))
		pv.setWindowName("show XYZI");
	else if (typeid(PointType_func) == typeid(pcl::PointXYZRGB))
		pv.setWindowName("show XYZRGB");
	else
		throw std::runtime_error("This PointType is unsupported.");

	pcl::PointCloud<PointType_func>::Ptr cloud_show(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_show_static(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_moving(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_moving_before(new pcl::PointCloud<PointType_func>());
	pcl::PointCloud<PointType_func>::Ptr cloud_temp(new pcl::PointCloud<PointType_func>());

	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d HM_Trans_now = Eigen::Matrix4d::Identity();

	double disp_translation = 0.;
	double disp_rotation = 0.;
	disp_translation = 0.05;
	disp_rotation = 0.5 * M_PI / 180.;

	int index_PC_now = 0;
	bool b_makeNewPC = true;
	bool b_first = true;
	bool b_escaped = false;
	bool b_break = false;

	enum KEYNUM {
		NONE,
		UP,
		DOWN,
		RIGHT,
		LEFT,
		TURN_R,
		TURN_L,
		ENTER,
		SUBTRACT
	};
	KEYNUM key_;

	vector<string> filenames_;
	CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");

	std::string filename_txt;

	vector<Eigen::Matrix4d>	HM_displacement_vec;

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

			cloud_moving_before->clear();

			string filename_PC;
			filename_PC = filenames_[index_PC_now];
			filename_PC = dir_ + "/" + filename_PC;

			if (-1 == pcl::io::loadPCDFile(filename_PC, *cloud_moving_before)) break;

			cout << "PC(" << index_PC_now << ") number :" << cloud_moving_before->size() << endl;

			cout << endl;
			cout << "**********( key option )**********" << endl;
			cout << "Use numpad" << endl;
			cout << " TURN_LEFT:7    UP:8  TURN_RIGHT:9" << endl;
			cout << "      LEFT:4    ----       RIGHT:6" << endl;
			cout << "      ------  DOWN:2       -------" << endl;

			cout << endl;
			cout << "Reset:-(numpad)" << endl;

			cout << "Switch:ENTER" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;

			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();

			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i <= index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_Trans_now = HM_free;
			Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
			pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);

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

		//https://www.slideshare.net/masafuminoda/pcl-11030703
		//Viewer
		//left drag：rotation of view point
		//Shift+left drag：translation of view point
		//Ctrl+left drag：rotation in display
		//right drag：zoom
		//g：display measure
		//j：save screenshot

		//input key
		short key_num_up = GetAsyncKeyState(VK_NUMPAD8);
		short key_num_down = GetAsyncKeyState(VK_NUMPAD2);
		short key_num_right = GetAsyncKeyState(VK_NUMPAD6);
		short key_num_left = GetAsyncKeyState(VK_NUMPAD4);
		short key_num_turn_r = GetAsyncKeyState(VK_NUMPAD9);
		short key_num_turn_l = GetAsyncKeyState(VK_NUMPAD7);
		short key_num_enter = GetAsyncKeyState(VK_RETURN);
		short key_num_escape = GetAsyncKeyState(VK_ESCAPE);
		short key_num_subt_numpad = GetAsyncKeyState(VK_SUBTRACT);

		if ((key_num_up & 1) == 1) key_ = UP;
		else if ((key_num_down & 1) == 1) key_ = DOWN;
		else if ((key_num_right & 1) == 1) key_ = RIGHT;
		else if ((key_num_left & 1) == 1) key_ = LEFT;
		else if ((key_num_turn_r & 1) == 1) key_ = TURN_R;
		else if ((key_num_turn_l & 1) == 1) key_ = TURN_L;
		else if ((key_num_enter & 1) == 1) key_ = ENTER;
		else if ((key_num_subt_numpad & 1) == 1) key_ = SUBTRACT;
		else if ((key_num_escape & 1) == 1)
		{
			cout << "ESC called" << endl;
			b_escaped = true;
			break;
		}
		else key_ = NONE;

		if (b_first)
		{
			key_ = NONE;
			b_first = false;
		}

		//determine transformation by key input
		switch (key_) {
		case UP:
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., disp_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case DOWN:
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., -disp_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case RIGHT:
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(disp_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case LEFT:
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(-disp_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case TURN_R:
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., -disp_rotation)
				* HM_Trans_now;
			break;

		case TURN_L:
			HM_Trans_now = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., disp_rotation)
				* HM_Trans_now;
			break;

		case ENTER:
			*cloud_show_static += *cloud_moving;
			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i < index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_displacement_vec[index_PC_now] = HM_free.inverse() * HM_Trans_now;
			index_PC_now++;
			if (index_PC_now == HM_displacement_vec.size()) b_break = true;
			b_makeNewPC = true;
			cout << "ENTER pressed" << endl;
			break;

		case SUBTRACT:
			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i <= index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_Trans_now = HM_free;
			cout << "-(numpad) pressed" << endl;
			break;

		default:
			break;
		}

		if (!(key_ == NONE || key_ == ENTER)) {
			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();
			Trans_ = calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
			pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);
		}
		cloud_show->clear();
		*cloud_show += *cloud_show_static;
		*cloud_show += *cloud_moving;

		if (cloud_show->size() != 0) {
			pv.setPointCloud(cloud_show);
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
}

void CPointcloudFuction::combinePointCloud_naraha()
{
	string dir_;
	dir_ = "../../data/temp";
	string dir_save_relativePath;
	dir_save_relativePath = "_XYZRGB";
	vector<string> filenames_velo_nonir;
	vector<string> filenames_velo;
	vector<string> filenames_nir;
	//CTimeString::getFileNames_extension(file_dir, filenames_, "nir.pcd");
	CTimeString::getFileNames_extension(dir_, filenames_velo_nonir, ").pcd");
	CTimeString::getFileNames_extension(dir_, filenames_velo, "velo.pcd");
	CTimeString::getFileNames_extension(dir_, filenames_nir, "nir.pcd");

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
	pitch_init = 24. * M_PI / 180.;
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
			pcl::io::loadPCDFile(dir_ + "/" + filenames_velo_nonir[index_], *cloud_velo);

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
				point_.g = cloud_velo->points[i].intensity;
				point_.b = 0;
				cloud_save->push_back(point_);
			}
			
			string filename_save = filenames_velo_nonir[index_].substr(0, 3) + "XYZRGB_naraha.pcd";
			pcl::io::savePCDFile<pcl::PointXYZRGB>
				(dir_ + "/" + dir_save_relativePath + "/" + filename_save, *cloud_save);
		}
		break;

	case 1:
		for (int index_ = 0; index_ < filenames_velo.size(); index_++)
		{
			cloud_velo->clear();
			cout << "reading:" << filenames_velo[index_] << endl;
			pcl::io::loadPCDFile(dir_ + "/" + filenames_velo[index_], *cloud_velo);
			cloud_nir->clear();
			cout << "reading:" << filenames_nir[index_] << endl;
			pcl::io::loadPCDFile(dir_ + "/" + filenames_nir[index_], *cloud_nir);

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
				point_.g = cloud_velo->points[i].intensity;
				point_.b = 0;
				cloud_save->push_back(point_);
			}

			for (int i = 0; i < cloud_nir->size(); i++)
			{
				pcl::PointXYZRGB point_;
				point_.x = cloud_nir->points[i].x;
				point_.y = cloud_nir->points[i].y;
				point_.z = cloud_nir->points[i].z;
				point_.r = cloud_nir->points[i].intensity;
				point_.g = 0;
				point_.b = 0;
				cloud_save->push_back(point_);
			}

			string filename_save = filenames_velo[index_].substr(0,3) +"XYZRGB_naraha.pcd";
			pcl::io::savePCDFile<pcl::PointXYZRGB>
				(dir_ + "/" + dir_save_relativePath + "/" + filename_save, *cloud_save);
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

	//typedef typename pcl::PointXYZI PointType_func;
	typedef typename pcl::PointXYZRGB PointType_func;


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

	double disp_translation = 0.;
	double disp_rotation = 0.;
	disp_translation = 0.05;
	disp_rotation = 0.5 * M_PI / 180.;

	int index_PC_now = 0;
	bool b_makeNewPC = true;
	bool b_first = true;
	bool b_escaped = false;
	bool b_break = false;

	enum KEYNUM {
		NONE,
		//UP,
		//DOWN,
		//RIGHT,
		//LEFT,
		//TURN_R,
		//TURN_L,
		TRANSLATION,
		ENTER,
		SUBTRACT
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
			filename_PC = dir_ + "/" + filename_PC;

			if (-1 == pcl::io::loadPCDFile(filename_PC, *cloud_moving_before)) break;
			pcl::copyPointCloud(*cloud_moving_before, *cloud_moving);
			if (-1 == pcl::io::loadPCDFile(filename_PC, *cloud_moving)) break;
			pcl::copyPointCloud(*cloud_moving, *cloud_moving_before);

			cout << "PC(" << index_PC_now << ") number :" << cloud_moving->size() << endl;

			cout << endl;
			cout << "**********( key option )**********" << endl;
			//cout << "Use numpad" << endl;
			//cout << " TURN_LEFT:7    UP:8  TURN_RIGHT:9" << endl;
			//cout << "      LEFT:4    ----       RIGHT:6" << endl;
			//cout << "      ------  DOWN:2       -------" << endl;
			//cout << endl;

			cout << "Reset:-(numpad)" << endl;
			cout << "Switch:ENTER" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;

			//cloud_moving->clear();
			//Trans_ = Eigen::Affine3f::Identity();

			//HM_free = Eigen::Matrix4d::Identity();
			//for (int i = 0; i <= index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			//HM_Trans_now = HM_free;
			//Trans_ = calcAffine3fFromHomogeneousMatrix(HM_free);
			//pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);

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

		short key_num_enter = GetAsyncKeyState(VK_RETURN);
		short key_num_escape = GetAsyncKeyState(VK_ESCAPE);
		short key_num_subt_numpad = GetAsyncKeyState(VK_SUBTRACT);
		short key_num_t = GetAsyncKeyState(0x54);//T

		if ((key_num_enter & 1) == 1) key_ = ENTER;
		else if ((key_num_subt_numpad & 1) == 1) key_ = SUBTRACT;
		else if ((key_num_t & 1) == 1) key_ = TRANSLATION;
		else if ((key_num_escape & 1) == 1)
		{
			cout << "ESC called" << endl;
			b_escaped = true;
			break;
		}
		else key_ = NONE;

		if (b_first)
		{
			key_ = NONE;
			b_first = false;
		}

		//determine transformation by key input
		switch (key_) {
		case ENTER:
			//*cloud_show_static += *cloud_moving;
			//HM_free = Eigen::Matrix4d::Identity();
			//for (int i = 0; i < index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			//HM_displacement_vec[index_PC_now] = HM_free.inverse() * HM_Trans_now;
			index_PC_now++;
			if (index_PC_now == filenames_.size()) b_break = true;
			b_makeNewPC = true;
			cout << "ENTER pressed" << endl;
			break;

		case SUBTRACT:
			HM_Trans_now = Eigen::Matrix4d::Identity();
			cout << "-(numpad) pressed" << endl;
			break;

		case TRANSLATION:


			enum Variable{
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

			do
			{
				string s_input;
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

			} while (var != Other_vr);

			if (var == ESC_vr) break;

			//HM_Trans_now

			
			break;

		default:
			break;
		}

		if (!(key_ == NONE || key_ == ENTER)) {
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

