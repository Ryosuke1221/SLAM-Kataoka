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

//calcXYZRPYFromPositoinMatrix

void CPointcloudFuction::all_process()
{
	//moveFile();

	int WhichProcess = 0;
	string filename1, filename2;
	bool b_finish = false;
	enum OPTION {
		EN_escape = 0,
		//EN_FreeSpace,
		EN_sequentshow,
		//EN_handregistration,
		EN_GetPcdFromCSV
	};

	while (!b_finish)
	{
		cout << endl;
		cout << "please input process number" << endl;
		cout << EN_escape << ": escape" << endl;
		//cout << EN_FreeSpace << ": free space" << endl;
		cout << EN_sequentshow << ": sequent show" << endl;
		//cout << EN_handregistration << ": hand registration" << endl;
		cout << EN_GetPcdFromCSV << ": get .pcd from .csv" << endl;

		cin >> WhichProcess;
		switch (WhichProcess)
		{
		case EN_escape:
			//escape
			b_finish = true;
			break;

		case EN_sequentshow:
			show_sequent();
			break;

		//case EN_handregistration:
		//	initVisualizer();
		//	//HandRegistration("../savedfolder/naraha summer/sequent");
		//	//HandRegistration("../savedfolder/naraha summer/sequent");
		//	HandRegistration("../savedfolder/temp");
		//	break;

		case EN_GetPcdFromCSV:
			getPCDFromCSV_naraha();
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
	foldername_ = "../../data/temp";

	typedef typename pcl::PointXYZI PointType_func;


	//CPointVisualization<pcl::PointXYZI> pv;
	CPointVisualization<PointType_func> pv;
	//CPointVisualization<pcl::PointXYZRGB> pv_XYZRGB;
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

			index_++;

		}

		//save
		if ((GetAsyncKeyState(VK_RETURN) & 1) == 1)
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

