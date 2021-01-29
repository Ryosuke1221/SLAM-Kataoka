#include "PointcloudBasicProcess.h"

void CPointcloudBasicProcess::all_process()
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
		EN_DrawTrajectory,
		EN_DoMappingFromTrajectory,
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
			FreeSpace();
			break;

		case EN_FileProcess:
			FileProcess("../../data");
			break;

		case EN_SequentShow:
			show_sequent_PointTypes("../../data");
			break;

		case EN_DrawTrajectory:
			DrawTrajectory("../../data");
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory("../../data/process12_DoMappingFromTrajectory");
			break;

		default:
			break;
		}

	}

}

void CPointcloudBasicProcess::FreeSpace()
{
	int i_method;
	//i_method = 0;
	i_method = 1;
	i_method = 2;
	i_method = 3;
	//i_method = 4;
	i_method = 5;

	//if (i_method == 0)	//calc nearest neighbor of FPFH
	//{
	//	typedef typename pcl::FPFHSignature33 FeatureT;
	//	typedef typename pcl::PointXYZRGB T_PointType;

	//	int num_nearest = 10;
	//	//num_nearest = 1;

	//	string dir_ = "../../data";

	//	string s_file_0 = "000XYZRGB_naraha.pcd";
	//	string s_file_1 = "001XYZRGB_naraha.pcd";

	//	float voxel_size;
	//	voxel_size = 0.1;

	//	float radius_normal_FPFH, radius_FPFH;
	//	radius_normal_FPFH = 0.5;
	//	radius_FPFH = 1.;


	//	pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
	//	pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	//	pcl::io::loadPCDFile(dir_ + "/" + s_file_0, *cloud_tgt);
	//	pcl::io::loadPCDFile(dir_ + "/" + s_file_1, *cloud_src);
	//	cloud_tgt->is_dense = true;
	//	cloud_src->is_dense = true;

	//	//compute fpfh
	//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
	//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
	//	{
	//		pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
	//		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
	//		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
	//		sor->setInputCloud(cloud_tgt);
	//		sor->filter(*cloud_VGF);
	//		fpfh_tgt = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_tgt, radius_normal_FPFH, radius_FPFH);
	//	}
	//	{
	//		pcl::PointCloud<T_PointType>::Ptr cloud_VGF(new pcl::PointCloud<T_PointType>());
	//		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
	//		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
	//		sor->setInputCloud(cloud_src);
	//		sor->filter(*cloud_VGF);
	//		fpfh_src = CFPFH_PCL::computeFPFH<T_PointType>(cloud_VGF, cloud_src, radius_normal_FPFH, radius_FPFH);
	//	}

	//	pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr feature_tree_(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
	//	feature_tree_->setInputCloud(fpfh_tgt);

	//	//index_near_vecvec = CFPFH_PCL::getNearestOfFPFH(fpfh_src, num_nearest, feature_tree_, squaredDistance_near_vecvec);
	//	pcl::Correspondences corrs_;
	//	corrs_ = CFPFH_PCL::getNearestOfFPFH(fpfh_src, num_nearest, feature_tree_);

	//	for (int j = 0; j < corrs_.size(); j++)
	//	{
	//		if (j % 100 != 0) continue;
	//		cout << "j:" << j << "  query:" << corrs_[j].index_query << " match:" << corrs_[j].index_match << " distance:" << corrs_[j].distance << endl;
	//	}

	//}

	/*else*/ if (i_method == 1)	//calc covariance of matrix of pointcloud
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
		mat_ = calcEigenMatrixFromPointCloud(cloud_tgt);
		Eigen::Matrix<float, 3, 3> mat_cov = calcCovarianceMatrix(mat_);
		cout << "show mat_cov" << endl;
		cout << mat_cov << endl;

	}

	//else if (i_method == 4)//simple ICP
	//{

	//	//#include <pcl/registration/icp.h>

	//	//getting ready of point cloud
	//	string dir_ = "../../data";
	//	string s_file_0 = "000XYZRGB_naraha.pcd";
	//	string s_file_1 = "001XYZRGB_naraha.pcd";
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	pcl::io::loadPCDFile(dir_ + "/" + s_file_0, *cloud_tgt);
	//	cloud_tgt->is_dense = true;
	//	pcl::io::loadPCDFile(dir_ + "/" + s_file_1, *cloud_src);
	//	cloud_src->is_dense = true;

	//	//setting point cloud to initial position
	//	//move cloud_tgt to "T(i-1)"
	//	//move cloud_src to "T(i-1) + odometory(i)"

	//	//calclating alignment
	//	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> align_ICP;
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_notUse(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	align_ICP.setInputSource(cloud_src);
	//	align_ICP.setInputTarget(cloud_tgt);
	//	align_ICP.setMaximumIterations(10000);	//times, you should adjust it
	//	align_ICP.setMaxCorrespondenceDistance(0.1);	//[m], you should adjust it
	//	align_ICP.setEuclideanFitnessEpsilon(1e-5);
	//	align_ICP.setTransformationEpsilon(1e-6);
	//	align_ICP.align(*cloud_notUse);

	//	//getting result
	//	Eigen::Matrix4d trans_result = Eigen::Matrix4d::Identity();
	//	trans_result = align_ICP.getFinalTransformation().cast<double>();
	//	double d_X = trans_result(0, 3);						//[m]
	//	double d_Y = trans_result(1, 3);						//[m]
	//	double d_Z = trans_result(2, 3);						//[m]
	//	double d_Pitch = asin(-trans_result(2, 0));				//[rad]
	//	double d_Roll = asin(trans_result(2, 1) / cos(d_Pitch));//[rad]
	//	double d_Yaw = asin(trans_result(0, 1) / cos(d_Pitch));	//[rad]

	//	//Transformation to src
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_after(new pcl::PointCloud<pcl::PointXYZRGB>());
	//	{
	//		Eigen::Affine3f trans_result_affine = Eigen::Affine3f::Identity();
	//		trans_result_affine.translation() << d_X, d_Y, d_Z;
	//		trans_result_affine.rotate(Eigen::AngleAxisf(d_Yaw, Eigen::Vector3f::UnitZ()));
	//		trans_result_affine.rotate(Eigen::AngleAxisf(d_Roll, Eigen::Vector3f::UnitY()));
	//		trans_result_affine.rotate(Eigen::AngleAxisf(d_Pitch, Eigen::Vector3f::UnitX()));
	//		pcl::transformPointCloud(*cloud_src, *cloud_src_after, trans_result_affine);
	//		//posotion of cloud_src: "0" -> "T(i-1) + odometory(i)" -> "T(i-1) + odometory(i) + T_ICP" = "T(i)" 
	//	}


	//}
}

void CPointcloudBasicProcess::changeColor_plane(pcl::PointXYZRGB &point_)
{
	point_.r = 0;
	point_.g = 0;
	point_.b = 255;
}

void CPointcloudBasicProcess::changeColor_plane(pcl::PointXYZI &point_)
{
	point_.intensity = 210;
}

void CPointcloudBasicProcess::FileProcess(string dir_)
{
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

void CPointcloudBasicProcess::FileProcess_copy(string dir_from, string dir_to)
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

void CPointcloudBasicProcess::FileProcess_delete(string dir)
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

void CPointcloudBasicProcess::FileProcess_evacuate(string dir)
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

void CPointcloudBasicProcess::FileProcess_FolderInFolder(string dir_, vector<string> &folder_vec)
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

void CPointcloudBasicProcess::DrawTrajectory(string dir_)
{
	//typedef typename pcl::PointXYZI T_PointType;
	typedef typename pcl::PointXYZRGB T_PointType;

	//read file name
	//string dir = "../../data/process07_DrawTrajectory";
	vector<string> filenames_txt;
	CTimeString::getFileNames_extension(dir_, filenames_txt, ".csv");

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
			if (!(0 <= i_value && i_value < filenames_txt.size()))
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
		trajectory_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames_txt[i_readfile_vec[0]]);
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
		trajectory_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filenames_txt[i_readfile_vec[1]]);
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

	else if (i_readfile_vec.size() == 1 && b_show_sequently) {}

	int index_ = 0;
	while (1)
	{
		if (b_show_sequently && GetAsyncKeyState(VK_SPACE) & 1)
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

void CPointcloudBasicProcess::DoMappingFromTrajectory(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	bool b_showNumber = false;
	b_showNumber = true;

	bool b_showArrow = false;
	b_showArrow = true;

	bool b_showTrajectory = false;
	b_showTrajectory = true;

	bool b_useGrid = false;
	b_useGrid = true;

	bool b_useTurnYaw = false;
	//b_useTurnYaw = true;

	bool b_useWhiteCloud = false;
	//b_useWhiteCloud = true;

	if (!b_showTrajectory)
	{
		b_showNumber = false;
		b_showArrow = false;
	}

	float yaw_trans = 0.;
	yaw_trans = -7.* D2R;

	string s_folder;
	{
		vector<string> s_folder_vec;
		CTimeString::getFileNames_folder(dir_, s_folder_vec);

		if (s_folder_vec.size() == 0)
		{
			cout << "ERROR: No PointCloud folder found." << endl;
			return;
		}

		for (int i = 0; i < s_folder_vec.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << s_folder_vec[i] << endl;
		}
		cout << "select folder of input point cloud" << endl;
		cout << "->";
		int i_folder;
		cin >> i_folder;
		s_folder = s_folder_vec[i_folder];
	}

	//input trajectory
	//string trajectory_csv = "trajectory.csv";
	string s_trajectory_csv;
	{
		vector<string> filenames_;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_, ".csv");
		if (filenames_.size() == 1)
			s_trajectory_csv = filenames_[0];
		else if (filenames_.size() == 0)
		{
			cout << "ERROR: No trajectory file found." << endl;
			return;
		}
		else
		{
			cout << "ERROR: Too many trajectory files found." << endl;
			return;
		}
	}
	vector<Eigen::Vector6d> trajectoryVector_vec;
	vector<int> frames_all;
	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder + "/" + s_trajectory_csv);

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
			if (b_useTurnYaw)
			{
				Eigen::Matrix4d yaw_Mat = Eigen::Matrix4d::Identity();
				yaw_Mat = calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., yaw_trans);
				yaw_Mat = yaw_Mat * calcHomogeneousMatrixFromVector6d(trajectoryVector_);
				trajectoryVector_ = calcVector6dFromHomogeneousMatrix(yaw_Mat);
				if (stoi(s_temp_vecvec[j][7]) == 1) frames_all.push_back(-1);
				else frames_all.push_back(j - 1);
			}
			trajectoryVector_vec.push_back(trajectoryVector_);
		}
	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	{

		vector<string> cloud_filenames;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, cloud_filenames, ".pcd");
		for (int i = 0; i < cloud_filenames.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + cloud_filenames[i], *cloud);
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
	if (b_useGrid) pv.drawGrid(30., 30., -30., -30.);

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
				if (b_useWhiteCloud)
				{
					for (int i = 0; i < cloud->size(); i++)
					{
						cloud->points[i].r = 255;
						cloud->points[i].g = 255;
						cloud->points[i].b = 255;
					}
				}
				//transformation
				{
					auto trans_ = calcAffine3fFromHomogeneousMatrix(
						calcHomogeneousMatrixFromVector6d(trajectoryVector_vec[index_cloud])
					);
					pcl::transformPointCloud(*cloud, *cloud, trans_);
				}
				*cloud_map += *cloud;
				pv.setPointCloud(cloud_map);
				if(b_showTrajectory)
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

void CPointcloudBasicProcess::show_sequent_PointTypes(string dir_)
{
	//string dir_;
	//dir_ = "../../data";
	vector<string> dir_folder_vec;
	FileProcess_FolderInFolder(dir_, dir_folder_vec);

	int i_select;
	cout << "select folder (index) ->";
	cin >> i_select;
	cout << i_select << "(" << dir_folder_vec[i_select] << ")" << endl;
	dir_ = dir_ + "/" + dir_folder_vec[i_select];
	cout << endl;

	bool b_onlyConvergence = false;
	//b_onlyConvergence = true;
	//{
	//	int i_find = dir_.find("GR_FPFH_SAC_IA/2020");
	//	if (i_find == std::string::npos) b_onlyConvergence = false;
	//	else b_onlyConvergence = true;
	//}

	vector<string> filenames_;
	if (!b_onlyConvergence) CTimeString::getFileNames_extension(dir_, filenames_, ".pcd");
	else CTimeString::getFileNames_extension(dir_, filenames_, "C1_XYZRGB.pcd");
	cout << "file size: " << filenames_.size() << endl;

	//type check
	{
		int num_intensity = 0;
		int num_rgb = 0;
		for (int j = 0; j < filenames_.size(); j++)
		{
			int i_type = getPointCloudType(dir_ + "/" + filenames_[j]);
			if (i_type == 0) num_intensity++;
			else if (i_type == 1) num_rgb++;
		}

		if (num_intensity > 0 && num_rgb == 0)
		{
			show_sequent_template<pcl::PointXYZI>(dir_, filenames_);
		}
		else if (num_rgb > 0 && num_intensity == 0)
		{
			show_sequent_template<pcl::PointXYZRGB>(dir_, filenames_);
		}
		else
		{
			cout << "ERROR(CPointcloudBasicProcess::show_sequent): Different types contained." << endl;
			throw std::runtime_error("ERROR(CPointcloudBasicProcess::show_sequent): Different types contained.");
		}
	}

}
