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
		EN_DrawTrajectory,
		EN_DoMappingFromTrajectory,
		EN_DynamicTranslation
	};

	string dir_ = "../../data";

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
		cout << " " << EN_DynamicTranslation << ": DynamicTranslation" << endl;

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
			FileProcess(dir_);
			break;

		case EN_SequentShow:
			show_sequent_PointTypes(dir_);
			break;

		case EN_DrawTrajectory:
			DrawTrajectory(dir_ + "/process07_DrawTrajectory");
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory(dir_ + "/process12_DoMappingFromTrajectory");
			break;

		case EN_DynamicTranslation:
			DynamicTranslation(dir_ + "/process06_DynamicTranslation");
			break;

		default:
			break;
		}
	}
}

void CPointcloudFunction::FreeSpace()
{
	int i_method;
	//i_method = 0;
	i_method = 1;
	i_method = 2;

	if (i_method == 0)	//calc nearest neighbor of FPFH
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

	else if (i_method == 1)	//calc covariance of matrix of pointcloud
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

	else if (i_method == 2)//simple ICP
	{

		//#include <pcl/registration/icp.h>

		//getting ready of point cloud
		string dir_ = "../../data";
		string s_file_0 = "000XYZRGB_naraha.pcd";
		string s_file_1 = "001XYZRGB_naraha.pcd";
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::io::loadPCDFile(dir_ + "/" + s_file_0, *cloud_tgt);
		cloud_tgt->is_dense = true;
		pcl::io::loadPCDFile(dir_ + "/" + s_file_1, *cloud_src);
		cloud_src->is_dense = true;

		//setting point cloud to initial position
		//move cloud_tgt to "T(i-1)"
		//move cloud_src to "T(i-1) + odometory(i)"

		//calclating alignment
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> align_ICP;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_notUse(new pcl::PointCloud<pcl::PointXYZRGB>());
		align_ICP.setInputSource(cloud_src);
		align_ICP.setInputTarget(cloud_tgt);
		align_ICP.setMaximumIterations(10000);	//times, you should adjust it
		align_ICP.setMaxCorrespondenceDistance(0.1);	//[m], you should adjust it
		align_ICP.setEuclideanFitnessEpsilon(1e-5);
		align_ICP.setTransformationEpsilon(1e-6);
		align_ICP.align(*cloud_notUse);

		//getting result
		Eigen::Matrix4d trans_result = Eigen::Matrix4d::Identity();
		trans_result = align_ICP.getFinalTransformation().cast<double>();
		double d_X = trans_result(0, 3);						//[m]
		double d_Y = trans_result(1, 3);						//[m]
		double d_Z = trans_result(2, 3);						//[m]
		double d_Pitch = asin(-trans_result(2, 0));				//[rad]
		double d_Roll = asin(trans_result(2, 1) / cos(d_Pitch));//[rad]
		double d_Yaw = asin(trans_result(0, 1) / cos(d_Pitch));	//[rad]

		//Transformation to src
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_after(new pcl::PointCloud<pcl::PointXYZRGB>());
		{
			Eigen::Affine3f trans_result_affine = Eigen::Affine3f::Identity();
			trans_result_affine.translation() << d_X, d_Y, d_Z;
			trans_result_affine.rotate(Eigen::AngleAxisf(d_Yaw, Eigen::Vector3f::UnitZ()));
			trans_result_affine.rotate(Eigen::AngleAxisf(d_Roll, Eigen::Vector3f::UnitY()));
			trans_result_affine.rotate(Eigen::AngleAxisf(d_Pitch, Eigen::Vector3f::UnitX()));
			pcl::transformPointCloud(*cloud_src, *cloud_src_after, trans_result_affine);
			//posotion of cloud_src: "0" -> "T(i-1) + odometory(i)" -> "T(i-1) + odometory(i) + T_ICP" = "T(i)" 
		}
	}

}

void CPointcloudFunction::DynamicTranslation(string dir_)
{
	//bool b_saveByTXT = false;
	bool b_inputTranslation = false;

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
