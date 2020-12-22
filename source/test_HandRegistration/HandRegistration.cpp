#include "HandRegistration.h"

void CHandRegistration::mainProcess()
{
	string dir_;
	//dir_ = "../../data/temp/_Hand";
	dir_ = "../../data/data_test_HandRegistration";

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
		EN_SomePointclouds,
		EN_showFeatureValue,
		EN_HandRegistration
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
		cout << " " << EN_SomePointclouds << ": SomePointclouds" << endl;
		cout << " " << EN_showFeatureValue << ": showFeatureValue" << endl;
		cout << " " << EN_HandRegistration << ": HandRegistration" << endl;

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
			CHandRegistration::FreeSpace();
			break;

		case EN_FileProcess:
			FileProcess();
			break;

		case EN_SequentShow:
			show_sequent();
			break;

		case EN_DrawTrajectory:
			DrawTrajectory();
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory();
			break;

		case EN_HandRegistration:
			HandRegistration(dir_);
			break;

		default:
			break;
		}

	}

}

void CHandRegistration::FreeSpace()
{

}

void CHandRegistration::HandRegistration(string dir_)
{
	//"delta_T(i) = T(i-1).inverse() * T(i);",
	//because relative displacement can accumulate correctly.
	//When delta_T(i-1) changed, delta_T(i) should be able to work.

	//cout << "HandRegistration started!" << endl;
	//Sleep(1 * 1000);


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

	int MaximumIterations;
	double MaxCorrespondenceDistance, EuclideanFitnessEpsilon, TransformationEpsilon;
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
			filename_txt = dir_ + "/transformation.csv";
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

	while (1)
	{
		//input next PointCloud
		if (b_makeNewPC)
		{

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
			cout << "calc median: Right SHIFT" << endl;
			cout << "Registration: Right CTRL" << endl;
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
				cout << CExtendableICP::getMedianDistance(cloud_before, cloud_moving) << endl;
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
				pcl::IterativeClosestPoint<PointType_func, PointType_func> align_ICP;
				pcl::PointCloud<PointType_func>::Ptr cloud_temp_align(new pcl::PointCloud<PointType_func>());
				align_ICP.setMaximumIterations(MaximumIterations);
				align_ICP.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
				align_ICP.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
				align_ICP.setTransformationEpsilon(TransformationEpsilon);
				align_ICP.setInputSource(cloud_moving);
				align_ICP.setInputTarget(cloud_before);
				align_ICP.align(*cloud_temp_align);
				Registration_Vec = calcVector6dFromHomogeneousMatrix(align_ICP.getFinalTransformation().cast<double>());
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

		if (!(key_ == NONE || key_ == ENTER))
		{
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

		if (cloud_show->size() != 0 && !b_break)
		{
			//pv.setPointCloud(cloud_show);
			pv.setPointCloud(cloud_show, filenames_[index_PC_now]);
			pv.updateViewer();
			pv_2frame.setPointCloud(cloud_show_2frame, filenames_[index_PC_now]);
			pv_2frame.updateViewer();
		}

		if (b_break) break;
	}

	bool b_save_txt;
	bool b_loop = true;
	int i_save = 2;
	cout << "Do you save txt?  Yes:1 No:0" << endl;
	cout << "->";
	while (b_loop)
	{
		//b_loop = false;
		cin >> i_save;
		if (i_save == 1)
		{
			b_save_txt = true;
			b_loop = false;
		}
		else if (i_save == 0)
		{
			b_save_txt = false;
			b_loop = false;
		}
		else b_loop = true;
	}

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

CHandRegistration::KEYNUM CHandRegistration::getKEYNUM()
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

