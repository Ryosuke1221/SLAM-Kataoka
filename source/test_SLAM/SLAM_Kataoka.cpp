#include"SLAM_Kataoka.h"

CSLAM_Kataoka::CSLAM_Kataoka(void)//èâä˙âª--------------------------------------------
	//:M_pFilePointCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
	//M_pFilePCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
	////M_before_pASUSXtionPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>),
	//M_pPointCloud_before(new pcl::PointCloud<pcl::PointXYZRGB>)
	////M_pFinal(new pcl::PointCloud<pcl::PointXYZRGB>)
	:
M_align_ICP(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>)
, M_align_RANSAC(new pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>)
, M_Tree_FPFH(new pcl::search::KdTree<pcl::PointXYZRGB>)

{
	M_pFilePointCloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pFilePCloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_before = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

	//RANSAC?
	M_pPointCloud_alignment_Source = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_alignment_Target = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

	M_pPC_FPFH_Source = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPC_FPFH_Target = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPC_Normals_FPFH_Source = (new pcl::PointCloud<pcl::Normal>)->makeShared();
	M_pPC_Normals_FPFH_Target = (new pcl::PointCloud<pcl::Normal>)->makeShared();
	M_Result_FPFH_Source = (new pcl::PointCloud<pcl::FPFHSignature33>)->makeShared();
	M_Result_FPFH_Target = (new pcl::PointCloud<pcl::FPFHSignature33>)->makeShared();
	M_pASUSXtionPointCloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

	M_pPointCloud_show0_pre = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_show0 = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_show1_pre = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_show1 = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_show2_pre = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_show2_XYZI_pre = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();
	M_pPointCloud_show2_XYZI = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();

	M_pPointCloud_show3 = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_show3_XYZI = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();

	M_pPC_Normals_Edge = (new pcl::PointCloud<pcl::Normal>)->makeShared();

	//experiment
	M_pPointCloud_Artificial_Experiment = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_Temp = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_b_InputOdometryRead = false;
	M_b_first = true;
	M_pPointCloud_init_src = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_init_tgt = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_kataokaPCL = new CKataokaPCL;

	M_pPointCloud_chara0_src = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_chara1_src = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_chara2_src = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_chara0_tgt = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_chara1_tgt = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_chara2_tgt = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

	M_th_height_map = 2.;

	M_pPointCloud_octomap = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

	//hand closure
	M_pPointCloud_00_filtered = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_13_filtered = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

	M_pPointCloud_Evaluation = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_Evaluation_TRUE = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
}
//-----------------------------------------------------------------------------------------

CSLAM_Kataoka::~CSLAM_Kataoka(void)
{
}


void CSLAM_Kataoka::doProcess_OctoMap()
{
	CTimeString time_;
	string t_start = time_.getTimeString();
	static string t_FirstFrame;
	if (true == M_b_first) t_FirstFrame = time_.getTimeString();


	M_b_ShowHorizontal = true;

	bool b_LoopCandidate = false;

	int i_skip = 1;
	//i_skip = 5;
	static int i_frame_start = 0;
	static int i_frame_end = -1;

	//naraha
	i_frame_end = 13;
	////todai
	//////start at center of corridor
	////i_frame_start = 415;
	//////end at edge of first corridor
	////i_frame_end = 570;
	//////start at second corridor toward right
	////i_frame_start = 875;
	//////end at second corridor edge
	////i_frame_end = 985;
	//////start at
	////i_frame_start = 1110;
	//////end at
	////i_frame_end = 1225;
	////start at left edge
	//i_frame_start = 640;
	////end at right edge
	//i_frame_end = 985;
	////start
	//i_frame_start = 0;
	////end
	//i_frame_end = 1350;

	////start after first curve
	//i_frame_start = 250;
	////start near curve
	//i_frame_start = 165;


	//n frame skip
	if (i_frame_end%i_skip != 0) {

		i_frame_end = i_frame_end - i_frame_end % i_skip;
	}

	//Naraha
	if (M_b_first == false)	M_i_cnt++;


	//SensorDataProcessing ----------------------------------------------------------------------------------------------
	do_SensorDataProcessing(i_frame_start, i_frame_end, i_skip);


	//Localization ----------------------------------------------------------------------------------------------
	do_Localization();


	// Mapping ----------------------------------------------------------------------------------------------
	do_Mapping();


	//Visualization ----------------------------------------------------------------------------------------------
	do_Visualization();


	//CharaClosing_Ryuki
	//do_CharaClosing(i_frame_end);
	//if(M_i_cnt == i_frame_end) do_CharaClosing1121(i_frame_end);

	//if (b_ReadOnly && (M_i_cnt == 0)) {
	//	do_CharaClosing1121(i_frame_end, M_InputTrajectory_vec, true);
	//	{
	//		int aa;
	//		cin >> aa;
	//	}
	//}
	//else if (!b_ReadOnly && (M_i_cnt == 13)) {
	//	do_CharaClosing1121(i_frame_end, M_DREstimatedPath);
	//	{
	//		int aa;
	//		cin >> aa;
	//	}
	//}


	//if (M_i_cnt == 0) {
	//	do_CharaClosing_Check();
	//	int aa;
	//	cin >> aa;
	//}

	//Loop Closure ----------------------------------------------------------------------------------------------
	//b_LoopCandidate = isLoopCandidate();
	//if (M_i_cnt == 13) b_LoopCandidate = true;

	//if (b_LoopCandidate) do_LoopClosing(0);


	//evaluate map ----------------------------------------------------------------------------------------------
	do_exp_CalculateEvaluation_202004(i_frame_start, i_frame_end, i_skip);


	string t_end = time_.getTimeString();
	cout << "time elapse(Loop " << M_i_cnt << ") = " << time_.getTElapsefrom2S(t_start, t_end) << endl;

	//t_FirstFrame
	string t_NowFrame = time_.getTimeString();
	cout << "sum of elapsed time = " << time_.getTElapsefrom2S(t_FirstFrame, t_NowFrame) << endl;
	cout << "(" << t_NowFrame << ")" << endl << endl << endl;


	//if (CSensorDataManager::getInstance()->getLoopCnt() >= 1354) this->terminate();

	//if (M_i_cnt == 431) Sleep(30 * 1000);

	//M_i_cnt++;

	M_b_first = false;

	if (M_i_cnt >= i_frame_end)
	{
		this->terminate();
	}


}

void CSLAM_Kataoka::do_SensorDataProcessing(int frame_first_arg, int frame_final_arg, int frame_skip_arg)
{

	//while (M_i_cnt < i_frame_start)
	//{
	//	cout << "Loop : " << M_i_cnt << endl;
	//	CSensorDataManager::getInstance()->setDuration(this->getDuration());
	//	if (!CSensorDataManager::getInstance()->readSensorData())	this->terminate();
	//	M_i_cnt++;
	//}

	if (M_b_first)
	{

		//skiping to start frame
		//calling getDuration until start frame
		//while (1)
		//{
		//	cout << "Loop : " << M_i_cnt << endl;
		//	CSensorDataManager::getInstance()->setDuration(this->getDuration());
		//	if (!CSensorDataManager::getInstance()->readSensorData())	this->terminate();
		//	if (M_i_cnt >= i_frame_start) break;
		//	M_i_cnt++;
		//}
		//cout << "start at " << M_i_cnt << endl;
	}

	////get sensor data ==================================================================================================
	////cout << "Loop(manager) : " << CSensorDataManager::getInstance()->getLoopCnt() << endl;
	//CSensorDataManager::getInstance()->setDuration(this->getDuration());
	//if (!CSensorDataManager::getInstance()->readSensorData())	this->terminate();

	cout << "Loop : " << M_i_cnt << endl;
	//M_EncoderData = M_EncoderData_Noise_vec[M_i_cnt];

	M_pASUSXtionPointCloud->clear();
	pcl::copyPointCloud(*M_pPointCloud_naraha_vec[M_i_cnt], *M_pASUSXtionPointCloud);
	//M_pASUSXtionPointCloud = M_pPointCloud_naraha_vec[M_i_cnt];

	//displacement by odometry
	//M_DRState = calc_Odometry_3variable(M_DRState, M_EncoderData.dDeltaX, M_EncoderData.dDeltaY, M_EncoderData.dDeltaYaw);	//x,y,yaw

	//SlSr
	{
		M_DRState_Displacement_odometry.setPose(0., 0., 0., 0., 0., 0.);
		//M_pDRLocalizer->setState(M_DRState_Displacement_odometry);
		//M_pDRLocalizer->setControlInput((M_EncoderDataSlSr_vec[M_i_cnt].Sr / 750) * 377 * 0.001, (M_EncoderDataSlSr_vec[M_i_cnt].Sl / 750) * 377 * 0.001);
		//M_pDRLocalizer->estimateState();
		//M_DRState_Displacement_odometry = M_pDRLocalizer->getState();
		cout << "M_DRState_Displacement_odometry:" << endl;
		cout << "X = " << M_DRState_Displacement_odometry.getX() << endl;
		cout << "Y = " << M_DRState_Displacement_odometry.getY() << endl;
		cout << "Yaw = " << M_DRState_Displacement_odometry.getYaw() * R2D << endl;
	}

	//todai
	//while (1)
	//{
	//	//cout << "Loop(manager) : " << CSensorDataManager::getInstance()->getLoopCnt() << endl;
	//	CSensorDataManager::getInstance()->setDuration(this->getDuration());
	//	if (!CSensorDataManager::getInstance()->readSensorData())	this->terminate();
	//	//get sensor data ==================================================================================================
	//	M_EncoderData = CSensorDataManager::getInstance()->getPioneer3DXEncoderData();
	//	M_pASUSXtionDepthImage = CSensorDataManager::getInstance()->getASUSXtionDepthGrayImage();
	//	M_pASUSXtionColorImage = CSensorDataManager::getInstance()->getASUSXtionRGBImage();
	//	M_pASUSXtionPointCloud = CSensorDataManager::getInstance()->getASUSXtionPointCloud();
	//	M_pASUSXtionRangeData = CSensorDataManager::getInstance()->getASUSXtionRangeData();
	//	M_pASUSXtionDepthSavingImage = CSensorDataManager::getInstance()->getASUSXtionDepthSavingImage();			//kataoka20190716
	//	//==================================================================================================
	//	//read NoiseOdometry
	//	M_EncoderData = M_EncoderData_Noise_vec[M_i_cnt];
	//	if(M_b_first == false)	M_i_cnt++;
	//	cout << "Loop : " << M_i_cnt << endl;
	//	//displacement by odometry
	//	M_DRState = calc_Odometry_3variable(M_DRState, M_EncoderData.dDeltaX, M_EncoderData.dDeltaY, M_EncoderData.dDeltaYaw);
	//	if (M_i_cnt % i_skip == 0) 	break;
	//}

	//cout << "Loop : " << M_i_cnt << endl;
	//CSensorDataManager::getInstance()->setDuration(this->getDuration());
	////if (!CSensorDataManager::getInstance()->readSensorData())	this->terminate();
	//if (!CSensorDataManager::getInstance()->readSensorData()) {
	//	this->terminate();
	//	cout << "M_num_warp_RANSAC = " << M_num_warp_RANSAC << endl;
	//	cout << "M_num_warp_ICP = " << M_num_warp_ICP << endl;
	//}

	do_exp_DecreasePointCloud(M_i_cnt, M_pASUSXtionPointCloud);

	//Use Character by Naraha Data
	//do_exp_CreateAndRead_CharaData();
	do_exp_CreateAndRead_CharaData_1130(frame_final_arg, true);
}

void CSLAM_Kataoka::do_Localization()
{
	cout << "Localization started" << endl;

	if (M_b_first)
	{
		M_DRState = M_TRUETrajectory_vec[M_i_cnt];
	}
	else
	{
		if (M_b_ReadOnly)
			M_DRState = M_InputTrajectory_vec[M_i_cnt];

		else
		{
			cout << "Registration" << endl;
			int i_method = 0;
			//i_method = 0;//0:ICP
			i_method = 1;//1:Chara_ICP
			//i_method = 2;//2:Spring_ICP_1
			//i_method = 3;//3:Spring_ICP_2
			do_Registration(i_method);

			////debug
			//cout << "M_pPointCloud_before size = " << M_pPointCloud_before->size() << endl;
			//cout << "M_pASUSXtionPointCloud size = " << M_pASUSXtionPointCloud->size() << endl;
			//for (size_t i = 0; i < 5; i++)
			//{
			//	cout << "M_pASUSXtionPointCloud->points[" << i << "]:";
			//	cout << " x:" << M_pASUSXtionPointCloud->points[i].x;
			//	cout << " y:" << M_pASUSXtionPointCloud->points[i].y;
			//	cout << " z:" << M_pASUSXtionPointCloud->points[i].z;
			//	cout << endl;
			//}
			//M_pPointCloud_Temp->clear();
			//pcl::copyPointCloud(*M_pPointCloud_before, *M_pPointCloud_Temp);
			//*M_pPointCloud_Temp += *M_pASUSXtionPointCloud;
			//for (size_t i = M_pPointCloud_before->size(); i < M_pPointCloud_before->size() + 5; i++)
			//{
			//	cout << "M_pPointCloud_Temp->points[" << i << "]:";
			//	cout << " x:" << M_pPointCloud_Temp->points[i].x;
			//	cout << " y:" << M_pPointCloud_Temp->points[i].y;
			//	cout << " z:" << M_pPointCloud_Temp->points[i].z;
			//	cout << endl;
			//}

			//displacement by odometry
			Eigen::Vector6d Pos_Vec = Eigen::Vector6d::Zero();
			Pos_Vec = calcXYZRPYFromPositoinMatrix(calcPositoinMatrixFromXYZRPY(M_DRState.getX(), M_DRState.getY(), M_DRState.getZ(),
				M_DRState.getRoll(), M_DRState.getPitch(), M_DRState.getYaw())
				* calcPositoinMatrixFromXYZRPY(M_DRState_Displacement_odometry.getX(), M_DRState_Displacement_odometry.getY(), M_DRState_Displacement_odometry.getZ(),
					M_DRState_Displacement_odometry.getRoll(), M_DRState_Displacement_odometry.getPitch(), M_DRState_Displacement_odometry.getYaw()));
			M_DRState.setPose(Pos_Vec(0, 0), Pos_Vec(1, 0), Pos_Vec(2, 0),
				Pos_Vec(3, 0), Pos_Vec(4, 0), Pos_Vec(5, 0));

			//calculate SensorState after displacement by odometry
			CRobotState SensorState_afterOdo_w;
			SensorState_afterOdo_w.setX(M_DRState.getX() + cos(M_DRState.getYaw()) * M_OnboardXtionState.getX());
			SensorState_afterOdo_w.setY(M_DRState.getY() + sin(M_DRState.getYaw()) * M_OnboardXtionState.getX());
			SensorState_afterOdo_w.setYaw(M_DRState.getYaw());

			//calculate SensorState after registration by RANSAC
			CRobotState SensorState_afterRANSAC_w;
			SensorState_afterRANSAC_w.setX(cos(M_yaw_initial)*SensorState_afterOdo_w.getX()
				- sin(M_yaw_initial)*SensorState_afterOdo_w.getY());
			SensorState_afterRANSAC_w.setY(sin(M_yaw_initial)*SensorState_afterOdo_w.getX()
				+ cos(M_yaw_initial)*SensorState_afterOdo_w.getY());
			SensorState_afterRANSAC_w.setYaw(SensorState_afterOdo_w.getYaw() + M_yaw_initial);
			SensorState_afterRANSAC_w.setX(SensorState_afterRANSAC_w.getX() + M_x_initial);
			SensorState_afterRANSAC_w.setY(SensorState_afterRANSAC_w.getY() + M_x_initial);

			//calculate SensorState after registration by ICP
			CRobotState SensorState_afterICP_w;
			SensorState_afterICP_w.setX(cos(M_yaw_ICP)*SensorState_afterRANSAC_w.getX()
				- sin(M_yaw_ICP)*SensorState_afterRANSAC_w.getY());
			SensorState_afterICP_w.setY(sin(M_yaw_ICP)*SensorState_afterRANSAC_w.getX()
				+ cos(M_yaw_ICP)*SensorState_afterRANSAC_w.getY());
			SensorState_afterICP_w.setYaw(SensorState_afterRANSAC_w.getYaw() + M_yaw_ICP);
			SensorState_afterICP_w.setX(SensorState_afterICP_w.getX() + M_x_ICP);
			SensorState_afterICP_w.setY(SensorState_afterICP_w.getY() + M_y_ICP);

			//calculate RobotState after registration
			M_DRState.setYaw(SensorState_afterICP_w.getYaw());
			M_DRState.setX(SensorState_afterICP_w.getX() - cos(M_DRState.getYaw()) * M_OnboardXtionState.getX());
			M_DRState.setY(SensorState_afterICP_w.getY() - sin(M_DRState.getYaw()) * M_OnboardXtionState.getX());

			////show displacement
			//cout << "displacement_X = " << M_DRState.getX() - DRState_old.getX() << endl;
			//cout << "displacement_Y = " << M_DRState.getY() - DRState_old.getY() << endl;
			//cout << "displacement_Yaw = " << (M_DRState.getYaw() - DRState_old.getYaw())*R2D << "[deg]" << endl;
		}
	}


	M_pPointCloud_before->clear();
	pcl::copyPointCloud(*M_pASUSXtionPointCloud, *M_pPointCloud_before);
	//M_Chara_tgt_vec.clear();
	//M_Chara_tgt_vec = M_Chara_src_vec;


	{
		CRobotState DRState_map;
		if (M_b_ShowHorizontal) {

			DRState_map = M_DRState;
			double delta_yaw = 36.9*D2R;
			Eigen::Matrix4d	Pos_Mat = Eigen::Matrix4d::Identity();
			Pos_Mat = calcPositoinMatrixFromXYZRPY(0., 0., 0., 0., 0., delta_yaw)
				* calcPositoinMatrixFromXYZRPY(
					DRState_map.getX(), DRState_map.getY(), DRState_map.getZ(),
					DRState_map.getRoll(), DRState_map.getPitch(), DRState_map.getYaw());

			Eigen::Vector6d Pos_XYZRPY = Eigen::Vector6d::Zero();
			Pos_XYZRPY = calcXYZRPYFromPositoinMatrix(Pos_Mat);
			DRState_map.setPose(Pos_XYZRPY(0, 0), Pos_XYZRPY(1, 0), Pos_XYZRPY(2, 0),
				Pos_XYZRPY(3, 0), Pos_XYZRPY(4, 0), Pos_XYZRPY(5, 0));
			M_pDREstimatedPathList->push_back(DRState_map);
		}
		else M_pDREstimatedPathList->push_back(M_DRState);
	}

	M_DREstimatedPath.push_back(M_DRState);

	cout << "Robot State: i:" << M_i_cnt;
	cout << " x: " << M_DRState.getX();
	cout << " y: " << M_DRState.getY();
	cout << " yaw: " << M_DRState.getYaw() * R2D << "[deg]";
	cout << endl;

}

void CSLAM_Kataoka::do_Mapping()
{
	cout << "Mapping started" << endl;

	//CTimeString time_;
	//string t1 = time_.getTimeString();

	//////show3
	////M_pPointCloud_show3->clear();
	////pcl::copyPointCloud(*M_pPointCloud_octomap, *M_pPointCloud_show3);

	////octomap
	//{
	//	M_pPointCloud_octomap->clear();
	//	pcl::copyPointCloud(*M_pASUSXtionPointCloud, *M_pPointCloud_octomap);
	//	M_Trans = Eigen::Affine3f::Identity();
	//	M_Trans.translation() << M_OnboardXtionState.getX(), M_OnboardXtionState.getY(), M_OnboardXtionState.getZ();
	//	M_Trans.rotate(Eigen::AngleAxisf(M_OnboardXtionState.getYaw(), Eigen::Vector3f::UnitZ()));
	//	M_Trans.rotate(Eigen::AngleAxisf(M_OnboardXtionState.getPitch(), Eigen::Vector3f::UnitY()));
	//	M_Trans.rotate(Eigen::AngleAxisf(M_OnboardXtionState.getRoll(), Eigen::Vector3f::UnitX()));
	//	pcl::transformPointCloud(*M_pPointCloud_octomap, *M_pPointCloud_octomap, M_Trans);
	//	CRobotState State_map = M_pDREstimatedPathList->back();
	//	M_Trans = Eigen::Affine3f::Identity();
	//	M_Trans.translation() << State_map.getX(), State_map.getY(), State_map.getZ();
	//	M_Trans.rotate(Eigen::AngleAxisf(State_map.getYaw(), Eigen::Vector3f::UnitZ()));
	//	M_Trans.rotate(Eigen::AngleAxisf(State_map.getPitch(), Eigen::Vector3f::UnitY()));
	//	M_Trans.rotate(Eigen::AngleAxisf(State_map.getRoll(), Eigen::Vector3f::UnitX()));
	//	pcl::transformPointCloud(*M_pPointCloud_octomap, *M_pPointCloud_octomap, M_Trans);
	//	//add PC to octomap
	//	//M_pOctoMapBuilder->setSensorState(CCalculus::getInstance()->transformLocal2GlobalCoordinate(M_OnboardXtionState, M_DRState));
	//	//M_pOctoMapBuilder->setSensorState(CCalculus::getInstance()->transformLocal2GlobalCoordinate(M_OnboardXtionState, State_map));
	//	CRobotState State_zero; State_zero.setPose(0., 0., 0., 0., 0., 0.);
	//	M_pOctoMapBuilder->setSensorState(CCalculus::getInstance()->transformLocal2GlobalCoordinate(M_OnboardXtionState, State_zero));
	//	M_pOctoMapBuilder->setPointCloud(M_pPointCloud_octomap);
	//	M_pOctoMapBuilder->buildOctoMap();
	//	M_tree = M_pOctoMapBuilder->getOctoMap();
	//	//evaluation
	//	*M_pPointCloud_Evaluation += *M_pPointCloud_octomap;
	//}

	////TRUE octomap
	//{
	//	M_pPointCloud_Temp->clear();
	//	pcl::copyPointCloud(*M_pASUSXtionPointCloud, *M_pPointCloud_Temp);
	//	CRobotState State_map;
	//	State_map = M_TRUETrajectory_vec[M_i_cnt];
	//	M_Trans = Eigen::Affine3f::Identity();
	//	M_Trans.translation() << M_OnboardXtionState.getX(), M_OnboardXtionState.getY(), M_OnboardXtionState.getZ();
	//	M_Trans.rotate(Eigen::AngleAxisf(M_OnboardXtionState.getYaw(), Eigen::Vector3f::UnitZ()));
	//	M_Trans.rotate(Eigen::AngleAxisf(M_OnboardXtionState.getPitch(), Eigen::Vector3f::UnitY()));
	//	M_Trans.rotate(Eigen::AngleAxisf(M_OnboardXtionState.getRoll(), Eigen::Vector3f::UnitX()));
	//	pcl::transformPointCloud(*M_pPointCloud_Temp, *M_pPointCloud_Temp, M_Trans);
	//	if (M_b_ShowHorizontal) {
	//		//rotation of robot
	//		double delta_yaw = 36.9*D2R;
	//		Eigen::Matrix4d	Pos_Mat = Eigen::Matrix4d::Identity();
	//		Pos_Mat = calcPositoinMatrixFromXYZRPY(0., 0., 0., 0., 0., delta_yaw)
	//			* calcPositoinMatrixFromXYZRPY(
	//				State_map.getX(), State_map.getY(), State_map.getZ(),
	//				State_map.getRoll(), State_map.getPitch(), State_map.getYaw());
	//		Eigen::Vector6d Pos_XYZRPY = Eigen::Vector6d::Zero();
	//		Pos_XYZRPY = calcXYZRPYFromPositoinMatrix(Pos_Mat);
	//		State_map.setPose(Pos_XYZRPY(0, 0), Pos_XYZRPY(1, 0), Pos_XYZRPY(2, 0),
	//			Pos_XYZRPY(3, 0), Pos_XYZRPY(4, 0), Pos_XYZRPY(5, 0));
	//	}
	//	//transform coodinate from robot to world
	//	M_Trans = Eigen::Affine3f::Identity();
	//	M_Trans.translation() << State_map.getX(), State_map.getY(), State_map.getZ();
	//	M_Trans.rotate(Eigen::AngleAxisf(State_map.getYaw(), Eigen::Vector3f::UnitZ()));
	//	M_Trans.rotate(Eigen::AngleAxisf(State_map.getPitch(), Eigen::Vector3f::UnitY()));
	//	M_Trans.rotate(Eigen::AngleAxisf(State_map.getRoll(), Eigen::Vector3f::UnitX()));
	//	pcl::transformPointCloud(*M_pPointCloud_Temp, *M_pPointCloud_Temp, M_Trans);
	//	*M_pPointCloud_Evaluation_TRUE += *M_pPointCloud_Temp;
	//}

	////1202
	////show2
	//M_pPointCloud_show2_pre->clear();
	//pcl::copyPointCloud(*M_pPointCloud_octomap, *M_pPointCloud_show2_pre);

	//cout << "add PC to octomap" << endl;

	//string t2 = time_.getTimeString();
	//cout << "time elapse(Mapping) = " << time_.getTElapsefrom2S(t1, t2) << endl;
}

void CSLAM_Kataoka::do_Visualization()
{
	cout << "Visualization started" << endl;

	M_pPointCloud_show2_XYZI_pre->clear();
	for (int i = 0; i < M_pPointCloud_show2_pre->size(); i++) {
		pcl::PointXYZI point_show;
		point_show.x = M_pPointCloud_show2_pre->points[i].x;
		point_show.y = M_pPointCloud_show2_pre->points[i].y;
		point_show.z = M_pPointCloud_show2_pre->points[i].z;
		//point_show.intensity = 15;
		point_show.intensity = 40;
		M_pPointCloud_show2_XYZI_pre->push_back(point_show);
	}

	//cv::imshow("RGB Image", *M_pASUSXtionColorImage);
	//cv::imshow("Range Image", *M_pASUSXtionDepthImage);
	//cv::waitKey(1);
	//CRenderingObjects::getInstance()->setColorOctoMap(M_tree);
	//CRenderingObjects::getInstance()->setRobotState(M_pDREstimatedPathList->back());
	//CRenderingObjects::getInstance()->setEstimatedPathList0(M_pDREstimatedPathList, 153, 0, 255);

	//M_pPointCloud_show0_pre
	pcl::copyPointCloud(*M_pPointCloud_Evaluation, *M_pPointCloud_show0_pre);

	//M_pPointCloud_show1_pre
	pcl::copyPointCloud(*M_pPointCloud_Evaluation_TRUE, *M_pPointCloud_show1_pre);


	//modify(color)
	{
		////show0
		//M_pPointCloud_show0->clear();
		//for (int i = 0; i < M_pPointCloud_show0_pre->size(); i++) {
		//	pcl::PointXYZRGB point_;
		//	point_ = M_pPointCloud_show0_pre->points[i];
		//	//change color
		//	int range;
		//	range = 80;
		//	if ((int)(point_.r) + range > 255) point_.r = 255;
		//	else point_.r = point_.r + (uchar)(range);
		//	if ((int)(point_.g) + range > 255) point_.g = 255;
		//	else point_.g = point_.g + (uchar)(range);
		//	if ((int)(point_.b) + range > 255) point_.b = 255;
		//	else point_.b = point_.b + (uchar)(range);
		//	M_pPointCloud_show0->push_back(point_);
		//}
		////M_pPointCloud_show0->clear();
		////pcl::copyPointCloud(*M_pPointCloud_show0_pre, *M_pPointCloud_show0);

		//show0 20200405
		M_pPointCloud_show0->clear();
		for (size_t i = 0; i < M_pPointCloud_show0_pre->size(); i++)
		{
			pcl::PointXYZRGB point_ = M_pPointCloud_show0_pre->points[i];
			point_.r = 255;
			point_.g = 0;
			point_.b = 0;
			M_pPointCloud_show0->push_back(point_);
		}

		////show1
		//M_pPointCloud_show1->clear();
		//pcl::copyPointCloud(*M_pPointCloud_show1_pre, *M_pPointCloud_show1);

		//show1 20200405
		M_pPointCloud_show1->clear();
		for (size_t i = 0; i < M_pPointCloud_show1_pre->size(); i++)
		{
			pcl::PointXYZRGB point_ = M_pPointCloud_show1_pre->points[i];
			point_.r = 0;
			point_.g = 255;
			point_.b = 0;
			M_pPointCloud_show1->push_back(point_);
		}

		//show2
		M_pPointCloud_show2_XYZI->clear();
		pcl::copyPointCloud(*M_pPointCloud_show2_XYZI_pre, *M_pPointCloud_show2_XYZI);
	}


	if (M_b_ShowHorizontal) {
		//rotation of robot
		double delta_yaw = 36.9*D2R;
		//transform coodinate from robot to world
		M_Trans = Eigen::Affine3f::Identity();
		M_Trans.translation() << 0., 0., 0.;
		M_Trans.rotate(Eigen::AngleAxisf(delta_yaw, Eigen::Vector3f::UnitZ()));
		M_Trans.rotate(Eigen::AngleAxisf(0., Eigen::Vector3f::UnitY()));
		M_Trans.rotate(Eigen::AngleAxisf(0., Eigen::Vector3f::UnitX()));
		pcl::transformPointCloud(*M_pPointCloud_show0, *M_pPointCloud_show0, M_Trans);
		pcl::transformPointCloud(*M_pPointCloud_show1, *M_pPointCloud_show1, M_Trans);
	}

	////Target (need to turn)
	//CRenderingObjects::getInstance()->setPointCloud0(M_pPointCloud_show0);
	////Source (need to turn)
	//CRenderingObjects::getInstance()->setPointCloud1(M_pPointCloud_show1);
	////After Registration  (green)
	//CRenderingObjects::getInstance()->setPointCloud2(M_pPointCloud_show2_XYZI);

	////Target (need to turn)
	//CRenderingObjects::getInstance()->setPointCloud0(M_pPointCloud_show0);
	////Source (need to turn)
	//CRenderingObjects::getInstance()->setPointCloud1(M_pPointCloud_show1);
	////After Registration  (green)
	//CRenderingObjects::getInstance()->setPointCloud2(M_pPointCloud_show2_XYZI);
}