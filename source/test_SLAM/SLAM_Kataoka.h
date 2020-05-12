//#include <Eigen/Core>
#include <iostream>
#include <fstream>
//#include <stdlib.h>
#include <pcl/common/transforms.h>
#include <pcl/correspondence.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "RobotState.h"
#include "TimeString.h"

#include <random>
#include <sstream>

//RANSAC
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <boost/thread/thread.hpp>


//keypoint
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include "KataokaPCL.h"
//#include "Map_Looper.h"

#include<opencv2/opencv.hpp>


//kataoka 20191013
namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

class CSLAM_Kataoka
{
public:
	const static int OCTOMAP = 1;
	const static int OCTOMAPWITHFILE = 4;

private:
	int m_nProcMode;
	static void Thread(void* arg);

public:
	void execute(int nPeriod, int nProcMode);
	void initialize();
	void terminate();

	void doProcess_OctoMap();
	void doProcess_OctoMapWithFile();

	bool checkMapBuildingTiming(int nLoop);
public:
	CSLAM_Kataoka(void);
	~CSLAM_Kataoka(void);

	//typedef Eigen::Matrix<double, 6, 1> Vector6d;		//kataoka20191013

private:
	//-------Variables using in processes
	//CPioneer3DXInterface::EncoderData M_EncoderData;
	CRobotState M_DRState;
	CRobotState M_State;
	CRobotState M_OnboardXtionState;
	CRobotState M_OnboardLRFState;
	//CDeadReckoningLocalizer * M_pDRLocalizer;
	//CRangeBasedPFLocalizer * M_pRPFLocalizer;
	list<CRobotState> * M_pDREstimatedPathList;

	pcl::PointCloud<pcl::PointXYZ>::Ptr M_pASUSXtionRangeData;
	cv::Mat * M_pASUSXtionDepthImage;
	cv::Mat * M_pASUSXtionColorImage;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pASUSXtionPointCloud;

	cv::Mat * M_pWebcamRGBImage;
	float m_fResolution;
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> M_VGFilter;
	pcl::PassThrough<pcl::PointXYZRGB> M_PTFilter;
	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> M_RORFilter;
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr M_align_ICP;

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pFinal;
	//octomap::ColorOcTree * M_tree;
	//octomap::pose6d M_FrameOrigin;
	//octomap::point3d M_SensorOrigin;
	//octomap::Pointcloud M_OMPointCloud;
	//octomap::ColorOcTreeNode * M_NodeResult;
	//octomap::point3d M_query;

	//octomap::ScanNode M_ScanNode;
	//COctoMapBuilder * M_pOctoMapBuilder;

	//Eigen::Matrix4d M_transformation_matrix_ICP;
	Eigen::Affine3f M_Trans;
	ifstream M_ifstream;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pFilePointCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pFilePCloud;
	int M_nPointCnt;
	double ppoints, cpoints;
	ofstream M_fileout;
	//CTWODMapState * M_p2DMap;

	//kataoka0517
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_before;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_octomap;		//variable for adding to octomap


	//kataoka0716 edge
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pASUSXtionPointCloud_edge;
	cv::Mat * M_pASUSXtionDepthSavingImage;

	//kataoka0718 ransac
	//kataoka icp_modified
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_alignment_Source;				//temporal valiable for Alignment
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_alignment_Target;

	//FPFH RANSAC
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPC_FPFH_Source;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPC_FPFH_Target;
	pcl::PointCloud<pcl::Normal>::Ptr M_pPC_Normals_FPFH_Source;
	pcl::PointCloud<pcl::Normal>::Ptr M_pPC_Normals_FPFH_Target;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr M_Result_FPFH_Source;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr M_Result_FPFH_Target;
	pcl::SampleConsensusPrerejective<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr M_align_RANSAC;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr M_Tree_FPFH;
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> M_Normal;
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> M_FPFH_Source;
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> M_FPFH_Target;

	Eigen::Matrix4d M_transformation_matrix_RANSAC;

	//kataoka 0718
public:
	//vector<frame> graph;
	//vector<dataset_Open3D> GraphSLAM_open3d;
	vector<CRobotState> GraphSLAM_temp_RobotState;
	vector<CRobotState> Residual_error;
	//void do_Registration();
	void do_Registration(int i_method);

	void do_Localization();
	void do_EdgeExtraction_Image();
	CRobotState calc_Odometry_3variable(CRobotState DRState_arg, double DeltaX_odo, double DeltaY_odo, double DeltaYaw_odo);
	CRobotState calc_Odometry_6variable(CRobotState DRState_arg, double DeltaX_odo, double DeltaY_odo, double DeltaYaw_odo);

private:
	double M_x_initial, M_y_initial, M_z_initial, M_roll_initial, M_pitch_initial, M_yaw_initial;
	double M_x_ICP, M_y_ICP, M_z_ICP, M_roll_ICP, M_pitch_ICP, M_yaw_ICP;

	int M_i_cnt;

	void do_ArtificialPC();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_show0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_show1;
	pcl::PointCloud<pcl::PointXYZI>::Ptr M_pPointCloud_show2_XYZI;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_show0_pre;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_show1_pre;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_show2_pre;
	pcl::PointCloud<pcl::PointXYZI>::Ptr M_pPointCloud_show2_XYZI_pre;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_show3;
	pcl::PointCloud<pcl::PointXYZI>::Ptr M_pPointCloud_show3_XYZI;

	//Edge from Shape 20190730
public:
	void do_EdgeExtraction_Shape();
private:
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> M_Normal_Edge;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr M_Tree_Edge;
	pcl::PointCloud<pcl::Normal>::Ptr M_pPC_Normals_Edge;

public:
	void do_RANSAC_FPFH();

	//experiment
public:
	void do_proposed_weight();
	CRobotState do_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_src,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_tgt, int MaximumIterations,
		double MaxCorrespondenceDistance, double EuclideanFitnessEpsilon, double TransformationEpsilon,
		double RANSACOutlierRejectionThreshold, double th_translation_ICP, double th_rotation_ICP);
	CRobotState do_exp_ICP_proposed_weight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_src,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_tgt, int MaximumIterations,
		double MaxCorrespondenceDistance, double EuclideanFitnessEpsilon, double TransformationEpsilon,
		double RANSACOutlierRejectionThreshold, double th_translation_ICP, double th_rotation_ICP);

	void do_exp_CalculateDisplacementError(int i_which);

	CRobotState do_InicialRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_tgt);

	void do_exp_getPointCloudPtr_atFrame(int frame_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_arg);

	//void do_exp_getPointCloudPtr_fromOcoMap(octomap::ColorOcTree * tree_octomap, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_arg);

	void do_exp_CalculateEvaluation(int i_frame_start, int i_frame_end, int i_skip, double d_VGF);

	CRobotState	do_exp_ICP_proposed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_tgt,
		vector<int> chara_src_vec, vector<int> chara_tgt_vec,
		int MaximumIterations, double MaxCorrespondenceDistance, double EuclideanFitnessEpsilon, double TransformationEpsilon,
		double penalty_chara, double dist_search, double weight_dist_chara, int i_method_arg);

	void do_OutputTrajectoryCSV(int i_frame_start, int i_frame_end, int i_skip);
	void do_exp_Create_OdometryNoise(string filename_arg);
	void do_exp_CreateAndRead_DecreaseData_random(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_arg,
		vector<int> &chara_vec_arg, int num_arg, bool b_isFileExist);

private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_Artificial_Experiment;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_Temp;
	vector<int> M_chara_PC_vec;
	bool M_b_InputOdometryRead;
	vector<CRobotState> M_InputOdometry_vec;
	vector<CRobotState> M_InputOdometry_TRUE_vec;
	//vector<CRobotState> M_FinalOdometry_vec;
	vector<CRobotState> M_FinalOdometry_ICP_vec;
	vector<CRobotState> M_FinalOdometry_Proposed_vec;
	vector<CRobotState> M_TRUETrajectory_vec;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_chara0_src;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_chara1_src;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_chara2_src;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_chara0_tgt;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_chara1_tgt;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_chara2_tgt;

	//bool M_b_OdometryOnly;

	bool M_b_first;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_init_src;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_init_tgt;

	cv::Mat * M_pASUSXtionRGBImage_temp;
	cv::Mat * m_pASUSXtionDepthSavingImage_temp;

	CRobotState M_DRState_target;

	//boost::shared_ptr<CKataokaPCL> kataokaPCL;
	CKataokaPCL *M_kataokaPCL;

	double M_th_height_map;

	vector<int> M_Chara_Todai_src_vec;
	vector<int> M_Chara_Todai_tgt_vec;

	vector<CRobotState> M_InputTrajectory_vec;

	//vector< CPioneer3DXInterface::EncoderData> M_EncoderData_Noise_vec;

	//naraha
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> M_pPointCloud_naraha_vec;

public:
	void setPointCloud_Naraha(string filename_);
	Eigen::Matrix4d calcPositoinMatrixFromXYZRPY(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);
	Eigen::Vector6d calcXYZRPYFromPositoinMatrix(Eigen::Matrix4d transformation_Node);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Transformation_PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempPC, int cornum);

	//void Output_GraphbasedArc(dataset_Open3D output_data);

	//void Output_Graphbased_PointCloud(dataset_Open3D output_data2);
	/*CRobotState Transformation_LoopArc(CRobotState taregt_Robot, CRobotState source_Robot, pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetPC, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePC);*/
	CRobotState  Transformation_LoopArc(CRobotState start_Robot, CRobotState end_Robot, Eigen::Matrix4d transformation_matrix_ICP);

	void do_exp_ReadOdometrySlSr_Naraha(string filename_);

	void do_exp_Read_Trajectory(string filename_arg);

	void do_exp_ReadTRUETrajectory(string filename_);


private:
	struct SEncoderSlSr {
		double Sl;
		double Sr;
	};
	vector<SEncoderSlSr> M_EncoderDataSlSr_vec;

	//hand closure
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_00_filtered;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_13_filtered;

	//vector<int> M_Chara_src_vec;
	//vector<int> M_Chara_tgt_vec;

	//add
public:
	bool isLoopCandidate();
	void do_LoopClosing(int frame_loop);
	void do_exp_DecreasePointCloud(int frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_arg);
	void do_exp_CreateAndRead_DecreaseData_Area(int frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_arg);

private:
	double M_d_VGF;
	vector<vector<int>>	 M_Chara_vec_vec;

	//kataoka
public:
	void do_CharaClosing(int i_frame_end_arg);
	void do_CharaClosing1121(int i_frame_end_arg, vector<CRobotState> Trajectory_, bool b_ReakOnly = true);
	void do_CharaClosing_Check();

	struct Landmark {

	public:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud;
		int index_landmark;
		int frame_observed;
		Landmark() {
			pPointCloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
			index_landmark = 0;
			frame_observed = 0;
		}

		//Landmark& operator = (const Landmark& obj) {
		//	if (this == &obj) return *this;
		//	pcl::copyPointCloud(*obj.pPointCloud,*pPointCloud);
		//	index_landmark = obj.index_landmark;
		//	frame_observed = obj.frame_observed;
		//	return *this;
		//}
		//Landmark(const Landmark& obj) {
		//	pcl::copyPointCloud(*obj.pPointCloud, *pPointCloud);
		//	index_landmark = obj.index_landmark;
		//	frame_observed = obj.frame_observed;
		//}

	};

	struct Pair_Landmark {

	public:
		Landmark landmark_start;
		Landmark landmark_end;
		int frame_start;
		int frame_end;
		int index_landmark;

		Pair_Landmark() {
			frame_start = 0;
			frame_end = 0;
			index_landmark = 0;
		}

		//Pair_Landmark& operator = (const Pair_Landmark& obj) {
		//	if (this == &obj) return *this;
		//	landmark_start = obj.landmark_start;
		//	landmark_end = obj.landmark_end;
		//	frame_start = obj.frame_start;
		//	frame_end = obj.frame_end;
		//	index_landmark = obj.index_landmark;
		//	return *this;
		//}
		//Pair_Landmark(const Pair_Landmark& obj) {
		//	landmark_start = obj.landmark_start;
		//	landmark_end = obj.landmark_end;
		//	frame_start = obj.frame_start;
		//	frame_end = obj.frame_end;
		//	index_landmark = obj.index_landmark;

		//}
	};
	//ÉtÉåÅ[ÉÄÇÕê¨ï™Ç…ì¸ÇÍÇ»Ç≠ÇƒÇ‡Ç¢Ç¢ÇÃÇ©Ç‡ÅD

	int do_CalcLandmarkIndex(double X_, double Y_);

	void do_exp_CreateAndRead_CharaData();
	void do_exp_CreateAndRead_CharaData_1130(int i_frame_end_arg, bool b_useColor = false);
	//vector<int> do_GetCharaData(int frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_arg, bool b_useColor = false);
	vector<int> do_GetCharaData(int frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_arg);
	void do_CheckPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_arg, bool b_insert = false, string s_ = "");
	Eigen::Vector3d do_CalcCenterOfGravityOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_arg, bool b_cout = false);
private:
	vector<CRobotState> M_DREstimatedPath;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> M_cloud_vec_temp;
	//vector<vector<int>> M__Chara_vec_vec_temp;
	vector<Pair_Landmark> M_pair_landmark_vec;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_Evaluation;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_Evaluation_TRUE;

	vector<vector<float>> M_Intensity_vec_vec;

	//kataoka 20200405
	bool M_b_ReadOnly;
	CRobotState M_DRState_Displacement_odometry;
	bool M_b_ShowHorizontal;

public:
	void do_Visualization();
	void do_exp_CalculateEvaluation_202004(int i_frame_start, int i_frame_end, int i_skip);
	void do_Mapping();
	void do_SensorDataProcessing(int frame_first_arg, int frame_final_arg, int frame_skip_arg);
};