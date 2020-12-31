#pragma once

#include<iostream>
#include <vector>
#include <random>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_plotter.h>

#include "PointcloudBasicProcess.h"
#include "PointVisualization.h"
#include "ExtendableICP.h"
#include "FPFH_PCL.h"

//should be under pcl includes
#include<windows.h>
#include "TimeString.h"

using namespace std;

class CPointcloudFunction : public CPointcloudBasicProcess
{

public:
	CPointcloudFunction()
	{

	}

	void all_process();
	void getPCDFromCSV_naraha();
	void getPCDFromCSV_gotFromPCAP(string dir_save, string dir_data, string file_RelativePath_);
	void FreeSpace();
	void filterNIRPointCloud_naraha();
	void getCSVFromPointCloud();
	void combinePointCloud_naraha();

	void DynamicTranslation();

	void DoSegmentation();

	void GlobalRegistration_FPFH_SAC_IA();
	void GR_FPFH_getResultAnalysis(string dir_, string s_folder);
	void GR_FPFH_makeFusionCSV(string dir_, string s_folder);
	vector<vector<string>> GR_FPFH_makeMatrix(vector<vector<int>> int_vecvec);
	void GR_FPFH_makeSuccessEstimation(string dir_);
	void GR_FPFH_getResultOfPatterns(string dir_);
	vector<string> GR_FPFH_getResultOfOnePattern(string dir_, string s_folder);
	void GR_FPFH_SAC_IA_2frames(string dir_, vector<float> parameter_vec);
	void GR_FPFH_SAC_IA_Allframes(string dir_, vector<float> parameter_vec, bool b_changeParameter = true);
	vector<string> GR_FPFH_SAC_IA_Allframes_OnePair(string dir_, vector<float> parameter_vec, int i_tgt, int i_src,
		vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> cloud_vec, vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec,
		vector<Eigen::Vector6d> trajectory_vec, bool b_useRANSAC_EST, bool b_useOutputPointcloud);
	vector<pair<int, int>> GR_FPFH_SAC_IA_get_frame_pair_vec(string dir_);
	void GR_addToOutputString_OutputHeader_FPFH(vector<vector<string>> &s_output_vecvec);
	void GR_FPFH_error(string dir_, vector<float> parameter_vec);
	void GR_FPFH_error_AllFrames(string dir_, vector<float> parameter_vec, bool b_changeParameter = true);
	void GR_FPFH_variance_AllFrames(string dir_, vector<float> parameter_vec, bool b_changeParameter = true);
	void GR_FPFH_varyParameter(string dir_, vector<float> parameter_vec_arg);
	void GR_FPFH_FixFusion(string dir_, string s_folder);

	void DoOutlierRejector();

	void DoICP_proposed_AllFrames();
	void DoICP_proposed_givenParameter(string dir_, string filename_csv, vector<float> parameter_vec);
	void DoICP_proposed_only1method(
		string dir_, string s_folder, vector<vector<string>> &s_input_vecvec, vector<float> parameter_vec, int i_method);
	void DoICP_addToOutputString_OutputHeader(vector<vector<string>> &s_output_vecvec);
	void DoICP_proposed_varyParameters(string dir_, string filename_csv);
	void DoICP_proposed_mergeResult();
	vector<string> DoICP_proposed_mergeResult_OnePattern(string dir_, string s_folder, bool b_useMedian);

	void DoEvaluation_ICP_property();
	void DoEvaluation_ICP_property_files(string dir_);
	void DoEvaluation_ICP_property_addToFile(string dir_,
		string s_newfoldername, vector<vector<string>> &s_input_vecvec,
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vec);
	void DoEvaluation_ICP_property_calculation(string dir_, string s_folder, vector<Eigen::Vector6d> trajectoryVector_vec,
		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_vec, vector<Eigen::Vector6d> trajectoryVector_vec_TRUE,
		int i_method, vector<double> &error_relative_vec, vector<double> &error_absolute_vec,
		vector<double> &frameCloudMedian_vec, double &map_mean);
	void DoEvaluation_ICP_property_mergeResult(string dir_);
	vector<string> DoEvaluation_ICP_property_mergeResult_OnePattern(string dir_, string s_folder);

};
