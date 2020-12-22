#pragma once

#include<iostream>
#include <vector>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/ModelCoefficients.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_plotter.h>

////https://akio-tanaka.tumblr.com/page/2
//#pragma comment(lib,"opengl32.lib")	
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

#include"ExtendableICP.h"
#include"FPFH_PCL.h"
#include "PointVisualization.h"

//should be under pcl includes
#include<windows.h>
#include "TimeString.h"

#define M_PI 3.14159265359
#define D2R 0.017453288888889
#define R2D 57.29579143313326

using namespace std;

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

class CPointcloudFunction
{

public:
	CPointcloudFunction()
	{

	}

	void all_process();
	void show_sequent();
	void getPCDFromCSV_naraha();
	void getPCDFromCSV_gotFromPCAP(string dir_save, string dir_data, string file_RelativePath_);
	void FreeSpace();
	void filterNIRPointCloud_naraha();
	void getCSVFromPointCloud();
	void combinePointCloud_naraha();

	template <class T_PointType>
	void detectPlane(pcl::PointCloud<T_PointType> &cloud_,double th_distance, bool b_cout = false, bool b_remove = false)
	{
		//https://qiita.com/akachochin/items/47f1470565e76adb1880
		//https://www.slideshare.net/masafuminoda/pcl-11030703
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object  
		pcl::SACSegmentation<T_PointType> seg;
		// Optional  
		seg.setOptimizeCoefficients(true);
		// Mandatory  
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(200);
		seg.setDistanceThreshold(th_distance);	
		seg.setInputCloud(cloud_.makeShared());
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
			PCL_ERROR("Could not estimate a planar model for the given dataset.");

		if (b_cout)
		{
			cout << "Model coefficients:";
			for (int i = 0; i < coefficients->values.size(); i++)
				cout << " " << coefficients->values[i];
			cout << " (pitch[deg]: " << -asin(coefficients->values[0]) * 180. / M_PI << ")";
			cout << endl;
			std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
		}
		for (size_t i = 0; i < inliers->indices.size(); ++i)
			changeColor_plane(cloud_.points[inliers->indices[i]]);

		if (b_remove)
		{
			pcl::ExtractIndices<T_PointType> extract;
			extract.setInputCloud(cloud_.makeShared());
			extract.setIndices(inliers);
			extract.setNegative(true); //true: removing plane, false: removing except plane
			extract.filter(cloud_);
		}
	}

	void changeColor_plane(pcl::PointXYZRGB &point_);
	void changeColor_plane(pcl::PointXYZI &point_);

	void DynamicTranslation();

	void FileProcess();
	void FileProcess_copy(string dir_from, string dir_to);
	void FileProcess_delete(string dir);
	void FileProcess_evacuate(string dir);
	void FileProcess_FolderInFolder(string dir_, vector<string> &folder_vec);

private:
	enum KEYNUM {
		NONE,
		ZERO,
		X_,
		Y_,
		Z_,
		ROLL_,
		PITCH_,
		YAW_,
		X_MINUS,
		Y_MINUS,
		Z_MINUS,
		ROLL_MINUS,
		PITCH_MINUS,
		YAW_MINUS,
		ENTER,
		RSHIFT,
		RCTRL,
		ESC
	};
public:
	//should declare under enum type declaration
	KEYNUM getKEYNUM();
	void DrawTrajectory();
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

	void DoMappingFromTrajectory();

};
