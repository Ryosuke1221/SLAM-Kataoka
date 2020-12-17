#pragma once

#include<iostream>
#include <vector>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

//detect plane
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/filters/extract_indices.h>

////https://akio-tanaka.tumblr.com/page/2
//#pragma comment(lib,"opengl32.lib")	
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

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

class CPointcloudBasic
{

public:
	CPointcloudBasic()
	{

	}

	void all_process();
	void FreeSpace();
	
	void FileProcess();
	void FileProcess_copy(string dir_from, string dir_to);
	void FileProcess_delete(string dir);
	void FileProcess_evacuate(string dir);
	void FileProcess_FolderInFolder(string dir_, vector<string> &folder_vec);

	void changeColor_plane(pcl::PointXYZRGB &point_);
	void changeColor_plane(pcl::PointXYZI &point_);

	template <class T_PointType>
	void detectPlane(pcl::PointCloud<T_PointType> &cloud_, double th_distance, bool b_cout = false, bool b_remove = false)
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

	template <class T_PointType>
	static Eigen::Matrix<float, 3, Eigen::Dynamic> calcEigenMatrixFromPointCloud(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_)
	{
		int npts = cloud_->size();
		if (npts == 0)
		{
			cout << "ERROR(CPointcloudBasic::calcEigenMatrixFromPointCloud): No point contained." << endl;
			throw std::runtime_error("ERROR(CPointcloudBasic::calcEigenMatrixFromPointCloud): No point contained.");
		}
		Eigen::Matrix<float, 3, Eigen::Dynamic> mat_(3, npts);
		{
			auto itr_points = cloud_->begin();
			for (int i = 0; i < npts; i++)
			{
				if (itr_points == cloud_->end()) break;
				mat_(0, i) = itr_points->x;
				mat_(1, i) = itr_points->y;
				mat_(2, i) = itr_points->z;
				++itr_points;
			}
		}
		return mat_;
	}

	template<typename Derived>
	static Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> calcCovarianceMatrix(const Eigen::MatrixBase<Derived>& mat_)
	{
		//argument example: 3 * Dynamic 
		int npts = mat_.cols();
		if (npts == 0)
		{
			cout << "ERROR(CPointcloudBasic::calcCovarianceMatrix): No point contained." << endl;
			throw std::runtime_error("ERROR(CPointcloudBasic::calcCovarianceMatrix): No point contained.");
		}
		const float one_over_n = 1. / static_cast<float>(npts);
		Eigen::Matrix<float, Eigen::Dynamic, 1> mat_mean(mat_.rows(), 1);
		mat_mean = mat_.rowwise().sum() * one_over_n;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat_demean(mat_.rows(), npts);
		mat_demean = mat_.colwise() - mat_mean;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat_cov(mat_.rows(), mat_.rows());
		mat_cov = mat_demean * mat_demean.transpose() * one_over_n;
		return mat_cov;
	}

	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);
	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(Eigen::Vector6d XYZRPY_arg);

	static Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);
	static Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

	void show_sequent();

	void DrawTrajectory();
	void DoMappingFromTrajectory();

};
