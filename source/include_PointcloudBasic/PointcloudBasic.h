#pragma once

#define D2R 0.017453288888889
#define R2D 57.29579143313326
#define M_PI 3.141592

#include <iostream>
#include <vector>
#include <random>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/impl/correspondence_estimation.hpp>
#include <pcl/registration/registration.h>

////https://akio-tanaka.tumblr.com/page/2
//#pragma comment(lib,"opengl32.lib")	
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

//should be under pcl includes
#include <windows.h>
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

	static void print4x4Matrix(const Eigen::Matrix4d & matrix);

	static double getDistanceOf2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud1_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud2_arg);

	template <class T_PointType>
	static void estimateRigidTransformation_static(
		const boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src_arg,
		const boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt_arg,
		const pcl::Correspondences &correspondences,
		Eigen::Matrix4f &transformation_matrix)
	{
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src_corr(new pcl::PointCloud<T_PointType>());
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt_corr(new pcl::PointCloud<T_PointType>());

		for (int i = 0; i < correspondences.size(); i++)
		{
			cloud_src_corr->push_back(cloud_src_arg->points[correspondences[i].index_query]);
			cloud_tgt_corr->push_back(cloud_tgt_arg->points[correspondences[i].index_match]);
		}
		const int npts = static_cast <int> (correspondences.size());
		if (npts == 0)
		{
			cout << "input has no corrs in CExtendableICP::estimateRigidTransformation_static" << endl;
			transformation_matrix = Eigen::Matrix4f::Identity();
			return;
		}
		typedef float Scalar;
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts);
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, npts);
		{
			auto itr_src = cloud_src_corr->begin();
			auto itr_tgt = cloud_tgt_corr->begin();
			for (int i = 0; i < npts; ++i)
			{
				if (itr_src == cloud_src_corr->end()) break;
				if (itr_tgt == cloud_tgt_corr->end()) break;
				cloud_src(0, i) = itr_src->x;
				cloud_src(1, i) = itr_src->y;
				cloud_src(2, i) = itr_src->z;
				++itr_src;
				cloud_tgt(0, i) = itr_tgt->x;
				cloud_tgt(1, i) = itr_tgt->y;
				cloud_tgt(2, i) = itr_tgt->z;
				++itr_tgt;
			}
		}

		// Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
		{
			//typedef typename internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type TransformationMatrixType;
			typedef typename Eigen::Matrix4f TransformationMatrixType;
			//typedef typename internal::traits<TransformationMatrixType>::Scalar Scalar;
			typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;

			//EIGEN_STATIC_ASSERT(!NumTraits<Scalar>::IsComplex, NUMERIC_TYPE_MUST_BE_REAL)
			//	EIGEN_STATIC_ASSERT((internal::is_same<Scalar, typename internal::traits<OtherDerived>::Scalar>::value),
			//		YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

				//enum { Dimension = EIGEN_SIZE_MIN_PREFER_DYNAMIC(Derived::RowsAtCompileTime, OtherDerived::RowsAtCompileTime) };
			enum { Dimension = 3 };

			typedef Eigen::Matrix<Scalar, Dimension, 1> VectorType;
			typedef Eigen::Matrix<Scalar, Dimension, Dimension> MatrixType;
			//typedef typename internal::plain_matrix_type_row_major<Derived>::type RowMajorMatrixType;

			typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;

			const Index m = cloud_src.rows(); // dimension
			const Index n = cloud_src.cols(); // number of measurements

			// required for demeaning ...
			const RealScalar one_over_n = RealScalar(1) / static_cast<RealScalar>(n);

			// computation of mean
			//óÒï˚å¸Ç…ë´ÇµéZÇµÇƒÇ¢ÇÈ
			const VectorType src_mean = cloud_src.rowwise().sum() * one_over_n;
			const VectorType tgt_mean = cloud_tgt.rowwise().sum() * one_over_n;

			// demeaning of src and dst points
			//óÒï˚å¸Ç…ã§í ÇÃóÒÉxÉNÉgÉãÇà¯Ç´éZÇµÇƒÇ¢ÇÈÅD
			//const RowMajorMatrixType src_demean = src.colwise() - src_mean;
			//const RowMajorMatrixType dst_demean = dst.colwise() - dst_mean;
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> src_demean(3, npts);
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> tgt_demean(3, npts);
			src_demean = cloud_src.colwise() - src_mean;
			tgt_demean = cloud_tgt.colwise() - tgt_mean;

			// Eq. (36)-(37)
			const Scalar src_var = src_demean.rowwise().squaredNorm().sum() * one_over_n;

			// Eq. (38)
			const MatrixType sigma = one_over_n * tgt_demean * src_demean.transpose();

			Eigen::JacobiSVD<MatrixType> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

			// Initialize the resulting transformation with an identity matrix...
			Eigen::Matrix4f Rt = TransformationMatrixType::Identity(m + 1, m + 1);

			// Eq. (39)
			VectorType S = VectorType::Ones(m);

			if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
				S(m - 1) = -1;

			// Eq. (40) and (43)	
			//âÒì]ê¨ï™ÇÃÇ›Ç…ë„ì¸
			Rt.block(0, 0, m, m).noalias() = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

			//ï¿êiê¨ï™ÇÃÇ›Ç…ë„ì¸
			Rt.col(m).head(m) = tgt_mean;
			//ï¿êiê¨ï™ÇÃÇ›Ç…ë„ì¸
			//âÒì]ê¨ï™ÇÃÇ›åƒÇ—èoÇµ
			Rt.col(m).head(m).noalias() -= Rt.topLeftCorner(m, m)*src_mean;

			transformation_matrix = Rt;
		}

	}

	static float getCorrMedianDistance(pcl::Correspondences correspondences);

	void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &input_arg,
		pcl::PointCloud<pcl::PointXYZRGB> &output_arg, Eigen::Matrix4f transformation_arg);

	//20200519
	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);
	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(Eigen::Vector6d XYZRPY_arg);
	static Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);
	static Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

	static Eigen::Vector6d calcRobotPosition_3DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
		Eigen::Vector6d pose_sensor, Eigen::Vector6d disp_registration);
	static Eigen::Vector6d calcRobotPosition_6DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
		Eigen::Vector6d pose_sensor, Eigen::Vector6d disp_registration);

	template <class T_PointType>
	static Eigen::Vector3d getWeightPoint_3D(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_)
	{
		Eigen::Vector3d sum_vec = Eigen::Vector3d::Zero();
		Eigen::Vector3d weight_vec = Eigen::Vector3d::Zero();
		if (cloud_->size() == 0)
		{
			cout << "ERROR: no point found" << endl;
			return weight_vec;
		}
		for (size_t i = 0; i < cloud_->size(); i++)
		{
			sum_vec(0, 0) = sum_vec(0, 0) + cloud_->points[i].x;
			sum_vec(1, 0) = sum_vec(1, 0) + cloud_->points[i].y;
			sum_vec(2, 0) = sum_vec(2, 0) + cloud_->points[i].z;
		}
		weight_vec(0, 0) = sum_vec(0, 0) / cloud_->size();
		weight_vec(1, 0) = sum_vec(1, 0) / cloud_->size();
		weight_vec(2, 0) = sum_vec(2, 0) / cloud_->size();
		return weight_vec;
	}

	template <class T_PointType>
	static Eigen::Vector2d getWeightPoint_2D(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_)
	{
		Eigen::Vector2d sum_vec = Eigen::Vector2d::Zero();
		Eigen::Vector2d weight_vec = Eigen::Vector2d::Zero();
		if (cloud_->size() == 0)
		{
			cout << "ERROR: no point found" << endl;
			return weight_vec;
		}
		for (size_t i = 0; i < cloud_->size(); i++)
		{
			sum_vec(0, 0) = sum_vec(0, 0) + cloud_->points[i].x;
			sum_vec(1, 0) = sum_vec(1, 0) + cloud_->points[i].y;
		}
		weight_vec(0, 0) = sum_vec(0, 0) / cloud_->size();
		weight_vec(1, 0) = sum_vec(1, 0) / cloud_->size();
		return weight_vec;
	}

	template <class T_PointType>
	static float getMedianDistance(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt)
	{
		//corr
		pcl::Correspondences correspondences = determineCorrespondences_output(cloud_src, cloud_tgt);
		//call median function
		return getCorrMedianDistance(correspondences);
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_output(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,
		float th_distance = 100.)
	{
		pcl::Correspondences correspondences;
		correspondences.resize(cloud_src->size());
		std::vector<int> index(1);
		std::vector<float> distance(1);
		unsigned int nr_valid_correspondences = 0;
		pcl::KdTreeFLANN<T_PointType> match_search;
		match_search.setInputCloud(cloud_tgt->makeShared());
		for (size_t i = 0; i < cloud_src->size(); ++i)
		{
			int found_neighs = match_search.nearestKSearch(cloud_src->at(i), 1, index, distance);
			if (th_distance * th_distance < distance[0]) continue;
			pcl::Correspondence corr;
			corr.index_query = i;
			corr.index_match = index[0];
			corr.distance = distance[0];	//squared
			correspondences[nr_valid_correspondences++] = corr;
		}
		correspondences.resize(nr_valid_correspondences);
		return correspondences;
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_kdtreeArg_num(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt,
		int num_nearest)
	{
		pcl::Correspondences correspondences;
		correspondences.resize(cloud_src->size());
		std::vector<int> index(1);
		std::vector<float> distance(1);
		unsigned int nr_valid_correspondences = 0;
		for (size_t i = 0; i < cloud_src->size(); ++i)
		{
			int found_neighs = kdtree_tgt->nearestKSearch(cloud_src->at(i), num_nearest, index, distance);
			pcl::Correspondence corr;
			corr.index_query = i;
			corr.index_match = index[0];
			corr.distance = distance[0];	//squared
			correspondences[nr_valid_correspondences++] = corr;
		}
		correspondences.resize(nr_valid_correspondences);
		return correspondences;
	}

	static pcl::Correspondences getCorrespondences_eachPairHaving(const pcl::Correspondences &corr_src_tgt, const pcl::Correspondences &corr_tgt_src);

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_kdtreeArg_eachPairHaving_num(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_tgt,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_src, boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt,
		int num_nearest)
	{
		pcl::Correspondences corr_src_tgt = determineCorrespondences_kdtreeArg_num(cloud_feature_src, kdtree_tgt, num_nearest);
		pcl::Correspondences corr_tgt_src = determineCorrespondences_kdtreeArg_num(cloud_feature_tgt, kdtree_src, num_nearest);
		pcl::Correspondences corr_new = getCorrespondences_eachPairHaving(corr_src_tgt, corr_tgt_src);
		return corr_new;
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_kdtreeArg(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt, float th_value, bool b_multipleNear = false)
	{
		pcl::Correspondences correspondences;
		correspondences.resize(cloud_src->size());
		std::vector<int> index(1);
		std::vector<float> distance(1);
		unsigned int nr_valid_correspondences = 0;
		for (size_t j = 0; j < cloud_src->size(); ++j)
		{
			//int found_neighs = kdtree_tgt->nearestKSearch(cloud_src->at(i), num_nearest, index, distance);
			int found_neighs = kdtree_tgt->radiusSearch(cloud_src->at(j), th_value, index, distance);
			pcl::Correspondence corr;
			corr.index_query = j;
			if (!b_multipleNear)
			{
				corr.index_match = index[0];
				corr.distance = distance[0];	//squared
				correspondences[nr_valid_correspondences++] = corr;
			}
			else
			{
				for (int i = 0; i < index.size(); i++)
				{
					corr.index_match = index[i];
					corr.distance = distance[i];	//squared
					correspondences[nr_valid_correspondences++] = corr;
				}
			}
		}
		correspondences.resize(nr_valid_correspondences);
		return correspondences;
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_kdtreeArg_eachPairHaving(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_tgt,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_src, boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt, float th_value)
	{
		pcl::Correspondences corr_src_tgt = determineCorrespondences_kdtreeArg(cloud_feature_src, kdtree_tgt, th_value);
		pcl::Correspondences corr_tgt_src = determineCorrespondences_kdtreeArg(cloud_feature_tgt, kdtree_src, th_value);
		pcl::Correspondences corr_new = getCorrespondences_eachPairHaving(corr_src_tgt, corr_tgt_src);
		return corr_new;
	}

	template <class T_PointType>
	static Eigen::Matrix<float, 3, Eigen::Dynamic> calcEigenMatrixFromPointCloud(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_)
	{
		int npts = cloud_->size();
		if (npts == 0)
		{
			cout << "ERROR(CExtendableICP::calcEigenMatrixFromPointCloud): No point contained." << endl;
			throw std::runtime_error("ERROR(CExtendableICP::calcEigenMatrixFromPointCloud): No point contained.");
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
			cout << "ERROR(CExtendableICP::calcCovarianceMatrix): No point contained." << endl;
			throw std::runtime_error("ERROR(CExtendableICP::calcCovarianceMatrix): No point contained.");
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

};
