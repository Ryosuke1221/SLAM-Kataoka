#pragma once

#define D2R 0.017453288888889
#define R2D 57.29579143313326
#define M_PI 3.141592

#include <sstream>
#include <iostream>
#include <fstream>
#include <random>

#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/impl/correspondence_estimation.hpp>
#include <pcl/registration/registration.h>

#include <pcl/pcl_base.h>

#include <pcl/registration/default_convergence_criteria.h>

#include <Eigen/Core>

#include <pcl/registration/icp.h>

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/statistical_outlier_removal.h>


#include"KataokaCorrespondence.h"
#include"KataokaConvergence.h"
#include "TimeString.h"


namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

using namespace std;


class __declspec(dllexport) CKataokaPCL {

public:

	CKataokaPCL();
	~CKataokaPCL();

	typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;

	typedef typename KdTree::Ptr KdTreePtr;

	typedef std::vector< pcl::Correspondence, Eigen::aligned_allocator<pcl::Correspondence> > Correspondences;

	typedef boost::shared_ptr<Correspondences> CorrespondencesPtr;

	typedef typename pcl::registration::
		TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB, float> TransformationEstimation;

	typedef typename TransformationEstimation::Ptr TransformationEstimationPtr;

private:

	KdTreePtr tree_;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_source;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_target;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr M_pPointCloud_src_transformed;

	//pcl::PointCloud<pcl::PointXYZRGB> output;

	int M_max_iterations_;
	double M_euclidean_fitness_epsilon_, M_transformation_epsilon_, M_corr_dist_threshold_;

	int M_min_number_correspondences_;

	bool M_converged_;

	int M_nr_iterations_;

	Eigen::Matrix4f M_transformation_, M_previous_transformation_, M_final_transformation_, M_transformation_arg;

	Eigen::Vector6d M_final_transformation_Vec;


	pcl::registration::DefaultConvergenceCriteria<float>::Ptr M_convergence_criteria_;


	//pcl::CorrespondencesPtr M_correspondences_;
	CorrespondencesPtr_Kataoka M_correspondences_original;

public:

	void align();
	void align(Eigen::Matrix4d init);

	void computeTransformation();
	void determineCorrespondences(pcl::Correspondences &correspondences, double max_distance);

	Eigen::Matrix4f getFinalTransformation() {	return M_final_transformation_;	}

	void setInputSource(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_) {
		M_pPointCloud_source = input_;
	}

	void setInputTarget(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_) {
		M_pPointCloud_target = input_;
	}

	void setMaximumIterations(int MaximumIterations_arg)
	{
		M_max_iterations_ = MaximumIterations_arg;
	}

	void setMaxCorrespondenceDistance(double MaxCorrespondenceDistance_arg)
	{
		M_corr_dist_threshold_ = MaxCorrespondenceDistance_arg;
	}

	void setEuclideanFitnessEpsilon(double EuclideanFitnessEpsilon_arg)
	{
		M_euclidean_fitness_epsilon_ = EuclideanFitnessEpsilon_arg;
	}

	void setTransformationEpsilon(double TransformationEpsilon_arg) 
	{
		M_transformation_epsilon_ = TransformationEpsilon_arg;
	}


	bool hasConverged() 
	{
		return M_converged_;
	}

	double getFitnessScore();

	void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &input_arg,
		pcl::PointCloud<pcl::PointXYZRGB> &output_arg, Eigen::Matrix4f transformation_arg);

	void print4x4Matrix(const Eigen::Matrix4d & matrix);

	//proposed method
private:
	vector<int> M_chara_src_vec;
	vector<int> M_chara_tgt_vec;

	//double M_proposed_penalty_chara, M_proposed_dist_search, M_proposed_weight_dist_chara;
	double M_proposed_penalty_chara, M_proposed_weight_dist_chara;

public:
	void setCharaVector_src(vector<int> chara_vec);

	void setCharaVector_tgt(vector<int> chara_vec);

	void setCharaParameter(double penalty_chara, double weight_dist_chara) 
	{
		M_proposed_penalty_chara = penalty_chara;
		M_proposed_weight_dist_chara = weight_dist_chara;
	}

	void determineCorrespondences_chara(pcl::Correspondences &correspondences,
		double penalty_chara, double dist_search, double weight_dist_chara);

	double getFitnessScore_chara();

	double getDistanceOf2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud1_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud2_arg);

	//void determineCorrespondences_argPC(pcl::Correspondences &correspondences, double max_distance,
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_tgt);

	int do_exp_getCharaOfPoint_Todai(double x_, double y_, double z_);

	bool do_exp_getIsRemovedPoint_Todai(double x_, double y_, double z_);

	void determineCorrespondences_argPC_chara(pcl::Correspondences &correspondences, double max_distance,
		double penalty_chara, double dist_search_arg, double weight_dist_chara,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_src_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_tgt_arg,
		vector<int> Chara_start_vec, vector<int> Chara_end_vec);

	//convergence
private:
	boost::shared_ptr<CKataokaConvergence_> M_convergence_criteria_original;
	boost::shared_ptr<CKataokaConvergence_> M_convergence_criteria_Spring1;
	boost::shared_ptr<CKataokaConvergence_> M_convergence_criteria_Spring2;


	//estimateRigidTransformation
public:

	template <typename PointSource, typename PointTarget> void
		estimateRigidTransformation(
			const pcl::PointCloud<PointSource> &cloud_src_arg,
			const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
			const pcl::Correspondences &correspondences,
			Eigen::Matrix4f &transformation_matrix) const;

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


	template <typename Derived, typename OtherDerived>
	Eigen::Matrix4f	umeyama(const Eigen::MatrixBase<Derived>& src, const Eigen::MatrixBase<OtherDerived>& dst, bool with_scaling = true);

	//proposed method spring
private:
	vector<vector<double>> M_spring_src_vecvec;
	vector<vector<double>> M_spring_tgt_vecvec;
	vector<double> M_WeightConstant_Spring1_vec;
	int M_i_method;
	CorrespondencesPtr_Spring1 M_correspondences_spring1;
	CorrespondencesPtr_Spring2 M_correspondences_spring2;

public:
	void setSpring1VecVec_src(vector<vector<double>> spring_src_vecvec);
	void setSpring1VecVec_tgt(vector<vector<double>> spring_tgt_vecvec);
	void determineCorrespondences(Correspondences_Kataoka &correspondences, double max_distance);
	void determineCorrespondences_chara(Correspondences_Kataoka &correspondences,
		double penalty_chara, double dist_search_arg, double weight_dist_chara);

	template <typename PointSource, typename PointTarget> void
		estimateRigidTransformation(
			const pcl::PointCloud<PointSource> &cloud_src_arg,
			const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
			const Correspondences_Kataoka &correspondences,
			Eigen::Matrix4f &transformation_matrix) const;

	void setMothodInt(int i_method_arg)
	{
		M_i_method = i_method_arg;
	}

	void setParameter_Spring1(vector<double> WeightConstant_Spring1_vec)
	{
		M_WeightConstant_Spring1_vec = WeightConstant_Spring1_vec;
	}

	void determineCorrespondences_Spring1(Correspondences_Spring1 &correspondences,
		double max_distance, vector<double> WeightConstant_spring_vec);
	
	void determineCorrespondences_Spring2(Correspondences_Spring2 &correspondences,
		double max_distance, vector<double> WeightConstant_spring_vec);

	template <typename PointSource, typename PointTarget> void
		estimateRigidTransformation(
			const pcl::PointCloud<PointSource> &cloud_src_arg,
			const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
			const Correspondences_Spring1 &correspondences,
			Eigen::Matrix4f &transformation_matrix) const;

	Eigen::Vector6d getFinalTransformation_Vec() { return M_final_transformation_Vec; }

	template <typename PointSource, typename PointTarget> void
		estimateRigidTransformation(
			const pcl::PointCloud<PointSource> &cloud_src_arg,
			const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
			const Correspondences_Spring2 &correspondences,
			Eigen::Matrix4f &transformation_matrix) const;

	float getCorrMedianDistance(CorrespondencesPtr_Kataoka correspondences);
	float getCorrMedianDistance(CorrespondencesPtr_Spring1 correspondences);
	float getCorrMedianDistance(CorrespondencesPtr_Spring2 correspondences);
	static float getCorrMedianDistance(pcl::Correspondences correspondences);

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

	static int ICP_Chara_getCharaOfPoint_NarahaWinter(pcl::PointXYZRGB point_arg, vector<double> th_vec);
	static vector<int> ICP_Chara_GetCharaData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_PointCloud_arg);

	//template <class T_PointType>
	//static Eigen::Vector3d getWeightPoint(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_)
	//{
	//	Eigen::Vector3d sum_vec = Eigen::Vector3d::Zero();
	//	Eigen::Vector3d weight_vec = Eigen::Vector3d::Zero();
	//	if (cloud_->size() == 0)
	//	{
	//		cout << "ERROR: no point found" << endl;
	//		return weight_vec;
	//	}
	//	for (size_t i = 0; i < cloud_->size(); i++)
	//	{
	//		sum_vec(0, 0) = sum_vec(0, 0) + cloud_->points[i].x;
	//		sum_vec(1, 0) = sum_vec(1, 0) + cloud_->points[i].y;
	//		sum_vec(2, 0) = sum_vec(2, 0) + cloud_->points[i].z;
	//	}
	//	weight_vec(0, 0) = sum_vec(0, 0) / cloud_->size();
	//	weight_vec(1, 0) = sum_vec(1, 0) / cloud_->size();
	//	weight_vec(2, 0) = sum_vec(2, 0) / cloud_->size();
	//	return weight_vec;
	//}

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
	static vector<pcl::PointIndices> getSegmentation_indices(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg, float Tolerance, int MinClusterSize)
	{
		//https://vml.sakura.ne.jp/koeda/PCL/tutorials/html/cluster_extraction.html#cluster-extraction
		//https://www.slideshare.net/masafuminoda/pcl-11030703
		//std::cout << "PointCloud before filtering has: " << cloud_arg->points.size() << " data points." << std::endl;
		// Creating the KdTree object for the search method of the extraction
		boost::shared_ptr<pcl::search::KdTree<T_PointType>> tree(new pcl::search::KdTree<T_PointType>);
		tree->setInputCloud(cloud_arg);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<T_PointType> ec;
		ec.setClusterTolerance(Tolerance);//threshold; distance of clusters 
		ec.setMinClusterSize(MinClusterSize);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_arg);
		ec.extract(cluster_indices);
		return cluster_indices;
	}

	template <class T_PointType>
	static vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> getSegmentation_rest(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest, float Tolerance, int MinClusterSize)
	{
		std::vector<pcl::PointIndices> cluster_indices;
		cluster_indices = getSegmentation_indices(cloud_arg, Tolerance, MinClusterSize);

		vector<bool> b_rest_vec;
		for (int i = 0; i < cloud_arg->size(); i++)
			b_rest_vec.push_back(true);
		for (int j = 0; j < cluster_indices.size(); j++)
		{
			for (int i = 0; i < cluster_indices[j].indices.size(); i++)
				b_rest_vec[cluster_indices[j].indices[i]] = false;
		}

		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec;
		for (int j = 0; j < cluster_indices.size(); j++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_cluster(new pcl::PointCloud<T_PointType>);
			for (int i = 0; i < cluster_indices[j].indices.size(); i++)
				cloud_cluster->push_back(cloud_arg->points[cluster_indices[j].indices[i]]);
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			cloud_cluster_vec.push_back(cloud_cluster);
		}

		cloud_rest->clear();
		for (int i = 0; i < cloud_arg->size(); i++)
		{
			if (b_rest_vec[i] == true)
			{
				auto point = cloud_arg->points[i];
				cloud_rest->push_back(point);
			}
		}

		return cloud_cluster_vec;
	}

	template <class T_PointType>
	static vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> getSegmentation_robust(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest, float Tolerance, int MinClusterSize)
	{

		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec;
		cloud_rest->clear();

		cloud_cluster_vec = getSegmentation_rest(cloud_arg, cloud_rest, Tolerance, MinClusterSize);

		////next clustering (to rest)
		//{
		//	vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec_next;
		//	boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest_next(new pcl::PointCloud<T_PointType>);
		//	cloud_cluster_vec_next = getSegmentation_rest(cloud_rest, cloud_rest_next, Tolerance * 1.5, MinClusterSize);
		//	for (int i = 0; i < cloud_cluster_vec_next.size(); i++)
		//		cloud_cluster_vec.push_back(cloud_cluster_vec_next[i]);
		//	cloud_rest->clear();
		//	pcl::copyPointCloud(*cloud_rest_next, *cloud_rest);
		//}

		//next clustering (to cluster)
		{
			vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec_next;
			for (int j = 0; j < cloud_cluster_vec.size(); j++)
			{
				vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec_in1cluster;
				boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest_in1cluster(new pcl::PointCloud<T_PointType>);

				cloud_cluster_vec_in1cluster = getSegmentation_rest(cloud_cluster_vec[j], cloud_rest_in1cluster, Tolerance * 0.5, (float)MinClusterSize * 0.1);
				for (int i = 0; i < cloud_cluster_vec_in1cluster.size(); i++)
					cloud_cluster_vec_next.push_back(cloud_cluster_vec_in1cluster[i]);
				*cloud_rest += *cloud_rest_in1cluster;
			}
			cloud_cluster_vec.clear();
			cloud_cluster_vec = cloud_cluster_vec_next;
		}

		return cloud_cluster_vec;
	}

	template <class T_PointType>
	static void rejectOutlier(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> &cloud_output, float Tolerance, int MinClusterSize)
	{
		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec;
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_temp(new pcl::PointCloud<T_PointType>);
		cloud_cluster_vec = getSegmentation_robust(cloud_arg, cloud_temp, Tolerance, MinClusterSize);
		//cloud_cluster_vec = getSegmentation_rest(cloud_arg, cloud_temp, Tolerance, MinClusterSize);

		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_output_temp(new pcl::PointCloud<T_PointType>);
		for (int i = 0; i < cloud_cluster_vec.size(); i++)
			*cloud_output_temp += *cloud_cluster_vec[i];
		cloud_output->clear();
		pcl::copyPointCloud(*cloud_output_temp, *cloud_output);
	}

	template <class T_PointType>
	static void Remove_outliers(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> &cloud_output, int MeanK, float StddevMulThresh)
	{
		//http://virtuemarket-lab.blogspot.com/2015/03/outlier.html
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_filtered(new pcl::PointCloud<T_PointType>);
		pcl::StatisticalOutlierRemoval<T_PointType> sor;
		sor.setInputCloud(cloud_arg);
		sor.setMeanK(MeanK);
		sor.setStddevMulThresh(StddevMulThresh);
		sor.setNegative(false);
		sor.filter(*cloud_filtered);
		cloud_output->clear();
		pcl::copyPointCloud(*cloud_filtered, *cloud_output);
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

	inline static float getDistanceWeight(float distance_, float sigma_variance, int i_exponentiation = 2)
	{
		distance_ = fabs(distance_);
		if (i_exponentiation == 1) return 1. / (2. * M_PI * sigma_variance * sigma_variance) 
			* exp(-distance_ / (2. * sigma_variance * sigma_variance));
		else return 1. / (2. * M_PI * sigma_variance * sigma_variance)
			* exp(-distance_ * distance_ / (2. * sigma_variance * sigma_variance));
	}

	template <class T_PointType>
	static vector<Eigen::Vector3d> getPointCloud_featureGradient(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, const vector<float> &feature_vec,
		const vector<vector<int>> &nearIndex_vecvec, const vector<vector<float>> &squaredDistance_vecvec, float variance_)
	{
		vector<Eigen::Vector3d> featureGradientVector_vec;
		for (int j = 0; j < cloud_->size(); j++)
		{
			Eigen::Vector3d featureVector = Eigen::Vector3d::Zero();
			int num_near_valid = 0;
			for (int i = 0; i < nearIndex_vecvec[j].size(); i++)
			{
				if (squaredDistance_vecvec[j][i] == 0.) continue;
				T_PointType point_query = cloud_->points[j];
				T_PointType point_match = cloud_->points[nearIndex_vecvec[j][i]];
				Eigen::Vector3d featureVector_1point = Eigen::Vector3d::Zero();
				featureVector_1point <<
					point_match.x - point_query.x,
					point_match.y - point_query.y,
					point_match.z - point_query.z;
				float scara_ = (float)(feature_vec[nearIndex_vecvec[j][i]] - feature_vec[j])
					* getDistanceWeight(squaredDistance_vecvec[j][i], variance_)
					* getDistanceWeight(squaredDistance_vecvec[j][i], variance_);
				featureVector_1point = scara_ * featureVector_1point;
				featureVector(0, 0) += featureVector_1point(0, 0);
				featureVector(1, 0) += featureVector_1point(1, 0);
				featureVector(2, 0) += featureVector_1point(2, 0);
				num_near_valid++;
			}
			if (num_near_valid != 0) featureVector = 1. / (float)num_near_valid * featureVector;
			featureGradientVector_vec.push_back(featureVector);
		}
		return featureGradientVector_vec;
	}

	template <class T_PointType>
	static vector<float> getPointCloud_featureDivergence(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, const vector<float> &feature_vec,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_, const float th_distance, float variance_, bool b_useGaussianFilter = false)
	{
		vector<float> featureDivergence_vec;
		vector<vector<int>> nearIndex_vecvec;
		vector<vector<float>> squaredDistance_vecvec;
		//nearest
		for (int j = 0; j < cloud_->size(); j++)
		{
			vector<int> nearIndex_vec;
			vector<float> squaredDistance_vec;
			kdtree_->radiusSearch(*cloud_, j, th_distance, nearIndex_vec, squaredDistance_vec);
			nearIndex_vecvec.push_back(nearIndex_vec);
			squaredDistance_vecvec.push_back(squaredDistance_vec);
		}
		vector<Eigen::Vector3d> featureGradientVector_vec;
		featureGradientVector_vec = getPointCloud_featureGradient(cloud_, feature_vec, nearIndex_vecvec, squaredDistance_vecvec, variance_);
		//getPointCloud_featureGradient(cloud_, feature_vec, kdtree_, th_distance, nearIndex_vecvec,
		//	squaredDistance_vecvec, featureGradientVector_vec);
		for (int j = 0; j < cloud_->size(); j++)
		{
			float featureDivergence_ = 0.;
			int num_near_valid = 0;
			for (int i = 0; i < nearIndex_vecvec[j].size(); i++)
			{
				if (squaredDistance_vecvec[j][i] == 0.) continue;
				T_PointType point_query = cloud_->points[j];
				T_PointType point_match = cloud_->points[nearIndex_vecvec[j][i]];

				Eigen::Vector3d differenceVector_pos = Eigen::Vector3d::Identity();
				differenceVector_pos <<
					point_match.x - point_query.x,
					point_match.y - point_query.y,
					point_match.z - point_query.z;
				featureDivergence_ += (differenceVector_pos.transpose()
					* (featureGradientVector_vec[nearIndex_vecvec[j][i]] - featureGradientVector_vec[j]))(0, 0)
					* getDistanceWeight(squaredDistance_vecvec[j][i], variance_)
					* getDistanceWeight(squaredDistance_vecvec[j][i], variance_);
			}
			if (num_near_valid != 0) featureDivergence_ /= (float)num_near_valid;
			featureDivergence_vec.push_back(featureDivergence_);
		}
		if (!b_useGaussianFilter) return featureDivergence_vec;
		else return getPointCloud_featureGaussianFilter(cloud_, featureDivergence_vec, nearIndex_vecvec, squaredDistance_vecvec, variance_);
	}

	template <class T_PointType>
	static vector<float> getPointCloud_featureGaussianFilter(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, 
		const vector<float> &feature_vec, const vector<vector<int>> &nearIndex_vecvec, const vector<vector<float>> &squaredDistance_vecvec, float variance_)
	{
		vector<float> featureGaussianFilter_vec;
		for (int j = 0; j < cloud_->size(); j++)
		{
			float featureGaussianFilte = 0.;
			float sum_weight = 0.;
			for (int i = 0; i < nearIndex_vecvec[j].size(); i++)
			{
				featureGaussianFilte += feature_vec[nearIndex_vecvec[j][i]] 
					* getDistanceWeight(squaredDistance_vecvec[j][i], variance_);
				sum_weight += getDistanceWeight(squaredDistance_vecvec[j][i], variance_);
			}
			if (nearIndex_vecvec[j].size() != 0) featureGaussianFilte /= sum_weight;
			featureGaussianFilter_vec.push_back(featureGaussianFilte);
		}
		return featureGaussianFilter_vec;
	}

	template <class T_PointType>
	static boost::shared_ptr<pcl::PointCloud<T_PointType>> getPointCloud_ZAaxisByFeature(
		const boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, const vector<float> &feature_vec,
		float range_z_exist)
	{
		if (cloud_->size() != feature_vec.size())
		{
			cout << "ERROR: Point clous and features have a different size." << endl;
			throw std::runtime_error("ERROR: Point clous and features have a different size.");
		}

		float feature_min = std::numeric_limits<float>::max();
		float feature_max = -std::numeric_limits<float>::max();
		float feature_range;
		for (int j = 0; j < feature_vec.size(); j++)
		{
			if (feature_min > feature_vec[j]) feature_min = feature_vec[j];
			if (feature_max < feature_vec[j]) feature_max = feature_vec[j];
		}
		feature_range = feature_max - feature_min;
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_output(new pcl::PointCloud<T_PointType>);
		for (int j = 0; j < feature_vec.size(); j++)
		{
			float pos_z = (feature_vec[j] - feature_min) / feature_range * range_z_exist;
			T_PointType point_ = cloud_->points[j];
			point_.z = pos_z;
			cloud_output->push_back(point_);
		}
		return cloud_output;
	}

	template <class T_PointType>
	static void getPointCloud_removeFeatureOutlier(vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec, 
		vector<vector<float>> &feature_vecvec, float th_rate_BigAndSmall)
	{
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			if (cloud_vec[j]->size() != feature_vecvec[j].size())
			{
				cout << "ERROR: Point clous and features have a different size." << endl;
				throw std::runtime_error("ERROR: Point clous and features have a different size.");
			}
		}

		//calc th_low and th_high
		float th_low;
		float th_high;
		{
			vector<float> feature_vec_all;
			for (int j = 0; j < feature_vecvec.size(); j++)
				feature_vec_all.insert(feature_vec_all.end(), feature_vecvec[j].begin(), feature_vecvec[j].end());
			CTimeString::getOutlierRemovedIndex(feature_vec_all, th_rate_BigAndSmall, th_low, th_high);
			cout << "th_low:" << th_low << endl;
			cout << "th_high:" << th_high << endl;
		}

		//fix pointcloud and feature
		for (int j = 0; j < feature_vecvec.size(); j++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			vector<float> feature_vec;
			for (int i = 0; i < feature_vecvec[j].size(); i++)
			{
				if (feature_vecvec[j][i] < th_low) continue;
				if (th_high < feature_vecvec[j][i]) continue;
				cloud->push_back(cloud_vec[j]->points[i]);
				feature_vec.push_back(feature_vecvec[j][i]);

			}
			pcl::copyPointCloud(*cloud, *cloud_vec[j]);
			feature_vecvec[j] = feature_vec;
		}
	}

	template <typename T>
	static vector<pair<int,int>> determineCorrespondences_feature_histogram(const vector<T> &feature_vec_src, 
		const vector<T> &feature_vec_tgt, int num_bin, T feature_max, T feature_min, T feature_range, bool b_cout = false)
	{
		vector<vector<int>> index_hist_vecvec_src;
		index_hist_vecvec_src.resize(num_bin);
		for (int j = 0; j < feature_vec_src.size(); j++)
		{
			int i_bin = (int)((feature_vec_src[j] - feature_min) / feature_range);
			if (feature_vec_src[j] == feature_max) i_bin--;
			index_hist_vecvec_src[i_bin].push_back(j);
		}

		vector<vector<int>> index_hist_vecvec_tgt;
		index_hist_vecvec_tgt.resize(num_bin);
		for (int j = 0; j < feature_vec_tgt.size(); j++)
		{
			int i_bin = (int)((feature_vec_tgt[j] - feature_min) / feature_range);
			if (feature_vec_tgt[j] == feature_max) i_bin--;
			index_hist_vecvec_tgt[i_bin].push_back(j);
		}

		//calc pairs
		vector<pair<int, int>> corr_vec;	//src tgt
		for (int j = 0; j < index_hist_vecvec_src.size(); j++)
		{
			vector<vector<int>> temp_vecvec;
			temp_vecvec.push_back(index_hist_vecvec_src[j]);
			temp_vecvec.push_back(index_hist_vecvec_tgt[j]);
			vector<vector<int>> VectorPairPattern_temp_vecvec;
			VectorPairPattern_temp_vecvec = CTimeString::calcVectorPairPattern(temp_vecvec);
			for (int i = 0; i < VectorPairPattern_temp_vecvec.size(); i++)
				corr_vec.push_back(make_pair(VectorPairPattern_temp_vecvec[i][0], VectorPairPattern_temp_vecvec[i][1]));

			if (b_cout)
			{
				cout << "j:" << j << endl;
				cout << "VectorPairPattern_temp_vecvec.size():" << VectorPairPattern_temp_vecvec.size() << endl;
			}

		}

		return corr_vec;
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_output_kdtreeArg(
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
	static pcl::Correspondences determineCorrespondences_feature(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_tgt,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_src, boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt,
		int num_nearest)
	{
		pcl::Correspondences corr_src_tgt = determineCorrespondences_output_kdtreeArg(cloud_feature_src, kdtree_tgt, num_nearest);
		pcl::Correspondences corr_tgt_src = determineCorrespondences_output_kdtreeArg(cloud_feature_tgt, kdtree_src, num_nearest);
		pcl::Correspondences corr_new = getCorrespondences_eachPairHaving(corr_src_tgt, corr_tgt_src);
		return corr_new;
	}

	template <typename T>
	static pcl::Correspondences determineCorrespondences_feature(const vector<T> &features_src, const vector<T> &features_tgt, int num_nearest)
	{
		typedef pcl::PointXY T_PointType;
		pcl::PointCloud<T_PointType>::Ptr cloud_feature_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_feature_tgt(new pcl::PointCloud<T_PointType>());
		for (int j = 0; j < features_src.size(); j++)
		{
			T_PointType point_;
			point_.x = features_src[j];
			cloud_feature_src->push_back(point_);
		}
		cloud_feature_src->is_dense = true;
		for (int j = 0; j < features_tgt.size(); j++)
		{
			T_PointType point_;
			point_.x = features_tgt[j];
			cloud_feature_tgt->push_back(point_);
		}
		cloud_feature_tgt->is_dense = true;
		pcl::Correspondences corr_new;
		if (cloud_feature_src->size() == 0 || cloud_feature_tgt->size() == 0) return corr_new;
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_src(new pcl::KdTreeFLANN<T_PointType>);
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_tgt(new pcl::KdTreeFLANN<T_PointType>);
		kdtree_src->setInputCloud(cloud_feature_src);
		kdtree_tgt->setInputCloud(cloud_feature_tgt);
		corr_new = determineCorrespondences_feature(cloud_feature_src, cloud_feature_tgt, kdtree_src, kdtree_tgt, num_nearest);
		return corr_new;
	}

	template <typename T>
	static pcl::Correspondences determineCorrespondences_feature_remove(const vector<T> &features_src, const vector<T> &features_tgt, 
		const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, int num_nearest)
	{
		vector<T> features_src_removed;
		vector<T> features_tgt_removed;
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			features_src_removed.push_back(features_src[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			features_tgt_removed.push_back(features_tgt[index_unique_vec_tgt[j]]);
		pcl::Correspondences corrs_ = determineCorrespondences_feature(features_src_removed, features_tgt_removed, num_nearest);
		for (int j = 0; j < corrs_.size(); j++)
		{
			corrs_[j].index_query = index_unique_vec_src[corrs_[j].index_query];
			corrs_[j].index_match = index_unique_vec_tgt[corrs_[j].index_match];
		}
		return corrs_;
	}

	template <typename T>
	static vector<int> calcFeatureIndex_removingBiggestBin(const vector<T> &features_vec, const vector<int> &hist_vec_all,
		T value_max_hist, T value_min_hist)
	{
		int index_bin_biggest;
		int num_bin_biggest = 0;
		for (int j = 0; j < hist_vec_all.size(); j++)
		{
			if (num_bin_biggest < hist_vec_all[j])
			{
				num_bin_biggest = hist_vec_all[j];
				index_bin_biggest = j;
			}
		}

		vector<int> index_valid_vec;
		{
			vector<vector<int>> index_valid_vecvec_temp;
			vector<int> index_bin_vec;
			for (int j = 0; j < hist_vec_all.size(); j++)
			{
				if (index_bin_biggest == j) continue;
				index_bin_vec.push_back(j);
			}
			index_valid_vecvec_temp = CTimeString::getHistogram_IndexOfBin(features_vec, value_max_hist, value_min_hist, hist_vec_all.size(), index_bin_vec);
			for (int j = 0; j < index_valid_vecvec_temp.size(); j++)
				for (int i = 0; i < index_valid_vecvec_temp[j].size(); i++)
					index_valid_vec.push_back(index_valid_vecvec_temp[j][i]);
		}
		return index_valid_vec;
	}

	static vector<vector<int>> calcValidIndex_feature(const vector<vector<float>> &feature_vecvec, int num_bin_hist, bool b_showHistogram = false);

	template<typename Derived>
	static void getCorrespondance_RatioOfDistanceOfSrcAndTgt_perfectlyConnected(
		const Eigen::MatrixBase<Derived>& mat_fraction, const vector<int> cluster_input_vec,
		int num_size_cluster_min, float th_fraction, vector<vector<int>> &cluster_vecvec, vector<int> &cluster_rest)
	{
		vector<int> cluster_vec_temp = cluster_input_vec;
		vector<int> cluster_rest_temp;
		for (int j = 0; j < cluster_vec_temp.size(); j++)
		{
			for (int i = cluster_vec_temp.size() - 1; i >= j + 1; i--)
			{
				int j_small, i_big;
				if (cluster_vec_temp[j] < cluster_vec_temp[i])
				{
					i_big = cluster_vec_temp[i];
					j_small = cluster_vec_temp[j];
				}
				else
				{
					i_big = cluster_vec_temp[j];
					j_small = cluster_vec_temp[i];
				}

				if (mat_fraction(j_small, i_big) < th_fraction)
				{
					cluster_rest_temp.push_back(cluster_vec_temp[i]);
					cluster_vec_temp.erase(cluster_vec_temp.begin() + i);
				}
			}
		}

		if (cluster_vec_temp.size() < num_size_cluster_min)
		{
			cluster_rest.insert(cluster_rest.end(), cluster_vec_temp.begin(), cluster_vec_temp.end());
			return;
		}
		cluster_vecvec.push_back(cluster_vec_temp);
		if(cluster_vec_temp.size() >= num_size_cluster_min)
		
		{
			getCorrespondance_RatioOfDistanceOfSrcAndTgt_perfectlyConnected(mat_fraction, cluster_rest_temp, num_size_cluster_min, th_fraction,
				cluster_vecvec, cluster_rest);
		}
	}

	template <class T_PointType>
	static vector<pcl::Correspondences> getCorrespondance_RatioOfDistanceOfSrcAndTgt(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt, const pcl::Correspondences &corr_, float th_fraction, bool b_cout = false)
	{
		//RatioOfDistanceOfSrcAndTgt
		int num_corr_init = corr_.size();
		if (num_corr_init < 2)
		{
			cout << "ERROR(CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt): Few correspondednces inputed." << endl;
			throw std::runtime_error("ERROR(CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt): Few correspondednces inputed.");
		}
		//calc ratio
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mat_fraction(num_corr_init, num_corr_init);
		mat_fraction.setZero();
		vector<vector<int>> corr_pair_vecvec;
		
		for (int j = 0; j < num_corr_init; j++)
		{
			for (int i = j + 1; i < num_corr_init; i++)
			{
				T_PointType point_src_j = cloud_src->points[corr_[j].index_query];
				T_PointType point_tgt_j = cloud_tgt->points[corr_[j].index_match];
				T_PointType point_src_i = cloud_src->points[corr_[i].index_query];
				T_PointType point_tgt_i = cloud_tgt->points[corr_[i].index_match];
				float distance_src = sqrt(
					pow(point_src_i.x - point_src_j.x, 2.)
					+ pow(point_src_i.y - point_src_j.y, 2.)
					+ pow(point_src_i.z - point_src_j.z, 2.));
				float distance_tgt = sqrt(
					pow(point_tgt_i.x - point_tgt_j.x, 2.)
					+ pow(point_tgt_i.y - point_tgt_j.y, 2.)
					+ pow(point_tgt_i.z - point_tgt_j.z, 2.));
				float value_;
				if (distance_src == 0. || distance_tgt == 0.)
					value_ = 0.;
				else if(distance_src >= distance_tgt)
					value_ = distance_tgt/ distance_src;
				else/* if (distance_src < distance_tgt)*/
					value_ = distance_src / distance_tgt;
				if (value_ >= th_fraction)
				{
					vector<int> corr_pair_vec;
					corr_pair_vec.push_back(j);
					corr_pair_vec.push_back(i);
					corr_pair_vecvec.push_back(corr_pair_vec);
					mat_fraction(j, i) = value_;
				}
			}
		}

		//if (b_cout)
		//{
		//	//debug
		//	cout << "mat_fraction:" << endl;
		//	cout << mat_fraction << endl;
		//}

		if (corr_pair_vecvec.size() < 1)
		{
			cout << "ERROR(CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt): Few correspondednces exist simultaneously." << endl;
			throw std::runtime_error("ERROR(CKataokaPCL::getCorrespondance_RatioOfDistanceOfSrcAndTgt): Few correspondednces exist simultaneously.");
		}

		//if (b_cout)
		//{
		//	//debug
		//	for (int j = 0; j < corr_pair_vecvec.size(); j++)
		//	{
		//		for (int i = 0; i < corr_pair_vecvec[j].size(); i++)
		//			cout << corr_pair_vecvec[j][i] << " ";
		//		cout << endl;
		//	}
		//	cout << endl;
		//}

		vector<vector<int>> corr_pair_cluster_vecvec_new;
		//{
		//	vector<vector<int>> corr_pair_cluster_vecvec;	//temp cluster
		//	corr_pair_cluster_vecvec = CTimeString::getIntCluster_SomeToSome(corr_pair_vecvec);

		//	if (b_cout)
		//	{
		//		//debug
		//		for (int j = 0; j < corr_pair_cluster_vecvec.size(); j++)
		//		{
		//			for (int i = 0; i < corr_pair_cluster_vecvec[j].size(); i++)
		//				cout << corr_pair_cluster_vecvec[j][i] << " ";
		//			cout << endl;
		//		}
		//		cout << endl;
		//	}

		//	for (int j = 0; j < corr_pair_cluster_vecvec.size(); j++)
		//	{
		//		vector<vector<int>> corr_new_temp;
		//		vector<int> corr_rest_temp;	//not use
		//		getCorrespondance_RatioOfDistanceOfSrcAndTgt_perfectlyConnected(mat_fraction, corr_pair_cluster_vecvec[j], 3, th_fraction,
		//			corr_new_temp, corr_rest_temp);
		//		corr_pair_cluster_vecvec_new.insert(corr_pair_cluster_vecvec_new.end(), corr_new_temp.begin(), corr_new_temp.end());
		//	}

		//	//sort by size
		//	{
		//		vector<vector<int>> size_vecvec;
		//		for (int j = 0; j < corr_pair_cluster_vecvec_new.size(); j++)
		//		{
		//			vector<int> size_vec;
		//			size_vec.push_back(j);
		//			size_vec.push_back(corr_pair_cluster_vecvec_new[j].size());
		//			size_vecvec.push_back(size_vec);
		//		}
		//		CTimeString::sortVector2d(size_vecvec, 1, false);

		//		vector<vector<int>> corr_pair_cluster_vecvec_new_temp;
		//		for (int j = 0; j < size_vecvec.size(); j++)
		//			corr_pair_cluster_vecvec_new_temp.push_back(corr_pair_cluster_vecvec_new[size_vecvec[j][0]]);
		//		corr_pair_cluster_vecvec_new = corr_pair_cluster_vecvec_new_temp;
		//	}
		//}

		{
			vector<vector<bool>> b_matrix;
			for (int j = 0; j < mat_fraction.rows(); j++)
			{
				vector<bool> b_temp_vec;
				for (int i = 0; i < mat_fraction.cols(); i++)
				{
					if (mat_fraction(j, i) != 0.) b_temp_vec.push_back(true);
					else b_temp_vec.push_back(false);
				}
				b_matrix.push_back(b_temp_vec);
			}
			cout << "b_matrix.size():" << b_matrix.size() << endl;
			cout << "b_matrix[0].size():" << b_matrix[0].size() << endl;

			//if (b_cout)
			//{
			//	for (int j = 0; j < b_matrix.size(); j++)
			//	{
			//		for (int i = 0; i < b_matrix[j].size(); i++)
			//		{
			//			cout << b_matrix[j][i] << " ";
			//		}
			//		cout << endl;
			//	}
			//	cout << endl;
			//}
			
			corr_pair_cluster_vecvec_new = CTimeString::getIntCluster_boolMatrix(b_matrix, 6, 5, 2);

		}

		//if (b_cout)
		//{
		//	//debug
		//	for (int j = 0; j < corr_pair_cluster_vecvec_new.size(); j++)
		//	{
		//		for (int i = 0; i < corr_pair_cluster_vecvec_new[j].size(); i++)
		//			cout << corr_pair_cluster_vecvec_new[j][i] << " ";
		//		cout << endl;
		//	}
		//	cout << endl;
		//}

		vector<pcl::Correspondences> corrs_output_vec;
		for (int j = 0; j < corr_pair_cluster_vecvec_new.size(); j++)
		{
			pcl::Correspondences corr_output;
			for (int i = 0; i < corr_pair_cluster_vecvec_new[j].size(); i++)
				corr_output.push_back(corr_[corr_pair_cluster_vecvec_new[j][i]]);
			corrs_output_vec.push_back(corr_output);
		}

		//sort by size
		{
			vector<vector<int>> size_vecvec;
			for (int j = 0; j < corrs_output_vec.size(); j++)
			{
				vector<int> size_vec;
				size_vec.push_back(j);
				size_vec.push_back(corrs_output_vec[j].size());
				size_vecvec.push_back(size_vec);
			}
			CTimeString::sortVector2d(size_vecvec, 1, false);

			vector<pcl::Correspondences> corrs_vec_temp;
			for (int j = 0; j < size_vecvec.size(); j++)
				corrs_vec_temp.push_back(corrs_output_vec[size_vecvec[j][0]]);
			corrs_output_vec = corrs_vec_temp;
		}

		if (b_cout)
		{
			cout << "corrs_output_vec.size():" << corrs_output_vec.size() << endl;
			for (int j = 0; j < corrs_output_vec.size(); j++)
				cout << "j:" << j << " corrs_output_vec[j].size():" << corrs_output_vec[j].size() << endl;
		}

		return corrs_output_vec;
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_output_kdtreeArg_value(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt,float th_value, bool b_multipleNear = false)
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
	static pcl::Correspondences determineCorrespondences_feature_value(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_feature_tgt,
		boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_src, boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt, float th_value)
	{
		pcl::Correspondences corr_src_tgt = determineCorrespondences_output_kdtreeArg_value(cloud_feature_src, kdtree_tgt, th_value);
		pcl::Correspondences corr_tgt_src = determineCorrespondences_output_kdtreeArg_value(cloud_feature_tgt, kdtree_src, th_value);
		pcl::Correspondences corr_new = getCorrespondences_eachPairHaving(corr_src_tgt, corr_tgt_src);
		return corr_new;
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_output_kdtreeArg_value_singleQuery(
		T_PointType point_query, boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_tgt, float th_value, bool b_multipleNear = false)
	{
		pcl::Correspondences correspondences;
		std::vector<int> index(1);
		std::vector<float> distance(1);
		int found_neighs = kdtree_tgt->radiusSearch(point_query, th_value, index, distance);
		pcl::Correspondence corr;
		corr.index_query = 0;
		if (!b_multipleNear)
		{
			corr.index_match = index[0];
			corr.distance = distance[0];	//squared
			correspondences.push_back(corr);
		}
		else
		{
			for (int i = 0; i < index.size(); i++)
			{
				corr.index_match = index[i];
				corr.distance = distance[i];	//squared
				correspondences.push_back(corr);
			}
		}

		if (correspondences.size() == 0) cout << "ERROR: no corr found" << endl;

		return correspondences;
	}

	template <typename T>
	static pcl::Correspondences determineCorrespondences_feature_value(const vector<T> &features_src, const vector<T> &features_tgt,  float th_value)
	{
		typedef pcl::PointXY T_PointType;
		pcl::PointCloud<T_PointType>::Ptr cloud_feature_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_feature_tgt(new pcl::PointCloud<T_PointType>());
		for (int j = 0; j < features_src.size(); j++)
		{
			T_PointType point_;
			point_.x = features_src[j];
			cloud_feature_src->push_back(point_);
		}
		cloud_feature_src->is_dense = true;
		for (int j = 0; j < features_tgt.size(); j++)
		{
			T_PointType point_;
			point_.x = features_tgt[j];
			cloud_feature_tgt->push_back(point_);
		}
		cloud_feature_tgt->is_dense = true;
		pcl::Correspondences corr_new;
		if (cloud_feature_src->size() == 0 || cloud_feature_tgt->size() == 0) return corr_new;
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_src(new pcl::KdTreeFLANN<T_PointType>);
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_tgt(new pcl::KdTreeFLANN<T_PointType>);
		kdtree_src->setInputCloud(cloud_feature_src);
		kdtree_tgt->setInputCloud(cloud_feature_tgt);
		corr_new = determineCorrespondences_feature_value(cloud_feature_src, cloud_feature_tgt, kdtree_src, kdtree_tgt, th_value);
		return corr_new;
	}

	template <typename T>
	static pcl::Correspondences determineCorrespondences_feature_value_remove(const vector<T> &features_src, const vector<T> &features_tgt,
		const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, float th_value)
	{
		vector<T> features_src_removed;
		vector<T> features_tgt_removed;
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			features_src_removed.push_back(features_src[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			features_tgt_removed.push_back(features_tgt[index_unique_vec_tgt[j]]);
		pcl::Correspondences corrs_ = determineCorrespondences_feature_value(features_src_removed, features_tgt_removed, th_value);
		for (int j = 0; j < corrs_.size(); j++)
		{
			corrs_[j].index_query = index_unique_vec_src[corrs_[j].index_query];
			corrs_[j].index_match = index_unique_vec_tgt[corrs_[j].index_match];
		}
		return corrs_;
	}

	template <class T_PointType>
	static Eigen::Matrix<float, 3, Eigen::Dynamic> calcEigenMatrixFromPointCloud(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_)
	{
		int npts = cloud_->size();
		if (npts == 0)
		{
			cout << "ERROR(CKataokaPCL::calcEigenMatrixFromPointCloud): No point contained." << endl;
			throw std::runtime_error("ERROR(CKataokaPCL::calcEigenMatrixFromPointCloud): No point contained.");
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
			cout << "ERROR(CKataokaPCL::calcCovarianceMatrix): No point contained." << endl;
			throw std::runtime_error("ERROR(CKataokaPCL::calcCovarianceMatrix): No point contained.");
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

	template <typename T, class T_PointType>
	static vector<pair<float, float>> calcCompareValueOfFeature_scalar(const pcl::Correspondences &corr_,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,
		const vector<T> &features_src, const vector<T> &features_tgt,
		const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, float th_value)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_src_removed(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt_removed(new pcl::PointCloud<T_PointType>());
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			cloud_src_removed->push_back(cloud_src->points[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			cloud_tgt_removed->push_back(cloud_tgt->points[index_unique_vec_tgt[j]]);
		vector<T> features_src_removed;
		vector<T> features_tgt_removed;
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			features_src_removed.push_back(features_src[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			features_tgt_removed.push_back(features_tgt[index_unique_vec_tgt[j]]);
		typedef pcl::PointXY T_PointType_feature;
		pcl::PointCloud<T_PointType_feature>::Ptr cloud_feature_src(new pcl::PointCloud<T_PointType_feature>());
		pcl::PointCloud<T_PointType_feature>::Ptr cloud_feature_tgt(new pcl::PointCloud<T_PointType_feature>());
		for (int j = 0; j < features_src_removed.size(); j++)
		{
			T_PointType_feature point_;
			point_.x = features_src_removed[j];
			point_.y = 0.;
			cloud_feature_src->push_back(point_);
		}
		cloud_feature_src->is_dense = true;
		for (int j = 0; j < features_tgt_removed.size(); j++)
		{
			T_PointType_feature point_;
			point_.x = features_tgt_removed[j];
			point_.y = 0.;
			cloud_feature_tgt->push_back(point_);
		}
		cloud_feature_tgt->is_dense = true;
		pcl::KdTreeFLANN<T_PointType_feature>::Ptr kdtree_feature_src(new pcl::KdTreeFLANN<T_PointType_feature>);
		pcl::KdTreeFLANN<T_PointType_feature>::Ptr kdtree_feature_tgt(new pcl::KdTreeFLANN<T_PointType_feature>);
		kdtree_feature_src->setInputCloud(cloud_feature_src);
		kdtree_feature_tgt->setInputCloud(cloud_feature_tgt);
		Eigen::Matrix<float, 2, 1> mat_weight_src = getWeightPoint_2D(cloud_feature_src).cast<float>();
		Eigen::Matrix<float, 2, 1> mat_weight_tgt = getWeightPoint_2D(cloud_feature_tgt).cast<float>();
		vector<pair<float, float>> compare_srctgt_vec;
		for (int j = 0; j < corr_.size(); j++)
		{
			float compare_src;
			{
				pcl::Correspondences corr_near;
				T_PointType_feature point_feature;
				point_feature.x = features_src[corr_[j].index_query];
				point_feature.y = 0.;
				corr_near = determineCorrespondences_output_kdtreeArg_value_singleQuery(point_feature, kdtree_feature_src, th_value, true);
				pcl::PointCloud<T_PointType>::Ptr cloud_near(new pcl::PointCloud<T_PointType>());
				for (int i = 0; i < corr_near.size(); i++)
					cloud_near->push_back(cloud_src_removed->points[corr_near[i].index_match]);
				if (corr_near.size() == 0)
					compare_src = 10000.;
				else
				{
					compare_src = calcCovarianceMatrix(calcEigenMatrixFromPointCloud(cloud_near)).trace();
					if (compare_src > 10000.)
					{
						cout << "so big!!" << endl;
						cout << "j:" << j << endl;
					}
					else if (compare_src < 0.00001) compare_src = 0.;
				}

			}
			float compare_tgt;
			{
				pcl::Correspondences corr_near;
				T_PointType_feature point_feature;
				point_feature.x = features_tgt[corr_[j].index_match];
				point_feature.y = 0.;
				corr_near = determineCorrespondences_output_kdtreeArg_value_singleQuery(point_feature, kdtree_feature_tgt, th_value, true);
				pcl::PointCloud<T_PointType>::Ptr cloud_near(new pcl::PointCloud<T_PointType>());
				for (int i = 0; i < corr_near.size(); i++)
					cloud_near->push_back(cloud_tgt_removed->points[corr_near[i].index_match]);
				if (corr_near.size() == 0)
					compare_tgt = 10000.;
				else
				{
					compare_tgt = calcCovarianceMatrix(calcEigenMatrixFromPointCloud(cloud_near)).trace();
					if (compare_tgt > 10000.)
					{
						cout << "so big!!" << endl;
						cout << "j:" << j << endl;
					}
					else if (compare_tgt < 0.00001) compare_tgt = 0.;
				}
			}
			compare_srctgt_vec.push_back(make_pair(compare_src, compare_tgt));
		}

		return compare_srctgt_vec;
	}

	static void calcRanking_query_match_ValueOfFeature(const vector<vector<pair<float, float>>> &compare_vecvec,
		vector<int> &frame_vec, vector<int> &corr_index_vec, vector<bool> &b_queryOrNot_vec, vector<float> &evaluation_vec, bool b_cout = false);

	static vector<vector<int>> calcRanking_ValueOfFeature_argCompare(const vector<vector<pair<float, float>>> &compare_vecvec, bool b_cout = false);

	template <class T_PointType, typename T>
	static vector<vector<int>> calcRanking_ValueOfFeature_scalar(const vector<pair<int, int>> &index_pair_vec, const vector<pcl::Correspondences> &corrs_vec,
		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec, const vector<vector<T>> &feature_vecvec, vector<vector<int>> &index_valid_vecvec, float th_nearest, bool b_cout = false)
	{
		vector<vector<pair<float, float>>> compare_vecvec;//[index_frame_pair][index_pair] :variance
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			vector<pair<float, float>> compare_vec;
			compare_vec = calcCompareValueOfFeature_scalar(corrs_vec[j],
				cloud_vec[i_src], cloud_vec[i_tgt], feature_vecvec[i_src], feature_vecvec[i_tgt],
				index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], th_nearest);
			if (b_cout)
			{
				for (int j = 0; j < compare_vec.size(); j++)
					if (j % 10 == 0) cout << "j:" << j << "  query:" << compare_vec[j].first << " match:" << compare_vec[j].second << endl;
			}
			compare_vecvec.push_back(compare_vec);
		}
		vector<vector<int>> rank_output_vecvec;//[index_frame_pair][index_pair]
		rank_output_vecvec = calcRanking_ValueOfFeature_argCompare(compare_vecvec, b_cout);
		return rank_output_vecvec;
	}

	template <class T_PointType, typename T>
	static void determineCorrespondences_allFrames_feature_scalar_remove(const vector<vector<T>> &feature_vecvec, vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		const vector<pair<int, int>> &index_pair_vec, float th_nearest, float th_rank_rate, vector<vector<int>> index_valid_vecvec, vector<pcl::Correspondences> &corrs_vec, bool b_cout = false)
	{
		if (feature_vecvec.size() != cloud_vec.size())
		{
			cout << "ERROR: number of feature and one of pointcloud have different size." << endl;
			return;
		}

		corrs_vec.clear();//[index_frame_pair][index_pair]
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			pcl::Correspondences corrs_;
			//corrs_ = determineCorrespondences_feature_value(feature_vecvec[i_src], feature_vecvec[i_tgt], th_nearest);
			corrs_ = determineCorrespondences_feature_value_remove(feature_vecvec[i_src], feature_vecvec[i_tgt],
				index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], th_nearest);

			corrs_vec.push_back(corrs_);
		}

		{
			vector<vector<int>> rank_vecvec;//[index_frame_pair][index_pair]
			rank_vecvec = CKataokaPCL::calcRanking_ValueOfFeature_scalar(index_pair_vec, corrs_vec, cloud_vec, feature_vecvec, index_valid_vecvec, th_nearest, b_cout);
			vector<pcl::Correspondences> corrs_vec_temp;
			for (int j = 0; j < index_pair_vec.size(); j++)
			{
				pcl::Correspondences temp;
				corrs_vec_temp.push_back(temp);
			}

			//if(b_cout) cout << "rank_vecvec.size():" << rank_vecvec.size() << endl;
			//for (int j = 0; j < rank_vecvec.size(); j++)
			//{
			//	if(b_cout) cout << "rank_vecvec[j].size():" << rank_vecvec[j].size() << endl;
			//	pcl::Correspondences corrs_temp;
			//	for (int i = 0; i < rank_vecvec[j].size(); i++)
			//	{
			//		if ((float)rank_vecvec[j][i] <= th_rank) corrs_temp.push_back(corrs_vec[j][i]);
			//	}
			//	corrs_vec_temp.push_back(corrs_temp);
			//}

			//sort
			vector<vector<int>> sort_vecvec;
			for (int j = 0; j < rank_vecvec.size(); j++)
			{
				for (int i = 0; i < rank_vecvec[j].size(); i++)
				{
					vector<int> sort_vec;
					sort_vec.push_back(j);
					sort_vec.push_back(i);
					sort_vec.push_back(rank_vecvec[j][i]);
					sort_vecvec.push_back(sort_vec);
				}
			}
			CTimeString::sortVector2d(sort_vecvec, 2);

			for (int j = 0; j < (int)(sort_vecvec.size() * th_rank_rate); j++)
			{
				int index_frame_pair = sort_vecvec[j][0];
				int index_pair = sort_vecvec[j][1];
				corrs_vec_temp[index_frame_pair].push_back(corrs_vec[index_frame_pair][index_pair]);
			}
			corrs_vec = corrs_vec_temp;
		}

		if (b_cout)
		{
			cout << "show corr" << endl;
			cout << "corrs_vec.size():" << corrs_vec.size() << endl;
			for (int j = 0; j < corrs_vec.size(); j++)
			{
				cout << "j(index_frame_pair):" << j << " ";
				cout << "corrs_vec[j].size():" << corrs_vec[j].size() << endl;
			}
		}
	}

	template <class T_PointType, typename T>
	static void determineCorrespondences_allFrames_feature_scalar(const vector<vector<T>> &feature_vecvec, vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		const vector<pair<int, int>> &index_pair_vec, float th_nearest, float th_rank_rate, vector<pcl::Correspondences> &corrs_vec, bool b_cout = false)
	{
		if (feature_vecvec.size() != cloud_vec.size())
		{
			cout << "ERROR: number of feature and one of pointcloud have different size." << endl;
			return;
		}

		vector<vector<int>> index_valid_vecvec;//[index_frame][index]
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<int> index_valid_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				index_valid_vec.push_back(i);
			index_valid_vecvec.push_back(index_valid_vec);
		}
		determineCorrespondences_allFrames_feature_scalar_remove(feature_vecvec, cloud_vec, index_pair_vec, th_nearest, th_rank_rate, index_valid_vecvec, corrs_vec, b_cout);
	}

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_geometricConstraint(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,
		const vector<pcl::Correspondences> &corrs_vec, bool b_cout = false)
	{
		vector<float> evaluation_vec;
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src_moving(new pcl::PointCloud<T_PointType>);

		for (int j = 0; j < corrs_vec.size(); j++)
		{
			Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
			estimateRigidTransformation(cloud_src, cloud_tgt, corrs_vec[j], transformation_matrix);
			pcl::transformPointCloud(*cloud_src, *cloud_src_moving, CKataokaPCL::calcAffine3fFromHomogeneousMatrix(transformation_matrix));
			float distance_sum = 0.;
			for (int i = 0; i < corrs_vec[j].size(); i++)
			{
				T_PointType point_src = cloud_src_moving->points[corrs_vec[j][i].index_query];
				T_PointType point_tgt = cloud_tgt->points[corrs_vec[j][i].index_match];
				distance_sum += sqrt(
					pow(point_src.x - point_tgt.x)
					+ pow(point_src.y - point_tgt.y)
					+ pow(point_src.z - point_tgt.z)
				);
			}
			distance_sum /= (float)corrs_vec[j].size();
			evaluation_vec.push_back(distance_sum);
		}
		
		//sort
		vector<vector<float>> evaluation_vecvec;
		for (int j = 0; j < evaluation_vec.size(); j++)
		{
			vector<float> temp_vec;
			temp_vec.push_back((float)j);
			temp_vec.push_back(evaluation_vec[j]);
			evaluation_vecvec.push_back(temp_vec);
		}
		CTimeString::sortVector2d(evaluation_vecvec, 1);

		return corrs_vec[evaluation_vecvec[0][0]];
	}

};
