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

	template <class T_PointType>
	static Eigen::Vector3d getWeightPoint(pcl::PointCloud<T_PointType> cloud_)
	{
		Eigen::Vector3d sum_vec = Eigen::Vector3d::Zero();
		Eigen::Vector3d weight_vec = Eigen::Vector3d::Zero();
		if (cloud_.size() == 0)
		{
			cout << "ERROR: no point found" << endl;
			return weight_vec;
		}
		for (size_t i = 0; i < cloud_.size(); i++)
		{
			sum_vec(0, 0) = sum_vec(0, 0) + cloud_.points[i].x;
			sum_vec(1, 0) = sum_vec(1, 0) + cloud_.points[i].y;
			sum_vec(2, 0) = sum_vec(2, 0) + cloud_.points[i].z;
		}
		weight_vec(0, 0) = sum_vec(0, 0) / cloud_.size();
		weight_vec(1, 0) = sum_vec(1, 0) / cloud_.size();
		weight_vec(2, 0) = sum_vec(2, 0) / cloud_.size();
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
			CTimeString::getOuolierRemovedIndex(feature_vec_all, th_rate_BigAndSmall, th_low, th_high);
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

	//template <class T_PointType>
	//static vector<float> getPointCloud_featureDivergence(
	//	boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, const vector<float> &feature_vec,
	//	boost::shared_ptr<pcl::KdTreeFLANN<T_PointType>> kdtree_, const float th_distance, float variance_, bool b_useGaussianFilter = false)


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
	static pcl::Correspondences determineCorrespondences_feature(vector<T> features_src, vector<T> features_tgt, int num_nearest)
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
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_src(new pcl::KdTreeFLANN<T_PointType>);
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_tgt(new pcl::KdTreeFLANN<T_PointType>);
		kdtree_src->setInputCloud(cloud_feature_src);
		kdtree_tgt->setInputCloud(cloud_feature_tgt);
		pcl::Correspondences corr_new = determineCorrespondences_feature(cloud_feature_src, cloud_feature_tgt, kdtree_src, kdtree_tgt, num_nearest);
		return corr_new;
	}

};
