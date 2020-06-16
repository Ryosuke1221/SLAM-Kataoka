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
#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>


#include"KataokaCorrespondence.h"
#include"KataokaConvergence.h"

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

	double M_proposed_penalty_chara, M_proposed_dist_search, M_proposed_weight_dist_chara;

public:
	void setCharaVector_src(vector<int> chara_vec);

	void setCharaVector_tgt(vector<int> chara_vec);

	void setCharaParameter(double penalty_chara, double dist_search, double weight_dist_chara) 
	{
		M_proposed_penalty_chara = penalty_chara;
		M_proposed_dist_search = dist_search;
		M_proposed_weight_dist_chara = weight_dist_chara;
	}

	void determineCorrespondences_chara(pcl::Correspondences &correspondences,
		double penalty_chara, double dist_search, double weight_dist_chara);

	double getFitnessScore_chara();

	double getDistanceOf2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud1_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud2_arg);

	void determineCorrespondences_argPC(pcl::Correspondences &correspondences, double max_distance,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_tgt);

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
		pcl::PointCloud<T_PointType> cloud_src, pcl::PointCloud<T_PointType> cloud_tgt)
	{
		//corr
		pcl::Correspondences correspondences;
		{
			correspondences.resize(cloud_src.size());
			std::vector<int> index(1);
			std::vector<float> distance(1);
			unsigned int nr_valid_correspondences = 0;
			pcl::KdTreeFLANN<T_PointType> match_search;
			match_search.setInputCloud(cloud_tgt.makeShared());
			for (size_t i = 0; i < cloud_src.size(); ++i)
			{
				int found_neighs = match_search.nearestKSearch(cloud_src.at(i), 1, index, distance);
				pcl::Correspondence corr;
				corr.index_query = i;
				corr.index_match = index[0];
				corr.distance = distance[0];
				correspondences[nr_valid_correspondences++] = corr;
			}
			correspondences.resize(nr_valid_correspondences);
			cout << "correspondences size = " << nr_valid_correspondences << endl;
		}
		//call median function
		return getCorrMedianDistance(correspondences);
	}

	template <class T_PointType>
	static pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
		pcl::PointCloud<T_PointType> cloud_, float voxel_size, float radius_normal, float radius_FPFH)
	{
		const auto view_point = T_PointType(0.0, 10.0, 10.0);

		bool useBeforeVGF = false;
		useBeforeVGF = true;

		pcl::PointCloud<T_PointType> cloud_VGF;
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_.makeShared());
		sor->filter(cloud_VGF);

		const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
		const pcl::search::KdTree<T_PointType>::Ptr kdtree(new pcl::search::KdTree<T_PointType>);
		//ne->setInputCloud(cloud_.makeShared());
		if(useBeforeVGF)
			ne->setInputCloud(cloud_.makeShared());
		else
			ne->setInputCloud(cloud_VGF.makeShared());
		ne->setRadiusSearch(radius_normal);
		ne->setSearchMethod(kdtree);
		ne->setViewPoint(view_point.x, view_point.y, view_point.z);
		ne->compute(*normals);

		//const float radius_FPFH = voxel_size * 5.0;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		const pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>);
		fpfhe->setInputCloud(cloud_VGF.makeShared());
		if (useBeforeVGF)
			fpfhe->setSearchSurface(cloud_.makeShared());
		else
		{}
		fpfhe->setInputNormals(normals);
		fpfhe->setSearchMethod(kdtree);
		fpfhe->setRadiusSearch(radius_FPFH);
		fpfhe->compute(*fpfh);

		//cout << "fpfh->size:" << fpfh->size() << endl;
		//vector<vector<double>> hist_vecvec;
		//for (int j = 0; j < fpfh->size(); j++)
		//{
		//	//if (j % 1000 != 0) continue;
		//	//cout << "j:" << j << " fpfh->points[j].descriptorSize:" << (int)fpfh->points[j].descriptorSize << endl;
		//	//cout << "j:" << j << " fpfh->points[j].descriptorSize:" << fpfh->points[j].descriptorSize << endl;

		//	int size_array = sizeof(fpfh->points[j].histogram);
		//	vector<double> hist_vec;
		//	for (int i = 0; i < size_array; i++)
		//	{
		//		hist_vec.push_back(fpfh->points[j].histogram[i]);
		//	}
		//	hist_vecvec.push_back(hist_vec);
		//}
		//cout << "hist_vecvec[0].size()" << hist_vecvec[0].size() << endl;
		//string filename = "../../data/" + CTimeString::getTimeString() + "hist.csv";

		return fpfh;
	}

	template <class T_PointType>
	static Eigen::Matrix4d align_FPFH_SAC_AI(pcl::PointCloud<T_PointType> cloud_src, pcl::PointCloud<T_PointType> cloud_tgt)
	{
		cout << "preprocess" << endl;

		//pcl::PointCloud<T_PointType>::Ptr src_(new pcl::PointCloud<T_PointType>());
		//pcl::PointCloud<T_PointType>::Ptr tgt_(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType> src_;
		pcl::PointCloud<T_PointType> tgt_;
		pcl::copyPointCloud(cloud_src, src_);
		pcl::copyPointCloud(cloud_tgt, tgt_);

		float voxel_size;
		//voxel_size = 0.01;
		//voxel_size = 0.05;
		voxel_size = 0.1;

		float radius_normal_FPFH, radius_FPFH;
		radius_normal_FPFH = voxel_size * 2.0;
		radius_FPFH = voxel_size * 5.0;

		float MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
		MaxCorrespondenceDistance_SAC = voxel_size * 2.5;
		int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;
		//MaximumIterations_SAC = 500000;
		//MaximumIterations_SAC = 50;	//8 & 8
		//MaximumIterations_SAC = 1000;
		MaximumIterations_SAC = 500;
		//NumberOfSamples_SAC = 4;//8 & 8
		//NumberOfSamples_SAC = 10;
		NumberOfSamples_SAC = 100;
		//CorrespondenceRandomness_SAC = 2;
		CorrespondenceRandomness_SAC = 10;
		//SimilarityThreshold_SAC = 0.9f;
		SimilarityThreshold_SAC = 0.01f;
		//InlierFraction_SAC = 0.25f;
		InlierFraction_SAC = 0.15f;
		cout << "fill InlierFraction_SAC ->";
		cin >> InlierFraction_SAC;


		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(src_.makeShared());
		sor->filter(src_);
		cout << "cloud_src.size():" << cloud_src.size() << endl;
		cout << "src_->size():" << src_.size() << endl;
		sor->setInputCloud(tgt_.makeShared());
		sor->filter(tgt_);
		cout << "cloud_tgt.size():" << cloud_tgt.size() << endl;
		cout << "tgt_->size():" << tgt_.size() << endl;

		//compute fpfh
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh_src = computeFPFH(cloud_src, voxel_size, radius_normal_FPFH, radius_FPFH);
		fpfh_tgt = computeFPFH(cloud_tgt, voxel_size, radius_normal_FPFH, radius_FPFH);
		//void* align;
		pcl::PointCloud<T_PointType> temp_;
		Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();

		cout << "RANSAC" << endl;
		int max_RANSAC = 50;
		int index_RANSAC = 0;
		int frame_failed = 0;
		//float th_RANSAC = 0.5f;
		vector<pair<float, Eigen::Matrix4d>> output_vec;
		while (1)
		{
			pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33> align;
			align.setInputSource(src_.makeShared());
			align.setSourceFeatures(fpfh_src);
			align.setInputTarget(tgt_.makeShared());
			align.setTargetFeatures(fpfh_tgt);
			align.setMaximumIterations(MaximumIterations_SAC);
			align.setNumberOfSamples(NumberOfSamples_SAC);
			align.setCorrespondenceRandomness(CorrespondenceRandomness_SAC);
			align.setSimilarityThreshold(SimilarityThreshold_SAC);				//th of corr rejecter
			align.setMaxCorrespondenceDistance(MaxCorrespondenceDistance_SAC);	//related to th of computing fitness score
			align.setInlierFraction(InlierFraction_SAC);						//th of inlier number
			//align->setMinSampleDistance(min_sample_distance_);	//function not found
			align.align(temp_);
			cout << "i:" << index_RANSAC << endl;
			cout << "align.getFitnessScore():" << align.getFitnessScore() << endl;
			cout << "align.hasConverged():" << align.hasConverged() << endl;
			cout << "align.getInliers().size():" << align.getInliers().size() << endl;
			transform_ = align.getFinalTransformation().cast<double>();

			if (align.hasConverged() == 1)
				output_vec.push_back(make_pair((float)align.getInliers().size() / (float)src_.size(), transform_));
			else 
				frame_failed++;

			index_RANSAC++;
			if (index_RANSAC >= max_RANSAC) break;
		}

		for (int i = 0; i < output_vec.size(); i++)
		{
			cout << "i:" << i << " score:" << output_vec[i].first << endl;
			cout << output_vec[i].second << endl;
			cout << endl;
		}

		//select most good value
		float score_min = 100.;
		int i_RANSAC;
		for (int i = 0; i < output_vec.size(); i++)
		{
			if (output_vec[i].first < score_min)
			{
				score_min = output_vec[i].first;
				i_RANSAC = i;
			}
		}

		cout << "Show Result" << endl;
		cout << "frame_failed:" << frame_failed << "(/" << max_RANSAC << ")" << endl;
		if (output_vec.size() != 0)
		{
			transform_ = output_vec[i_RANSAC].second;
			cout << "converged final transformation" << endl;
			cout << "i:" << i_RANSAC << " score:" << output_vec[i_RANSAC].first << endl;
			cout << transform_ << endl;
		}
		else 
			transform_ = Eigen::Matrix4d::Identity();


		cout << "align finished" << endl;

		return transform_;
	}

	template <class T_PointType>
	static bool align_SAC_AI(
		Eigen::Matrix4d &transformation_result, vector<int> &Inlier_, float &FitnessScore,
		pcl::PointCloud<T_PointType> cloud_src, pcl::PointCloud<pcl::FPFHSignature33> fpfh_src,
		pcl::PointCloud<T_PointType> cloud_tgt,	pcl::PointCloud<pcl::FPFHSignature33> fpfh_tgt,
		float voxel_size, float MaxCorrespondenceDistance, float SimilarityThreshold,
		float InlierFraction, int MaximumIterations, int NumberOfSamples, int CorrespondenceRandomness)
	{
		//VGF
		cout << "cloud_src.size():" << cloud_src.size() << endl;
		cout << "cloud_tgt.size():" << cloud_tgt.size() << endl;
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_src.makeShared());
		sor->filter(cloud_src);
		cout << "cloud_src.size():" << cloud_src.size() << endl;
		sor->setInputCloud(cloud_tgt.makeShared());
		sor->filter(cloud_tgt);
		cout << "cloud_tgt.size():" << cloud_tgt.size() << endl;

		//align
		pcl::PointCloud<T_PointType> temp_;
		pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33> align;
		align.setInputSource(cloud_src.makeShared());
		align.setSourceFeatures(fpfh_src.makeShared());
		align.setInputTarget(cloud_tgt.makeShared());
		align.setTargetFeatures(fpfh_tgt.makeShared());
		align.setMaximumIterations(MaximumIterations);
		align.setNumberOfSamples(NumberOfSamples);
		align.setCorrespondenceRandomness(CorrespondenceRandomness);
		align.setSimilarityThreshold(SimilarityThreshold);				//th of corr rejecter
		align.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);	//related to th of computing fitness score
		align.setInlierFraction(InlierFraction);						//th of inlier number
		//align->setMinSampleDistance(min_sample_distance_);	//function not found
		align.align(temp_);

		//result
		transformation_result = align.getFinalTransformation().cast<double>();
		Inlier_ = align.getInliers();
		FitnessScore = align.getFitnessScore();

		bool b_hasConverged = false;
		if (1 == align.hasConverged()) b_hasConverged = true;
		return b_hasConverged;
	}

	template <class T_PointType>
	static bool align_SAC_AI_RANSAC(
		Eigen::Matrix4d &transformation_result, vector<int> &Inlier_, float &FitnessScore,
		pcl::PointCloud<T_PointType> cloud_src, pcl::PointCloud<pcl::FPFHSignature33> fpfh_src,
		pcl::PointCloud<T_PointType> cloud_tgt, pcl::PointCloud<pcl::FPFHSignature33> fpfh_tgt,
		float voxel_size, float MaxCorrespondenceDistance, float SimilarityThreshold,
		float InlierFraction, int MaximumIterations, int NumberOfSamples, int CorrespondenceRandomness)
	{
		cout << "RANSAC" << endl;
		int max_RANSAC = 50;
		int index_RANSAC = 0;
		int frame_failed = 0;
		vector<pair<float, Eigen::Matrix4d>> output_vec;
		vector<vector<int>> inlier_vec;
		vector<float> fitnessscore_vec;

		while (1)
		{
			Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
			bool b_hasConverged = false;
			vector<int> inlier_;
			float fitnessscore;

			b_hasConverged = CKataokaPCL::align_SAC_AI<T_PointType>(transform_, inlier_, fitnessscore,
				cloud_src, fpfh_src, cloud_tgt, fpfh_tgt,
				voxel_size, MaxCorrespondenceDistance, SimilarityThreshold,
				InlierFraction, MaximumIterations, NumberOfSamples, CorrespondenceRandomness);

			cout << "index_RANSAC:" << index_RANSAC << endl;
			cout << "b_hasConverged:" << b_hasConverged << endl;
			cout << "fitnessscore:" << fitnessscore << endl;
			cout << "inlier_.size():" << inlier_.size() << endl;

			if (b_hasConverged)
			{
				output_vec.push_back(make_pair((float)inlier_.size() / (float)fpfh_src.size(), transform_));
				inlier_vec.push_back(inlier_);
				fitnessscore_vec.push_back(fitnessscore);
			}
			else
				frame_failed++;

			index_RANSAC++;
			if (index_RANSAC >= max_RANSAC) break;

		}

		for (int i = 0; i < output_vec.size(); i++)
		{
			cout << "i:" << i << " score:" << output_vec[i].first << endl;
			cout << output_vec[i].second << endl;
			cout << endl;
		}

		//select most good value
		float score_min = 100.;
		int i_RANSAC;
		for (int i = 0; i < output_vec.size(); i++)
		{
			if (output_vec[i].first < score_min)
			{
				score_min = output_vec[i].first;
				i_RANSAC = i;
			}
		}

		cout << "Show Result" << endl;
		cout << "frame_failed:" << frame_failed << "(/" << max_RANSAC << ")" << endl;
		if (output_vec.size() != 0)
		{
			cout << "converged final transformation" << endl;
			cout << "i:" << i_RANSAC << " score:" << output_vec[i_RANSAC].first << endl;
			cout << output_vec[i_RANSAC].second << endl;
			transformation_result = output_vec[i_RANSAC].second;
			Inlier_ = inlier_vec[i_RANSAC];
			FitnessScore = fitnessscore_vec[i_RANSAC];
		}
		else
			transformation_result = Eigen::Matrix4d::Identity();

		cout << "align finished" << endl;

		bool b_hasConverged = false;
		if(output_vec.size() != 0) b_hasConverged = true;
		return b_hasConverged;
	}

};
