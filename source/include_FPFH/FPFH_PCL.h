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

#include <pcl/filters/statistical_outlier_removal.h>

#include"TimeString.h"

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

using namespace std;

class __declspec(dllexport) CFPFH_PCL
{
public:

	template <class T_PointType>
	static pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_surface, float radius_normal, float radius_FPFH)
	{
		const auto view_point = T_PointType(0.0, 10.0, 10.0);
		const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
		const pcl::search::KdTree<T_PointType>::Ptr kdtree(new pcl::search::KdTree<T_PointType>);
		ne->setInputCloud(cloud_surface);
		ne->setRadiusSearch(radius_normal);
		ne->setSearchMethod(kdtree);
		ne->setViewPoint(view_point.x, view_point.y, view_point.z);
		ne->compute(*normals);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh = computeFPFH(cloud_, cloud_surface, normals, radius_FPFH);
		return fpfh;
	}

	template <class T_PointType>
	static pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_surface, 
		boost::shared_ptr<pcl::PointCloud<pcl::Normal>> normals_, float radius_FPFH)
	{
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_(new pcl::search::KdTree<T_PointType>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		const pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>);
		fpfhe->setInputCloud(cloud_);
		fpfhe->setSearchSurface(cloud_surface);
		fpfhe->setInputNormals(normals_);
		fpfhe->setSearchMethod(kdtree_);
		fpfhe->setRadiusSearch(radius_FPFH);
		fpfhe->compute(*fpfh);
		return fpfh;
	}

	template <class T_PointType>
	static bool align_SAC_AI(
		Eigen::Matrix4d &transformation_result, vector<int> &Inlier_, float &FitnessScore,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt,
		float voxel_size, float MaxCorrespondenceDistance, float SimilarityThreshold,
		float InlierFraction, int MaximumIterations, int NumberOfSamples, int CorrespondenceRandomness)
	{
		//VGF
		//cout << "cloud_src.size():" << cloud_src.size() << endl;
		//cout << "cloud_tgt.size():" << cloud_tgt.size() << endl;
		const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->setInputCloud(cloud_src);
		sor->filter(*cloud_src);
		//cout << "cloud_src.size():" << cloud_src.size() << endl;
		sor->setInputCloud(cloud_tgt);
		sor->filter(*cloud_tgt);
		//cout << "cloud_tgt.size():" << cloud_tgt.size() << endl;

		//align
		boost::shared_ptr<pcl::PointCloud<T_PointType>> temp_(new pcl::PointCloud<T_PointType>);
		pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33> align;
		align.setInputSource(cloud_src);
		align.setSourceFeatures(fpfh_src);
		align.setInputTarget(cloud_tgt);
		align.setTargetFeatures(fpfh_tgt);
		align.setMaximumIterations(MaximumIterations);
		align.setNumberOfSamples(NumberOfSamples);
		align.setCorrespondenceRandomness(CorrespondenceRandomness);
		align.setSimilarityThreshold(SimilarityThreshold);				//th of corr rejecter
		align.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);	//related to th of computing fitness score
		align.setInlierFraction(InlierFraction);						//th of inlier number
		//align->setMinSampleDistance(min_sample_distance_);	//function not found
		align.align(*temp_);

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
		Eigen::Matrix4d &transformation_result, vector<int> &Inlier_, float &FitnessScore, int &frame_failed,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt,
		float voxel_size, float MaxCorrespondenceDistance, float SimilarityThreshold,
		float InlierFraction, int MaximumIterations, int NumberOfSamples, int CorrespondenceRandomness, int max_RANSAC, bool b_cout)
	{
		//cout << "RANSAC" << endl;
		//int max_RANSAC = 50;
		int index_RANSAC = 0;
		//int frame_failed = 0;
		frame_failed = 0;
		vector<pair<float, Eigen::Matrix4d>> output_vec;
		vector<vector<int>> inlier_vec;
		vector<float> fitnessscore_vec;

		while (1)
		{
			Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
			bool b_hasConverged = false;
			vector<int> inlier_;
			float fitnessscore;

			b_hasConverged = align_SAC_AI<T_PointType>(transform_, inlier_, fitnessscore,
				cloud_src, fpfh_src, cloud_tgt, fpfh_tgt,
				voxel_size, MaxCorrespondenceDistance, SimilarityThreshold,
				InlierFraction, MaximumIterations, NumberOfSamples, CorrespondenceRandomness);

			cout << "index_RANSAC:" << index_RANSAC << endl;
			if (b_cout)
			{
				cout << "b_hasConverged:" << b_hasConverged << endl;
				cout << "fitnessscore:" << fitnessscore << endl;
				cout << "inlier_.size():" << inlier_.size() << endl;
				cout << endl;
			}
			if (b_hasConverged)
			{
				output_vec.push_back(make_pair((float)inlier_.size() / (float)fpfh_src->size(), transform_));
				inlier_vec.push_back(inlier_);
				fitnessscore_vec.push_back(fitnessscore);
			}
			else
				frame_failed++;

			index_RANSAC++;
			if (index_RANSAC >= max_RANSAC) break;

		}

		//for (int i = 0; i < output_vec.size(); i++)
		//{
		//	cout << "i:" << i << " score:" << output_vec[i].first << endl;
		//	cout << output_vec[i].second << endl;
		//	cout << endl;
		//}

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
		{
			transformation_result = Eigen::Matrix4d::Identity();
			Inlier_.clear();
			FitnessScore = 1000.;
		}

		cout << "align finished" << endl;

		bool b_hasConverged = false;
		if (output_vec.size() != 0) b_hasConverged = true;
		return b_hasConverged;
	}

	template <class T_PointType>
	static bool align_SAC_AI_RANSAC_TRUE(
		Eigen::Matrix4d &transformation_result, vector<int> &Inlier_, float &FitnessScore, int &frame_failed,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt,
		float voxel_size, float MaxCorrespondenceDistance, float SimilarityThreshold,
		float InlierFraction, int MaximumIterations, int NumberOfSamples, int CorrespondenceRandomness, int max_iteration, bool b_cout,
		Eigen::Matrix4d transformation_true, float th_distance)
	{
		int index_iteration = 0;
		frame_failed = 0;
		vector<pair<float, Eigen::Matrix4d>> output_vec;
		vector<vector<int>> inlier_vec;
		vector<float> fitnessscore_vec;

		while (1)
		{
			Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
			bool b_hasConverged = false;
			vector<int> inlier_;
			float fitnessscore;

			b_hasConverged = align_SAC_AI<T_PointType>(transform_, inlier_, fitnessscore,
				cloud_src, fpfh_src, cloud_tgt, fpfh_tgt,
				voxel_size, MaxCorrespondenceDistance, SimilarityThreshold,
				InlierFraction, MaximumIterations, NumberOfSamples, CorrespondenceRandomness);

			cout << "index_iteration:" << index_iteration << endl;
			if (b_cout)
			{
				cout << "b_hasConverged:" << b_hasConverged << endl;
				cout << "fitnessscore:" << fitnessscore << endl;
				cout << "inlier_.size():" << inlier_.size() << endl;
				cout << endl;
			}

			//estimation
			bool b_estimation = false;
			double distance_ = 0.;
			if (b_hasConverged)
			{
				boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src_est(new pcl::PointCloud<T_PointType>());
				boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src_estTRUE(new pcl::PointCloud<T_PointType>());

				pcl::copyPointCloud(*cloud_src, *cloud_src_est);
				pcl::copyPointCloud(*cloud_src, *cloud_src_estTRUE);
				{
					Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
					Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(transform_);
					pcl::transformPointCloud(*cloud_src_est, *cloud_src_est, Trans_temp);
				}
				{
					Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
					Trans_temp = CKataokaPCL::calcAffine3fFromHomogeneousMatrix(transformation_true);
					pcl::transformPointCloud(*cloud_src_estTRUE, *cloud_src_estTRUE, Trans_temp);
				}
				//distance to true
				for (size_t i = 0; i < cloud_src_est->size(); i++)
				{
					T_PointType point_, point_true;
					point_ = cloud_src_est->points[i];
					point_true = cloud_src_estTRUE->points[i];
					const float sqrt_before =
						pow(point_.x - point_true.x, 2.)
						+ pow(point_.y - point_true.y, 2.)
						+ pow(point_.z - point_true.z, 2.);
					distance_ += static_cast<double>(sqrt(
						pow(point_.x - point_true.x, 2.)
						+ pow(point_.y - point_true.y, 2.)
						+ pow(point_.z - point_true.z, 2.)));

				}
				if (cloud_src_est->size() != 0) distance_ /= cloud_src_est->size();
				else distance_ = 100.;

				if (distance_ < th_distance) b_estimation = true;
			}

			if (b_estimation)
			{
				//output_vec.push_back(make_pair((float)inlier_.size() / (float)fpfh_src->size(), transform_));
				output_vec.push_back(make_pair(distance_, transform_));
				inlier_vec.push_back(inlier_);
				fitnessscore_vec.push_back(fitnessscore);
			}
			else
				frame_failed++;

			index_iteration++;
			if (index_iteration >= max_iteration) break;

		}

		//for (int i = 0; i < output_vec.size(); i++)
		//{
		//	cout << "i:" << i << " score:" << output_vec[i].first << endl;
		//	cout << output_vec[i].second << endl;
		//	cout << endl;
		//}

		//select most good value
		float score_min = th_distance;
		int i_best;
		for (int i = 0; i < output_vec.size(); i++)
		{
			if (output_vec[i].first < score_min)
			{
				score_min = output_vec[i].first;
				i_best = i;
			}
		}

		cout << "Show Result" << endl;
		cout << "frame_failed:" << frame_failed << "(/" << max_iteration << ")" << endl;
		if (output_vec.size() != 0)
		{
			cout << "converged final transformation" << endl;
			cout << "i:" << i_best << " score:" << output_vec[i_best].first << endl;
			cout << output_vec[i_best].second << endl;
			transformation_result = output_vec[i_best].second;
			Inlier_ = inlier_vec[i_best];
			FitnessScore = fitnessscore_vec[i_best];
		}
		else
		{
			transformation_result = Eigen::Matrix4d::Identity();
			Inlier_.clear();
			FitnessScore = 1000.;
		}

		cout << "align finished" << endl;

		bool b_hasConverged = false;
		if (output_vec.size() != 0) b_hasConverged = true;
		return b_hasConverged;
		return false;
	}

	template <class T_PointType>
	static vector<float> getErrorOfFPFHSource(float &median_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt,
		float MaxCorrespondenceDistance)
	{
		//corr
		pcl::Correspondences correspondences;
		{
			correspondences.resize(cloud_src->size());
			std::vector<int> index(1);
			std::vector<float> distance(1);
			unsigned int nr_valid_correspondences = 0;
			pcl::KdTreeFLANN<T_PointType> match_search;
			match_search.setInputCloud(cloud_tgt);
			for (size_t i = 0; i < cloud_src->size(); ++i)
			{
				int found_neighs = match_search.nearestKSearch(cloud_src->at(i), 1, index, distance);
				if (distance[0] > MaxCorrespondenceDistance * MaxCorrespondenceDistance) continue;
				pcl::Correspondence corr;
				corr.index_query = i;
				corr.index_match = index[0];
				corr.distance = distance[0];	//squared
				correspondences[nr_valid_correspondences++] = corr;
			}
			correspondences.resize(nr_valid_correspondences);
			//cout << "correspondences size = " << nr_valid_correspondences << endl;
		}

		//return CKataokaPCL::getErrorOfFPFHSource_corr(median_arg, correspondences, fpfh_src, fpfh_tgt);
		return getErrorOfFPFHSource_corr(median_arg, correspondences, fpfh_src, fpfh_tgt);
	}

	static vector<float> getErrorOfFPFHSource_corr(float &median_arg, pcl::Correspondences correspondences,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt);
	static vector<float> getFPFHVariance(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_);

	static pcl::Correspondences CFPFH_PCL::getNearestOfFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		int num_near, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh);

	template <class T_PointType>
	static void getFPFHMeanAndSigma(vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec, float radius_FPFH, vector<float> &mean_fpfh, vector<float> &sigma_fpfh)
	{
		vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH));
		getFPFHMeanAndSigma(fpfh_vec, mean_fpfh, sigma_fpfh);
	}

	static void getFPFHMeanAndSigma(vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec,
		vector<float> &mean_fpfh, vector<float> &sigma_fpfh);

	static vector<bool> CFPFH_PCL::getFPFH_unique(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_feature,
		vector<float> mean_fpfh_vec, vector<float> sigma_fpfh_vec, float beta_fpfh);

	template <class T_PointType>
	static vector<vector<int>> getFPFH_unique_someRadius(vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec, vector<float> radius_FPFH_vec, 
		vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &fpfh_vec_output, bool b_cout = false)
	{
		fpfh_vec_output.clear();

		vector<vector<bool>> b_unique_vecvec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<bool> b_unique_vec;
			b_unique_vec.resize(cloud_vec[j]->size());
			fill(b_unique_vec.begin(), b_unique_vec.end(), true);
			b_unique_vecvec.push_back(b_unique_vec);
		}

		for (int j = 0; j < radius_FPFH_vec.size(); j++)
		{
			if(b_cout) cout << "calc:  radius_FPFH_vec[j]:" << radius_FPFH_vec[j] << endl;

			vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
			for (int i = 0; i < cloud_vec.size(); i++)
				fpfh_vec.push_back(computeFPFH(cloud_vec[i], cloud_vec[i], normals_vec[i], radius_FPFH_vec[j]));

			if (j == 1)	//center radius
			{
				for (int i = 0; i < fpfh_vec.size(); i++)
					fpfh_vec_output.push_back(fpfh_vec[i]);
			}

			vector<float> mean_fpfh;
			vector<float> sigma_fpfh;
			getFPFHMeanAndSigma(fpfh_vec, mean_fpfh, sigma_fpfh);
			for (int i = 0; i < mean_fpfh.size(); i++)
				if (b_cout) cout << "i:" << i << "  mean:" << mean_fpfh[i] << " sigma:" << sigma_fpfh[i] << endl;

			float beta_fpfh = 2.;
			for (int i = 0; i < fpfh_vec.size(); i++)
			{
				vector<bool> b_unique_vec;
				b_unique_vec = getFPFH_unique(fpfh_vec[i], mean_fpfh, sigma_fpfh, beta_fpfh);
				int num_unique = 0;
				for (int k = 0; k < b_unique_vec.size(); k++)
					if (b_unique_vec[k]) num_unique++;
				if (b_cout) cout << "i:" << i << " num_unique:" << num_unique << endl;

				for (int k = 0; k < b_unique_vec.size(); k++)
					b_unique_vecvec[i][k] = b_unique_vecvec[i][k] * b_unique_vec[k];
			}
			if (b_cout) cout << endl;
		}

		if (b_cout)
		{
			cout << "show result" << endl;
			for (int j = 0; j < b_unique_vecvec.size(); j++)
			{
				int num_unique = 0;
				for (int i = 0; i < b_unique_vecvec[j].size(); i++)
					if (b_unique_vecvec[j][i]) num_unique++;
				cout << "j:" << j << " num_unique:" << num_unique << endl;
			}
		}

		vector<vector<int>> index_vecvec;
		for (int j = 0; j < b_unique_vecvec.size(); j++)
		{
			vector<int> index_vec;
			for (int i = 0; i < b_unique_vecvec[j].size(); i++)
				if (b_unique_vecvec[j][i]) index_vec.push_back(i);
			index_vecvec.push_back(index_vec);
		}

		return index_vecvec;
	}

	template <class T_PointType>
	static vector<vector<int>> getFPFH_unique_someRadius(vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec, float radius_FPFH_center, 
		vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &fpfh_vec_output, bool b_cout = false)
	{
		vector<float> radius_FPFH_vec;
		radius_FPFH_vec.push_back(radius_FPFH_center * 0.75);
		radius_FPFH_vec.push_back(radius_FPFH_center);
		radius_FPFH_vec.push_back(radius_FPFH_center * 1.25);
		vector<vector<int>> index_vecvec;
		index_vecvec = getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_vec, fpfh_vec_output, b_cout);
		return index_vecvec;
	}

	static pcl::Correspondences getCorrespondences_eachPairHaving(const pcl::Correspondences &corr_src_tgt,
		const pcl::Correspondences &corr_tgt_src);

	static pcl::Correspondences getNearestOfFPFH_eachPairHaving(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src,
		pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt, int num_near);

	static pcl::Correspondences getNearestOfFPFH_eachPairHaving_remove(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, int num_near);

};
