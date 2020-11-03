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
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec, vector<float> radius_FPFH_vec, bool b_cout = false)
	{
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
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec, float radius_FPFH_center, bool b_cout = false)
	{
		vector<float> radius_FPFH_vec;
		radius_FPFH_vec.push_back(radius_FPFH_center * 0.75);
		radius_FPFH_vec.push_back(radius_FPFH_center);
		radius_FPFH_vec.push_back(radius_FPFH_center * 1.25);
		vector<vector<int>> index_vecvec;
		index_vecvec = getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_vec, b_cout);
		return index_vecvec;
	}

	template <class T_PointType>
	static vector<vector<int>> getFPFH_unique_someRadius_outputFile(vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec, float radius_FPFH_center, 
		string dir_, vector<string> filenames_vec, bool b_cout = false)
	{
		vector<float> radius_FPFH_vec;
		radius_FPFH_vec.push_back(radius_FPFH_center * 0.75);
		radius_FPFH_vec.push_back(radius_FPFH_center);
		radius_FPFH_vec.push_back(radius_FPFH_center * 1.25);
		vector<vector<int>> index_vecvec;
		index_vecvec = getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_vec, b_cout);

		//output
		vector<vector<string>> s_output_vecvec;
		//header
		{
			vector<string> s_temp;
			s_temp.push_back("file_name");
			s_temp.push_back("index_valid");
			s_output_vecvec.push_back(s_temp);
		}
		string s_radius;
		for (int i = 0; i < radius_FPFH_vec.size(); i++)
			s_radius += CTimeString::to_string_remove0(radius_FPFH_vec[i]) + " ";
		string s_filename = "validPoint_FPFH_" + s_radius + ".csv";
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<string> s_temp;
			s_temp.push_back(filenames_vec[j]);
			for (int i = 0; i < index_vecvec[j].size(); i++)
				s_temp.push_back(to_string(index_vecvec[j][i]));
			s_output_vecvec.push_back(s_temp);
		}

		CTimeString::getCSVFromVecVec(s_output_vecvec, dir_ + "/" + s_filename);

		return index_vecvec;
	}

	static vector<vector<int>> getFPFH_unique_someRadius_inputFile(string dir_, vector<string> filenames_cloud_vec, bool b_cout = false);

	static pcl::Correspondences getCorrespondences_eachPairHaving(const pcl::Correspondences &corr_src_tgt,
		const pcl::Correspondences &corr_tgt_src);

	static pcl::Correspondences determineCorrespondences_featureFpfh_eachPairHaving_num(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src,
		pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt, int num_near);

	static pcl::Correspondences determineCorrespondences_featureFpfh_eachPairHaving_num_remove(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, int num_near);

	static pcl::Correspondences determineCorrespondences_featureFpfh(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		int num_near_max, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh, float th_value);

	static pcl::Correspondences determineCorrespondences_featureFpfh_eachPairHaving(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src,
		pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt, int num_near_max, float th_value);

	static pcl::Correspondences determineCorrespondences_featureFpfh_eachPairHaving_remove(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, const vector<int> &index_unique_vec_src,
		const vector<int> &index_unique_vec_tgt, int num_near_max, float th_value);

	static pcl::Correspondences determineCorrespondences_featureFpfh_kdtreeArg_singleQuery(pcl::FPFHSignature33 point_query,
		const boost::shared_ptr<pcl::KdTreeFLANN<pcl::FPFHSignature33>> kdtree_tgt, float th_nearest_fpfh, int th_nearest_num, bool b_multipleNear = false);

	template <class T_PointType>
	static vector<pair<float, float>> calcRanking_compare_featureFPFH(const pcl::Correspondences &corr_,
		const boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, const boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,
		const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh_src, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh_tgt,
		const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, float th_nearest_fpfh, int th_nearest_num)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_src_removed(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt_removed(new pcl::PointCloud<T_PointType>());
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			cloud_src_removed->push_back(cloud_src->points[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			cloud_tgt_removed->push_back(cloud_tgt->points[index_unique_vec_tgt[j]]);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src_removed(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt_removed(new pcl::PointCloud<pcl::FPFHSignature33>);
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			fpfh_src_removed->push_back(fpfh_src->points[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			fpfh_tgt_removed->push_back(fpfh_tgt->points[index_unique_vec_tgt[j]]);
		pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
		pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
		kdtree_fpfh_src->setInputCloud(fpfh_src_removed);
		kdtree_fpfh_tgt->setInputCloud(fpfh_tgt_removed);
		vector<pair<float, float>> compare_srctgt_vec;
		for (int j = 0; j < corr_.size(); j++)
		{
			float compare_src;
			{
				pcl::Correspondences corr_near;
				pcl::FPFHSignature33 point_fpfh;
				point_fpfh = fpfh_src->points[corr_[j].index_query];
				corr_near = determineCorrespondences_featureFpfh_kdtreeArg_singleQuery(point_fpfh, kdtree_fpfh_src, th_nearest_fpfh, th_nearest_num, true);
				pcl::PointCloud<T_PointType>::Ptr cloud_near(new pcl::PointCloud<T_PointType>());
				for (int i = 0; i < corr_near.size(); i++)
					cloud_near->push_back(cloud_src_removed->points[corr_near[i].index_match]);
				if (corr_near.size() == 0)
					compare_src = 10000.;
				else
				{
					compare_src = CKataokaPCL::calcCovarianceMatrix(CKataokaPCL::calcEigenMatrixFromPointCloud(cloud_near)).trace();
					if (compare_src > 10000.)
					{
						cout << "so big!!" << endl;
						cout << "j:" << j << endl;
					}
				}

			}
			float compare_tgt;
			{
				pcl::Correspondences corr_near;
				pcl::FPFHSignature33 point_fpfh;
				//point_fpfh = fpfh_tgt->points[corr_[j].index_query];
				point_fpfh = fpfh_tgt->points[corr_[j].index_match];
				corr_near = determineCorrespondences_featureFpfh_kdtreeArg_singleQuery(point_fpfh, kdtree_fpfh_tgt, th_nearest_fpfh, th_nearest_num, true);
				pcl::PointCloud<T_PointType>::Ptr cloud_near(new pcl::PointCloud<T_PointType>());
				for (int i = 0; i < corr_near.size(); i++)
					cloud_near->push_back(cloud_tgt_removed->points[corr_near[i].index_match]);
				if (corr_near.size() == 0)
					compare_tgt = 10000.;
				else
				{
					compare_tgt = CKataokaPCL::calcCovarianceMatrix(CKataokaPCL::calcEigenMatrixFromPointCloud(cloud_near)).trace();
					if (compare_tgt > 10000.)
					{
						cout << "so big!!" << endl;
						cout << "j:" << j << endl;
					}
				}

			}
			compare_srctgt_vec.push_back(make_pair(compare_src, compare_tgt));
		}
		return compare_srctgt_vec;
	}

	template <class T_PointType>
	static vector<vector<int>> calcRanking_featureFPFH(const vector<pair<int, int>> &index_pair_vec, const vector<pcl::Correspondences> &corrs_vec,
		const vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec, const vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &fpfh_vec, 
		const vector<vector<int>> &index_valid_vecvec, float th_nearest_fpfh, int th_nearest_num, bool b_cout = false)
	{
		cout << "calcRanking_featureFPFH" << endl;
		vector<vector<pair<float, float>>> compare_vecvec;//[index_frame_pair][index_pair] :variance
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			vector<pair<float, float>> compare_vec;
			compare_vec = calcRanking_compare_featureFPFH(corrs_vec[j], cloud_vec[i_src], cloud_vec[i_tgt],
				fpfh_vec[i_src], fpfh_vec[i_tgt], index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], 
				th_nearest_fpfh, th_nearest_num);
			if (b_cout)
			{
				for (int j = 0; j < compare_vec.size(); j++)
					if (j % 10 == 0) cout << "j:" << j << "  query:" << compare_vec[j].first << " match:" << compare_vec[j].second << endl;
			}
			compare_vecvec.push_back(compare_vec);
		}
		vector<vector<int>> rank_output_vecvec;//[index_frame_pair][index_pair]
		rank_output_vecvec = CKataokaPCL::calcRanking_compareArg(compare_vecvec, b_cout);
		return rank_output_vecvec;
	}

	template <class T_PointType>
	static void determineCorrespondences_allFramesRanking_featureFpfh_remove(const vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &fpfh_vec,
		const vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec, const vector<pair<int, int>> &index_pair_vec, 
		float th_nearest_fpfh, int th_nearest_num, float th_rank_rate, const vector<vector<int>> index_valid_vecvec, vector<pcl::Correspondences> &corrs_vec_arg, vector<vector<float>> &evaluation_vecvec, bool b_cout = false)
	{
		cout << "determineCorrespondences_allFramesRanking_featureFpfh_remove" << endl;
		if (fpfh_vec.size() != cloud_vec.size())
		{
			cout << "ERROR: number of feature and one of pointcloud have different size." << endl;
			return;
		}

		cout << "  calc corr" << endl;
		corrs_vec_arg.clear();//[index_frame_pair][index_pair]
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			cout << "  i_tgt:" << i_tgt << endl;
			cout << "  i_src:" << i_src << endl;
			pcl::Correspondences corrs_;
			corrs_ = determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
				index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], th_nearest_num, th_nearest_fpfh);
			corrs_vec_arg.push_back(corrs_);
		}
		cout << "  calc ranking" << endl;
		{
			vector<vector<int>> rank_vecvec;//[index_frame_pair][index_pair]
			rank_vecvec = calcRanking_featureFPFH(index_pair_vec, corrs_vec_arg, cloud_vec, fpfh_vec, index_valid_vecvec, th_nearest_fpfh, th_nearest_num, b_cout);
			vector<pcl::Correspondences> corrs_vec_temp;
			for (int j = 0; j < index_pair_vec.size(); j++)
			{
				pcl::Correspondences temp;
				corrs_vec_temp.push_back(temp);
			}
			cout << "  calc ranking sort" << endl;
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
			cout << "  calc ranking sort fin" << endl;

			//evaluation_vecvec
			for (int j = 0; j < index_pair_vec.size(); j++)
			{
				vector<float> temp_vec;
				evaluation_vecvec.push_back(temp_vec);
			}
			for (int j = 0; j < (int)(sort_vecvec.size() * th_rank_rate); j++)
			{
				int index_frame_pair = sort_vecvec[j][0];
				int index_pair = sort_vecvec[j][1];
				corrs_vec_temp[index_frame_pair].push_back(corrs_vec_arg[index_frame_pair][index_pair]);
				evaluation_vecvec[index_frame_pair].push_back((float)sort_vecvec[j][2]);
			}
			corrs_vec_arg = corrs_vec_temp;
		}

		if (b_cout)
		{
			cout << "show corr" << endl;
			cout << "corrs_vec.size():" << corrs_vec_arg.size() << endl;
			for (int j = 0; j < corrs_vec_arg.size(); j++)
			{
				cout << "j(index_frame_pair):" << j << " ";
				cout << "corrs_vec[j].size():" << corrs_vec_arg[j].size() << endl;
			}
		}
	}

	template <class T_PointType>
	static void determineCorrespondences_allFramesRanking_featureFpfh_remove(const vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &fpfh_vec,
		const vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec, const vector<pair<int, int>> &index_pair_vec,
		float th_nearest_fpfh, int th_nearest_num, float th_rank_rate, const vector<vector<int>> index_valid_vecvec, vector<pcl::Correspondences> &corrs_vec_arg, bool b_cout = false)
	{
		vector<vector<float>> evaluation_vecvec;
		determineCorrespondences_allFramesRanking_featureFpfh_remove(fpfh_vec, cloud_vec, index_pair_vec, th_nearest_fpfh, th_nearest_num, th_rank_rate, index_valid_vecvec, corrs_vec_arg, evaluation_vecvec, b_cout);
	}

};
