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

//#include"TimeString.h"

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

		bool useBeforeVGF = false;
		useBeforeVGF = true;

		const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
		const pcl::search::KdTree<T_PointType>::Ptr kdtree(new pcl::search::KdTree<T_PointType>);
		//ne->setInputCloud(cloud_.makeShared());
		if (useBeforeVGF)
			ne->setInputCloud(cloud_surface);
		else
			ne->setInputCloud(cloud_);
		ne->setRadiusSearch(radius_normal);
		ne->setSearchMethod(kdtree);
		ne->setViewPoint(view_point.x, view_point.y, view_point.z);
		ne->compute(*normals);

		//const float radius_FPFH = voxel_size * 5.0;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		const pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>);
		fpfhe->setInputCloud(cloud_);
		if (useBeforeVGF)
			fpfhe->setSearchSurface(cloud_surface);
		else
		{
		}
		fpfhe->setInputNormals(normals);
		fpfhe->setSearchMethod(kdtree);
		//fpfhe->setKSearch(10);
		fpfhe->setRadiusSearch(radius_FPFH);
		fpfhe->compute(*fpfh);
		//cout << "fpfhe->getSearchParameter():" << fpfhe->getSearchParameter() << endl;

		//{
		//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh2(new pcl::PointCloud<pcl::FPFHSignature33>);
		//	pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33> fpfhe2;
		//	cout << "fpfhe2.getKSearch():" << fpfhe2.getKSearch() << endl;
		//	cout << "fpfhe2.getSearchParameter():" << fpfhe2.getSearchParameter() << endl;
		//	fpfhe2.setInputCloud(cloud_.makeShared());
		//	if (useBeforeVGF)
		//		fpfhe2.setSearchSurface(cloud_surface.makeShared());
		//	else
		//	{
		//	}
		//	fpfhe2.setInputNormals(normals);
		//	fpfhe2.setSearchMethod(kdtree);
		//	fpfhe2.setRadiusSearch(radius_FPFH);
		//	fpfhe2.compute(*fpfh2);
		//	cout << "fpfhe.getKSearch():" << fpfhe2.getKSearch() << endl;
		//	cout << "fpfhe.getSearchParameter():" << fpfhe2.getSearchParameter() << endl;
		//	//cout << "fpfhe2.hist_f1_.size():" << fpfhe2.hist_f1_.size() << endl;
		//	//cout << "fpfhe2.hist_f2_.size():" << fpfhe2.hist_f2_.size() << endl;
		//	//cout << "fpfhe2.hist_f3_.size():" << fpfhe2.hist_f3_.size() << endl;
		//	cout << "fpfhe2.k_:" << fpfhe2.k_ << endl;
		//	//cout << "fpfhe2.nr_bins_f1_:" << fpfhe2.nr_bins_f1_ << endl;
		//	//cout << "fpfhe2.nr_bins_f2_:" << fpfhe2.nr_bins_f2_ << endl;
		//	//cout << "fpfhe2.nr_bins_f3_:" << fpfhe2.nr_bins_f3_ << endl;
		//	//fpfhe2.setKSearch(0.5);
		//	//fpfhe2.setSearchParameter(0.5);

		//}

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
	static pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH_radius(vector<int> &index_vec,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_surface, vector<float> radius_normal_vec, float radius_FPFH)
	{
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_decreasing(new pcl::PointCloud<T_PointType>);
		pcl::copyPointCloud(*cloud_, *cloud_decreasing);
		vector<pair<bool, vector<bool>>>b_pickup_colrow_pair;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);

		for (int i_radius = 0; i_radius < radius_normal_vec.size(); i_radius++)
		{
			cout << "i_radius:" << i_radius << " " << radius_normal_vec[i_radius] << endl;
			cout << "cloud_decreasing.size():" << cloud_decreasing->size() << endl;
			fpfh->clear();
			fpfh = computeFPFH<T_PointType>(cloud_decreasing, cloud_surface, radius_normal_vec[i_radius], radius_FPFH);

			//array to vectorvector
			int num_nan = 0;
			vector<vector<float>> histogram_colrow;
			//cout << "sizeof(fpfh.points[0].histogram):" << sizeof(fpfh.points[0].histogram) << endl;
			for (int j = 0; j < fpfh->size(); j++)
			{
				vector<float> histogram_row;
				for (int i = 0; i < sizeof(fpfh->points[j].histogram); i++)
				{
					if (isnan(fpfh->points[j].histogram[i]) || fpfh->points[j].histogram[i] > 100. || fpfh->points[j].histogram[i] < 0)
					{
						histogram_row.push_back(0.);
						num_nan++;
					}
					else
						histogram_row.push_back(fpfh->points[j].histogram[i]);

				}
				histogram_colrow.push_back(histogram_row);
			}
			cout << "num_nan:" << num_nan << endl;

			//cout << "fpfh" << endl;
			//for (int i = 0; i < histogram_colrow[0].size(); i++)
			//	cout << "i:" << i << " " << histogram_colrow[0][i] << endl;


			if (i_radius == 0)
			{
				for (int j = 0; j < histogram_colrow.size(); j++)
				{
					vector<bool> b_pickup_row;
					for (int i = 0; i < histogram_colrow[j].size(); i++)
						b_pickup_row.push_back(true);
					b_pickup_colrow_pair.push_back(make_pair(true, b_pickup_row));
				}
			}

			//mean
			vector<float> histogram_mean_row;
			{
				vector<float> histogram_sum_row;
				histogram_sum_row.clear();
				histogram_sum_row.resize(histogram_colrow[0].size());
				fill(histogram_sum_row.begin(), histogram_sum_row.end(), 0.);
				for (int j = 0; j < histogram_colrow.size(); j++)
				{
					for (int i = 0; i < histogram_colrow[j].size(); i++)
						histogram_sum_row[i] += histogram_colrow[j][i];
				}
				for (int i = 0; i < histogram_sum_row.size(); i++)
					histogram_mean_row.push_back(histogram_sum_row[i] / ((float)histogram_colrow.size()));
				//cout << "mean" << endl;
				//for (int i = 0; i < histogram_mean_row.size(); i++)
				//	cout << "i:" << i << " " << histogram_mean_row[i] << endl;
			}

			//pickup
			{
				vector<vector<bool>> b_pickup_colrow_new;
				int j_fpfh = 0;
				for (int j_pick = 0; j_pick < b_pickup_colrow_pair.size(); j_pick++)
				{
					bool b_usePoint = false;
					b_usePoint = b_pickup_colrow_pair[j_pick].first;
					if (b_usePoint)
					{
						vector<bool> b_pickup_row_new;
						for (int i = 0; i < b_pickup_colrow_pair[j_pick].second.size(); i++)
						{
							if (histogram_colrow[j_fpfh][i] > histogram_mean_row[i])
								b_pickup_row_new.push_back(true);
							else
								b_pickup_row_new.push_back(false);
						}
						b_pickup_colrow_new.push_back(b_pickup_row_new);
						j_fpfh++;
					}
					else
					{
						vector<bool> b_pickup_row_new;
						fill(b_pickup_row_new.begin(), b_pickup_row_new.end(), false);
						b_pickup_colrow_new.push_back(b_pickup_row_new);
					}
				}
				for (int j = 0; j < b_pickup_colrow_pair.size(); j++)
				{
					for (int i = 0; i < b_pickup_colrow_pair[j].second.size(); i++)
						b_pickup_colrow_pair[j].second[i] = b_pickup_colrow_pair[j].second[i] && b_pickup_colrow_new[j][i];
				}
				//update bool of pair.first
				for (int j = 0; j < b_pickup_colrow_pair.size(); j++)
				{
					b_pickup_colrow_pair[j].first = false;
					for (int i = 0; i < b_pickup_colrow_pair[j].second.size(); i++)
					{
						if (b_pickup_colrow_pair[j].second[i])
						{
							b_pickup_colrow_pair[j].first = true;
							break;
						}
					}
				}

			}
			//cloud_decreasing
			cloud_decreasing->clear();
			for (int j = 0; j < b_pickup_colrow_pair.size(); j++)
				if (b_pickup_colrow_pair[j].first) cloud_decreasing->push_back(cloud_->points[j]);

			//output fpfh
			if (i_radius == radius_normal_vec.size() - 1)
			{
				fpfh->clear();
				fpfh = computeFPFH<T_PointType>(cloud_decreasing, cloud_surface,
					radius_normal_vec[radius_normal_vec.size() - 1], radius_FPFH);
			}

		}

		//output index selected by FPFH with all radius
		index_vec.clear();
		for (int j = 0; j < b_pickup_colrow_pair.size(); j++)
			if (b_pickup_colrow_pair[j].first) index_vec.push_back(j);

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
};
