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

	static Eigen::Vector6d calcRobotPosition_3DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
		Eigen::Vector6d pose_sensor, Eigen::Vector6d disp_registration);
	static Eigen::Vector6d calcRobotPosition_6DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
		Eigen::Vector6d pose_sensor, Eigen::Vector6d disp_registration);

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
		pcl::Correspondences correspondences;
		{
			correspondences.resize(cloud_src->size());
			std::vector<int> index(1);
			std::vector<float> distance(1);
			unsigned int nr_valid_correspondences = 0;
			pcl::KdTreeFLANN<T_PointType> match_search;
			match_search.setInputCloud(cloud_tgt->makeShared());
			for (size_t i = 0; i < cloud_src->size(); ++i)
			{
				int found_neighs = match_search.nearestKSearch(cloud_src->at(i), 1, index, distance);
				pcl::Correspondence corr;
				corr.index_query = i;
				corr.index_match = index[0];
				corr.distance = distance[0];	//squared
				correspondences[nr_valid_correspondences++] = corr;
			}
			correspondences.resize(nr_valid_correspondences);
			//cout << "correspondences size = " << nr_valid_correspondences << endl;
		}
		//call median function
		return getCorrMedianDistance(correspondences);
	}

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
		if(useBeforeVGF)
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
		{}
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
			fpfh = CKataokaPCL::computeFPFH<T_PointType>(cloud_decreasing, cloud_surface, radius_normal_vec[i_radius], radius_FPFH);

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
				fpfh = CKataokaPCL::computeFPFH<T_PointType>(cloud_decreasing, cloud_surface,
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
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt,
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

			b_hasConverged = CKataokaPCL::align_SAC_AI<T_PointType>(transform_, inlier_, fitnessscore,
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
		if(output_vec.size() != 0) b_hasConverged = true;
		return b_hasConverged;
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
};
