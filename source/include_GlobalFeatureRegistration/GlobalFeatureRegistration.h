#pragma once

#include "ExtendableICP.h"
#include "FPFH_PCL.h"

class CGlobalFeatureRegistration : public CPointcloudBasic
{

public:
	CGlobalFeatureRegistration()
	{

	}

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

	static vector<bool> getFPFH_unique(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_feature,
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
			if (b_cout) cout << "calc:  radius_FPFH_vec[j]:" << radius_FPFH_vec[j] << endl;

			vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
			for (int i = 0; i < cloud_vec.size(); i++)
				fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[i], cloud_vec[i], normals_vec[i], radius_FPFH_vec[j]));

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
					compare_src = CPointcloudBasic::calcCovarianceMatrix(CPointcloudBasic::calcEigenMatrixFromPointCloud(cloud_near)).trace();
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
					compare_tgt = CPointcloudBasic::calcCovarianceMatrix(CPointcloudBasic::calcEigenMatrixFromPointCloud(cloud_near)).trace();
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
		cout << "    calcRanking_featureFPFH" << endl;
		vector<vector<pair<float, float>>> compare_vecvec;//[index_frame_pair][index_pair] :variance
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			cout << "    i_tgt:" << i_tgt;
			cout << ", i_src:" << i_src << endl;
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
		//rank_output_vecvec = calcRanking_compareArg(compare_vecvec, b_cout);
		rank_output_vecvec = calcRanking_compareArg_multipleEachCovariance(compare_vecvec, b_cout);
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
			cout << "fpfh_vec.size():" << fpfh_vec.size() << endl;
			cout << "cloud_vec.size():" << cloud_vec.size() << endl;

			cout << "ERROR: number of feature and one of pointcloud have different size." << endl;
			return;
		}

		cout << "  calc corr" << endl;
		corrs_vec_arg.clear();//[index_frame_pair][index_pair]
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			cout << "  i_tgt:" << i_tgt;
			cout << ", i_src:" << i_src << endl;
			pcl::Correspondences corrs_;
			corrs_ = determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
				index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], th_nearest_num, th_nearest_fpfh);
			cout << "corrs_.size():" << corrs_.size() << endl;
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
	static vector<pair<int, int>> determineCorrespondences_featureScalar_histogram(const vector<T> &feature_vec_src,
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

	template <typename T>
	static pcl::Correspondences determineCorrespondences_featureScalar_num(const vector<T> &features_src, const vector<T> &features_tgt, int num_nearest)
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
		corr_new = CPointcloudBasic::determineCorrespondences_kdtreeArg_eachPairHaving_num(cloud_feature_src, cloud_feature_tgt, kdtree_src, kdtree_tgt, num_nearest);
		return corr_new;
	}

	template <typename T>
	static pcl::Correspondences determineCorrespondences_featureScalar_num_remove(const vector<T> &features_src, const vector<T> &features_tgt,
		const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, int num_nearest)
	{
		vector<T> features_src_removed;
		vector<T> features_tgt_removed;
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			features_src_removed.push_back(features_src[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			features_tgt_removed.push_back(features_tgt[index_unique_vec_tgt[j]]);
		pcl::Correspondences corrs_ = determineCorrespondences_featureScalar_num(features_src_removed, features_tgt_removed, num_nearest);
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

	template <class T_PointType>
	static pcl::Correspondences determineCorrespondences_featureScalar_kdtreeArg_singleQuery(
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
	static pcl::Correspondences determineCorrespondences_featureScalar(const vector<T> &features_src, const vector<T> &features_tgt, float th_value)
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
		corr_new = CPointcloudBasic::determineCorrespondences_kdtreeArg_eachPairHaving(cloud_feature_src, cloud_feature_tgt, kdtree_src, kdtree_tgt, th_value);
		return corr_new;
	}

	template <typename T>
	static pcl::Correspondences determineCorrespondences_featureScalar_remove(const vector<T> &features_src, const vector<T> &features_tgt,
		const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, float th_value)
	{
		pcl::Correspondences corrs_;
		if (features_src.size() == 0 || features_tgt.size() == 0) return corrs_;
		vector<T> features_src_removed;
		vector<T> features_tgt_removed;
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			features_src_removed.push_back(features_src[index_unique_vec_src[j]]);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			features_tgt_removed.push_back(features_tgt[index_unique_vec_tgt[j]]);
		corrs_ = determineCorrespondences_featureScalar(features_src_removed, features_tgt_removed, th_value);
		for (int j = 0; j < corrs_.size(); j++)
		{
			corrs_[j].index_query = index_unique_vec_src[corrs_[j].index_query];
			corrs_[j].index_match = index_unique_vec_tgt[corrs_[j].index_match];
		}
		return corrs_;
	}

	template <typename T, class T_PointType>
	static vector<pair<float, float>> calcRanking_compare_featureScalar(const pcl::Correspondences &corr_,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,
		const vector<T> &features_src, const vector<T> &features_tgt,
		const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, float th_value)
	{
		vector<pair<float, float>> compare_srctgt_vec;
		if (features_src.size() == 0 || features_tgt.size() == 0) return compare_srctgt_vec;
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
		for (int j = 0; j < corr_.size(); j++)
		{
			float compare_src;
			{
				pcl::Correspondences corr_near;
				T_PointType_feature point_feature;
				point_feature.x = features_src[corr_[j].index_query];
				point_feature.y = 0.;
				corr_near = determineCorrespondences_featureScalar_kdtreeArg_singleQuery(point_feature, kdtree_feature_src, th_value, true);
				pcl::PointCloud<T_PointType>::Ptr cloud_near(new pcl::PointCloud<T_PointType>());
				for (int i = 0; i < corr_near.size(); i++)
					cloud_near->push_back(cloud_src_removed->points[corr_near[i].index_match]);
				if (corr_near.size() == 0)
					compare_src = 10000.;
				else
				{
					compare_src = CPointcloudBasic::calcCovarianceMatrix(CPointcloudBasic::calcEigenMatrixFromPointCloud(cloud_near)).trace();
					compare_src += 0.001;
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
				T_PointType_feature point_feature;
				point_feature.x = features_tgt[corr_[j].index_match];
				point_feature.y = 0.;
				corr_near = determineCorrespondences_featureScalar_kdtreeArg_singleQuery(point_feature, kdtree_feature_tgt, th_value, true);
				pcl::PointCloud<T_PointType>::Ptr cloud_near(new pcl::PointCloud<T_PointType>());
				for (int i = 0; i < corr_near.size(); i++)
					cloud_near->push_back(cloud_tgt_removed->points[corr_near[i].index_match]);
				if (corr_near.size() == 0)
					compare_tgt = 10000.;
				else
				{
					compare_tgt = CPointcloudBasic::calcCovarianceMatrix(CPointcloudBasic::calcEigenMatrixFromPointCloud(cloud_near)).trace();
					compare_tgt += 0.001;
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

	static void calcRanking_compareArg_eachValue(const vector<vector<pair<float, float>>> &compare_vecvec,
		vector<int> &frame_vec, vector<int> &corr_index_vec, vector<bool> &b_queryOrNot_vec, vector<float> &evaluation_vec, bool b_cout = false);

	static vector<vector<int>> calcRanking_compareArg(const vector<vector<pair<float, float>>> &compare_vecvec, bool b_cout = false);

	static void calcRanking_compareArg_eachValue_multipleEachCovariance(const vector<vector<float>> &compare_vecvec,
		vector<int> &frame_vec, vector<int> &corr_index_vec, vector<float> &evaluation_vec, bool b_cout = false)
	{
		cout << "calcRanking_compareArg_eachValue_multipleEachCovariance" << endl;
		vector<vector<float>> ranking_vecvec;	//[ranking][kind_value]
		for (int j = 0; j < compare_vecvec.size(); j++)
		{
			for (int i = 0; i < compare_vecvec[j].size(); i++)
			{
				vector<float> ranking_vec;
				ranking_vec.push_back((float)(j));						//frame
				ranking_vec.push_back((float)(i));				//index
				ranking_vec.push_back(compare_vecvec[j][i]);	//evaluation
				ranking_vecvec.push_back(ranking_vec);
			}
		}
		CTimeString::sortVector2d(ranking_vecvec, 2);

		for (int j = 0; j < ranking_vecvec.size(); j++)
		{
			frame_vec.push_back(ranking_vecvec[j][0]);
			corr_index_vec.push_back((int)(ranking_vecvec[j][1]));
			evaluation_vec.push_back(ranking_vecvec[j][2]);
		}

		if (!b_cout) return;

		for (int j = 0; j < ranking_vecvec.size(); j++)
		{
			if (j % 10 == 0)
			{
				cout << "j:" << j;
				cout << " frame:" << ranking_vecvec[j][0];
				cout << " corr_index:" << ((int)ranking_vecvec[j][1]);
				cout << " evaluation:" << ranking_vecvec[j][2] << endl;
			}
		}
		cout << endl;
	}

	static vector<vector<int>> calcRanking_compareArg_multipleEachCovariance(const vector<vector<pair<float, float>>> &compare_vecvec, bool b_cout = false)
	{
		cout << "calcRanking_compareArg_multipleEachCovariance (" << CTimeString::getTimeString() << ")" << endl;
		vector<vector<float>> compare_vecvec_multiple;	//[index_frame_pair][index_pair] :variance
		for (int j = 0; j < compare_vecvec.size(); j++)
		{
			vector<float> compare_vec_multiple;
			for (int i = 0; i < compare_vecvec[j].size(); i++)
				compare_vec_multiple.push_back(compare_vecvec[j][i].first * compare_vecvec[j][i].second);
			compare_vecvec_multiple.push_back(compare_vec_multiple);
		}

		vector<int> frame_vec;			//[ranking][value]
		vector<int> corr_index_vec;		//[ranking][value]
		vector<float> evaluation_vec;	//[ranking][value]
		calcRanking_compareArg_eachValue_multipleEachCovariance(compare_vecvec_multiple, frame_vec, corr_index_vec, evaluation_vec, b_cout);

		vector<vector<int>> rank_output_vecvec;	//[index_frame_pair][index_pair] :ranking, Some ingredients have no rank (invalid frame_pair).
		for (int j = 0; j < compare_vecvec_multiple.size(); j++)
		{
			vector<int> rank_output_vec;
			for (int i = 0; i < compare_vecvec_multiple[j].size(); i++)
				rank_output_vec.push_back(-1);	//initialization
			rank_output_vecvec.push_back(rank_output_vec);
		}

		{
			float value_before = 0.;
			float value_now = 0.;
			int rank_last;
			for (int j = 0; j < evaluation_vec.size(); j++)
			{
				value_now = evaluation_vec[j];
				if (j == 0 || value_now != value_before)
					rank_last = j;
				rank_output_vecvec[frame_vec[j]][corr_index_vec[j]] = rank_last;//[index_frame_pair][index_pair]
				value_before = value_now;
			}
		}

		if (!b_cout) return rank_output_vecvec;

		//show
		for (int j = 0; j < rank_output_vecvec.size(); j++)
		{
			for (int i = 0; i < rank_output_vecvec[j].size(); i++)
				if (i % 10 == 0)cout << "frame:" << j << " index:" << i << "  rank:" << rank_output_vecvec[j][i] << endl;
		}

		return rank_output_vecvec;
	}

	template <class T_PointType, typename T>
	static vector<vector<int>> calcRanking_featureScalar(const vector<pair<int, int>> &index_pair_vec, const vector<pcl::Correspondences> &corrs_vec,
		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec, const vector<vector<T>> &feature_vecvec, vector<vector<int>> &index_valid_vecvec, float th_nearest, bool b_cout = false)
	{
		vector<vector<pair<float, float>>> compare_vecvec;//[index_frame_pair][index_pair] :variance
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;		//index of cloud
			int i_src = index_pair_vec[j].second;		//index of cloud
			vector<pair<float, float>> compare_vec;
			compare_vec = calcRanking_compare_featureScalar(corrs_vec[j],
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
		//rank_output_vecvec = calcRanking_compareArg(compare_vecvec, b_cout);
		rank_output_vecvec = calcRanking_compareArg_multipleEachCovariance(compare_vecvec, b_cout);
		return rank_output_vecvec;
	}

	template <class T_PointType, typename T>
	static void determineCorrespondences_allFramesRanking_featureScalar_remove(const vector<vector<T>> &feature_vecvec, vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		const vector<pair<int, int>> &index_pair_vec, float th_nearest, float th_rank_rate, vector<vector<int>> index_valid_vecvec, vector<pcl::Correspondences> &corrs_vec, vector<vector<float>> &evaluation_vecvec, bool b_cout = false)
	{
		cout << "determineCorrespondences_allFramesRanking_featureScalar_remove" << endl;
		if (feature_vecvec.size() != cloud_vec.size())
		{
			cout << "ERROR: number of feature and one of pointcloud have different size." << endl;
			return;
		}
		cout << "  calc corr" << endl;
		corrs_vec.clear();//[index_frame_pair][index_pair]
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			cout << "  i_tgt:" << i_tgt;
			cout << ", i_src:" << i_src << endl;
			pcl::Correspondences corrs_;
			corrs_ = determineCorrespondences_featureScalar_remove(feature_vecvec[i_src], feature_vecvec[i_tgt],
				index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], th_nearest);
			cout << "    corrs_.size():" << corrs_.size() << endl;
			corrs_vec.push_back(corrs_);
		}
		cout << "  calc ranking" << endl;
		{
			vector<vector<int>> rank_vecvec;//[index_frame_pair][index_pair] : ranking, Some ingredients have no rank (invalid frame_pair).
			rank_vecvec = calcRanking_featureScalar(index_pair_vec, corrs_vec, cloud_vec, feature_vecvec, index_valid_vecvec, th_nearest, b_cout);
			vector<pcl::Correspondences> corrs_vec_temp;
			for (int j = 0; j < index_pair_vec.size(); j++)
			{
				pcl::Correspondences temp;
				corrs_vec_temp.push_back(temp);
			}
			//sort
			vector<vector<int>> sort_vecvec;	//[num][kind_value(index_frame_pair, index_pair, ranking)] : ranking
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
				corrs_vec_temp[index_frame_pair].push_back(corrs_vec[index_frame_pair][index_pair]);
				evaluation_vecvec[index_frame_pair].push_back((float)sort_vecvec[j][2]);
			}
			corrs_vec = corrs_vec_temp;//[index_frame_pair][index_pair], Some frame_pair has no correspond (invalid frame_pair).
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
	static void determineCorrespondences_allFramesRanking_featureScalar_remove(const vector<vector<T>> &feature_vecvec, vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		const vector<pair<int, int>> &index_pair_vec, float th_nearest, float th_rank_rate, vector<vector<int>> index_valid_vecvec, vector<pcl::Correspondences> &corrs_vec, bool b_cout = false)
	{
		vector<vector<float>> evaluation_vecvec;
		determineCorrespondences_allFramesRanking_featureScalar_remove(feature_vecvec, cloud_vec, index_pair_vec, th_nearest, th_rank_rate, index_valid_vecvec, corrs_vec, evaluation_vecvec, b_cout);
	}

	template <class T_PointType, typename T>
	static void determineCorrespondences_allFramesRanking_featureScalar(const vector<vector<T>> &feature_vecvec, vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		const vector<pair<int, int>> &index_pair_vec, float th_nearest, float th_rank_rate, vector<pcl::Correspondences> &corrs_vec, vector<vector<float>> &evaluation_vecvec, bool b_cout = false)
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
		determineCorrespondences_allFramesRanking_featureScalar_remove(feature_vecvec, cloud_vec, index_pair_vec, th_nearest, th_rank_rate, index_valid_vecvec, corrs_vec, evaluation_vecvec, b_cout);
	}

	template <class T_PointType, typename T>
	static void determineCorrespondences_allFramesRanking_featureScalar(const vector<vector<T>> &feature_vecvec, vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
		const vector<pair<int, int>> &index_pair_vec, float th_nearest, float th_rank_rate, vector<pcl::Correspondences> &corrs_vec, bool b_cout = false)
	{
		vector<vector<float>> evaluation_vecvec;
		determineCorrespondences_allFramesRanking_featureScalar(feature_vecvec, cloud_vec, index_pair_vec, th_nearest, th_rank_rate, corrs_vec, evaluation_vecvec, b_cout);
	}

	template <class T_PointType>
	static vector<pcl::Correspondences> determineCorrespondences_geometricConstraint(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt, const pcl::Correspondences &corr_, float th_fraction, bool b_cout = false)
	{
		vector<pcl::Correspondences> corrs_output_vec;
		//RatioOfDistanceOfSrcAndTgt
		int num_corr_init = corr_.size();
		if (num_corr_init < 2)
		{
			cout << "Few correspondednces inputed." << endl;
			return corrs_output_vec;
		}
		//calc ratio		
		vector<vector<bool>> b_matrix;
		for (int j = 0; j < num_corr_init; j++)
		{
			vector<bool> b_temp_vec;
			b_temp_vec.resize(num_corr_init);
			fill(b_temp_vec.begin(), b_temp_vec.begin(), false);
			b_matrix.push_back(b_temp_vec);
		}
		int num_valid = 0;
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
				else if (distance_src >= distance_tgt)
					value_ = distance_tgt / distance_src;
				else/* if (distance_src < distance_tgt)*/
					value_ = distance_src / distance_tgt;
				if (value_ >= th_fraction)
				{
					b_matrix[j][i] = true;
					num_valid++;
				}
			}
		}

		if (num_valid < 3)
		{
			//cout << "ERROR(CExtendableICP::getCorrespondance_RatioOfDistanceOfSrcAndTgt): Few correspondednces exist simultaneously." << endl;
			//throw std::runtime_error("ERROR(CExtendableICP::getCorrespondance_RatioOfDistanceOfSrcAndTgt): Few correspondednces exist simultaneously.");
			cout << "Few correspondednces exist simultaneously. (CExtendableICP::getCorrespondance_RatioOfDistanceOfSrcAndTgt)" << endl;
			return corrs_output_vec;
		}

		vector<vector<int>> corr_pair_cluster_vecvec_new;
		{
			cout << "b_matrix.size():" << b_matrix.size() << endl;
			cout << "b_matrix[0].size():" << b_matrix[0].size() << endl;
			corr_pair_cluster_vecvec_new = CTimeString::getIntCluster_boolMatrix(b_matrix, 6, 5, 2);
		}

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
	static pcl::Correspondences determineCorrespondences_geometricConstraint_evaluateCluster(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src, boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_tgt,
		const vector<pcl::Correspondences> &corrs_vec, int i_method, bool b_cout = false)
	{
		if (corrs_vec.size() == 0)
		{
			cout << "input has no corrs in CExtendableICP::determineCorrespondences_geometricConstraint_evaluateCluster" << endl;
			pcl::Correspondences temp;
			return temp;
		}
		//i_method:0, evaluation by the mean distance of correspondences after transformation
		//i_method:1, evaluation by the median distance of correspondences after transformation
		//i_method:2, evaluation by the mean distance of nearest neighbors after transformation
		//i_method:3, evaluation by the median distance of nearest neighbors after transformation

		vector<float> evaluation_vec;
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_src_moving(new pcl::PointCloud<T_PointType>);

		vector<Eigen::Matrix4d> transformation_vec;
		for (int j = 0; j < corrs_vec.size(); j++)
		{
			Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
			CPointcloudBasic::estimateRigidTransformation_static(cloud_src, cloud_tgt, corrs_vec[j], transformation_matrix);
			transformation_vec.push_back(transformation_matrix.cast<double>());
		}

		for (int j = 0; j < transformation_vec.size(); j++)
		{
			pcl::transformPointCloud(*cloud_src, *cloud_src_moving, CPointcloudBasic::calcAffine3fFromHomogeneousMatrix(transformation_vec[j]));
			vector<float> distance_vec;

			//distances of correspondences 
			if (i_method == 0 || i_method == 1)
			{
				for (int i = 0; i < corrs_vec[j].size(); i++)
				{
					T_PointType point_src = cloud_src_moving->points[corrs_vec[j][i].index_query];
					T_PointType point_tgt = cloud_tgt->points[corrs_vec[j][i].index_match];
					distance_vec.push_back(sqrt(
						pow(point_src.x - point_tgt.x, 2.)
						+ pow(point_src.y - point_tgt.y, 2.)
						+ pow(point_src.z - point_tgt.z, 2.)));
				}
			}
			//distances of nearest neighbors
			else if (i_method == 2 || i_method == 3)
			{
				pcl::Correspondences corrs = CPointcloudBasic::determineCorrespondences_output(cloud_src_moving, cloud_tgt);
				for (int i = 0; i < corrs.size(); i++)
					distance_vec.push_back(corrs[i].distance);
			}

			if (i_method == 0 || i_method == 2)
			{
				float distance_sum = 0.;
				for (int i = 0; i < distance_vec.size(); i++)
					distance_sum += distance_vec[i];
				distance_sum /= (float)distance_vec.size();
				evaluation_vec.push_back(distance_sum);
			}
			else if (i_method == 1 || i_method == 3)
			{
				evaluation_vec.push_back(CTimeString::getMedian(distance_vec));
			}
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

		//cout << "evaluation_vecvec[0][0]:" << evaluation_vecvec[0][0] << endl;

		return corrs_vec[evaluation_vecvec[0][0]];
	}

};
