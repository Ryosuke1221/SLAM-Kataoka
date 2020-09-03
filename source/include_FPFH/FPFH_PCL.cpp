#include "FPFH_PCL.h"

vector<float> CFPFH_PCL::getErrorOfFPFHSource_corr(float &median_arg, pcl::Correspondences correspondences,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt)
{
	vector<float> error_vec;
	error_vec.resize(fpfh_src->size());
	fill(error_vec.begin(), error_vec.end(), 100000.);

	bool b_cout = false;

	int num_nan = 0;

	for (int j = 0; j < correspondences.size(); j++)
	{
		int i_src, i_tgt;
		i_src = correspondences[j].index_query;
		i_tgt = correspondences[j].index_match;

		float error_squared = 0.;
		bool b_nan = false;

		for (int i = 0; i < sizeof(fpfh_src->points[i_src].histogram); i++)
		{
			if (isnan(fpfh_src->points[i_src].histogram[i]) || isnan(fpfh_tgt->points[i_tgt].histogram[i])
				|| fpfh_src->points[i_src].histogram[i] > 200. || fpfh_tgt->points[i_tgt].histogram[i] > 200.
				|| fpfh_src->points[i_src].histogram[i] < 0. || fpfh_tgt->points[i_tgt].histogram[i] < 0.)
			{
				b_nan = true;
				num_nan++;
				cout << "j:" << j << " i:" << i << " nan occored" << endl;
				//cout << "correspondences.size():" << correspondences.size() << endl;
				//cout << "sizeof(fpfh_src->points[i_src].histogram):" << sizeof(fpfh_src->points[i_src].histogram) << endl;
				//cout << "sizeof(fpfh_tgt->points[i_tgt].histogram):" << sizeof(fpfh_tgt->points[i_tgt].histogram) << endl;
				//cout << "i_src:" << i_src << " i_tgt:" << i_tgt << endl;
				//cout << "fpfh_src->points.size():" << fpfh_src->points.size() << endl;
				//cout << "fpfh_tgt->points.size():" << fpfh_tgt->points.size() << endl;
				break;
			}
			else
				error_squared += pow(fpfh_src->points[i_src].histogram[i] - fpfh_tgt->points[i_tgt].histogram[i], 2.);
		}
		if (b_nan)
			error_squared = 10000.;
		else
			error_squared = sqrt(error_squared);

		error_vec[j] = error_squared;	//value or 100000.(init) or 10000.(invalid)
	}
	if (b_cout) if (num_nan != 0) cout << "num_nan:" << num_nan << endl;

	//show inlier rate
	if (b_cout) cout << "inlier rate:" << (float)correspondences.size() / (float)fpfh_src->size() << endl;

	//show median
	//float median_;
	vector<float> distance_vec;
	for (int i = 0; i < error_vec.size(); i++)
	{
		//if (error_vec[i] >= 10000.) continue;	//invalid: outlier and not nan
		if (error_vec[i] >= 100000.) continue;	//invalid: outlier
		distance_vec.push_back(error_vec[i]);
	}
	int size = distance_vec.size();

	sort(distance_vec.begin(), distance_vec.end());

	if (size % 2 == 1)
		median_arg = distance_vec[(size - 1) / 2];
	else
		median_arg = (distance_vec[(size / 2) - 1] + distance_vec[size / 2]) / 2.;
	if (b_cout) cout << "fpfh error median_arg:" << median_arg << endl;

	return error_vec;
}

vector<float> CFPFH_PCL::getFPFHVariance(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_)
{
	vector<float> fpfh_hist_mean;
	fpfh_hist_mean.resize(sizeof(fpfh_->points[0].histogram));
	fill(fpfh_hist_mean.begin(), fpfh_hist_mean.end(), 0.);
	int num_nan = 0;
	vector<bool> b_isnan_vec;

	bool b_cout = false;

	for (int j = 0; j < fpfh_->size(); j++)
	{
		bool b_has_nan = false;
		vector<float> fpfh_hist_mean_temp;
		for (int i = 0; i < fpfh_hist_mean.size(); i++)
		{
			//remove nan
			if (isnan(fpfh_->points[j].histogram[i]) || fpfh_->points[j].histogram[i] > 100. || fpfh_->points[j].histogram[i] < 0)
			{
				num_nan++;
				b_has_nan = true;
				if (b_cout) cout << "nan occored in j;" << j << " i:" << i << endl;
				break;
			}
			else
				//fpfh_hist_mean[i] += fpfh_->points[j].histogram[i];
				fpfh_hist_mean_temp.push_back(fpfh_->points[j].histogram[i]);
		}
		if (b_has_nan)
		{
			b_isnan_vec.push_back(true);
			continue;
		}
		else
		{
			for (int i = 0; i < fpfh_hist_mean.size(); i++)
				fpfh_hist_mean[i] += fpfh_hist_mean_temp[i];
			b_isnan_vec.push_back(false);
		}
	}
	if (b_cout) cout << "num_nan:" << num_nan << endl;

	int num_valid_points = 0;
	for (int j = 0; j < b_isnan_vec.size(); j++)
		if (!b_isnan_vec[j]) num_valid_points++;

	for (int i = 0; i < fpfh_hist_mean.size(); i++)
		fpfh_hist_mean[i] /= num_valid_points;

	vector<float> fpfh_hist_SquaredError;
	fpfh_hist_SquaredError.resize(sizeof(fpfh_->points[0].histogram));
	fill(fpfh_hist_SquaredError.begin(), fpfh_hist_SquaredError.end(), 0.);

	for (int j = 0; j < fpfh_->size(); j++)
	{
		if (b_isnan_vec[j]) continue;
		for (int i = 0; i < fpfh_hist_mean.size(); i++)
			fpfh_hist_SquaredError[i] += pow(fpfh_->points[j].histogram[i] - fpfh_hist_mean[i], 2.);
	}

	for (int i = 0; i < fpfh_hist_SquaredError.size(); i++)
		fpfh_hist_SquaredError[i] /= num_valid_points;

	return fpfh_hist_SquaredError;
}

pcl::Correspondences CFPFH_PCL::getNearestOfFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	int num_near, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh)
{
	pcl::Correspondences corr_vec;
	vector<vector<float>> squaredDistance_vecvec;
	squaredDistance_vecvec.clear();
	vector<vector<int>> index_vecvec;
	for (int j = 0; j < fpfh_src->size(); j++)
	{
		vector<int> index_vec;
		vector<float> squaredDistance_vec;
		int found_neighs = kdtree_fpfh->nearestKSearch(*fpfh_src, j, num_near, index_vec, squaredDistance_vec);
		for (int i = 0; i < index_vec.size(); i++)
		{
			pcl::Correspondence corr_;
			corr_.index_query = j;
			corr_.index_match = index_vec[i];
			corr_.distance = squaredDistance_vec[i];
			corr_vec.push_back(corr_);
		}
	}
	return corr_vec;
}

vector<vector<bool>> CFPFH_PCL::getFPFHMeanAndSigma(vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec,
	vector<float> &mean_fpfh, vector<float> &sigma_fpfh)
{
	mean_fpfh.clear();
	sigma_fpfh.clear();
	int num_hist;
	{
		//https://riptutorial.com/ja/cplusplus/example/2327/%E9%85%8D%E5%88%97%E3%82%92std----vector%E3%81%AB%E5%A4%89%E6%8F%9B%E3%81%99%E3%82%8B
		vector<float> feature_vec_temp(std::begin(fpfh_vec.at(0)->points.at(0).histogram),
			std::end(fpfh_vec.at(0)->points.at(0).histogram));
		num_hist = feature_vec_temp.size();
	}
	mean_fpfh.resize(num_hist);
	fill(mean_fpfh.begin(), mean_fpfh.end(), 0.);
	sigma_fpfh.resize(num_hist);
	fill(sigma_fpfh.begin(), sigma_fpfh.end(), 0.);

	int num_points = 0;
	for (int j = 0; j < fpfh_vec.size(); j++)
		num_points += fpfh_vec[j]->size();

	//ready for removing nan
	vector<vector<bool>> b_invalid_vecvec;
	int num_points_invalid = 0;
	for (int j = 0; j < fpfh_vec.size(); j++)
	{
		vector<bool> b_invalid_vec;
		b_invalid_vec.resize(fpfh_vec[j]->size());
		fill(b_invalid_vec.begin(), b_invalid_vec.end(), false);
		b_invalid_vecvec.push_back(b_invalid_vec);
	}

	//calc mean
	for (int j = 0; j < fpfh_vec.size(); j++)
	{
		for (int i = 0; i < fpfh_vec[j]->points.size(); i++)
		{
			vector<float> feature_vec;
			feature_vec.clear();
			vector<float> feature_vec_temp(std::begin(fpfh_vec.at(j)->points.at(i).histogram),
				std::end(fpfh_vec.at(j)->points.at(i).histogram));
			bool b_invalid_point = false;
			for (int k = 0; k < feature_vec_temp.size(); k++)
			{
				if (isnan(feature_vec_temp[k]) || feature_vec_temp[k] > 100. || feature_vec_temp[k] < 0.)
				{
					b_invalid_point = true;
					b_invalid_vecvec[j][i] = true;
					num_points_invalid++;
					break;
				}
				else feature_vec.push_back(feature_vec_temp[k]);

			}
			if (b_invalid_point) continue;
			for (int k = 0; k < feature_vec.size(); k++)
				mean_fpfh[k] += feature_vec[k];
		}
	}
	for (int j = 0; j < num_hist; j++)
		mean_fpfh[j] /= (float)(num_points - num_points_invalid);

	//cout << "num_points_invalid:" << num_points_invalid << endl;

	//calc sigma
	for (int j = 0; j < fpfh_vec.size(); j++)
	{
		for (int i = 0; i < fpfh_vec[j]->size(); i++)
		{
			if (b_invalid_vecvec[j][i]) continue;
			vector<float> feature_vec_temp(std::begin(fpfh_vec.at(j)->points.at(i).histogram),
				std::end(fpfh_vec.at(j)->points.at(i).histogram));
			for (int k = 0; k < feature_vec_temp.size(); k++)
				sigma_fpfh[k] += pow(feature_vec_temp[k] - mean_fpfh[k], 2.);
		}
	}
	for (int j = 0; j < num_hist; j++)
		sigma_fpfh[j] = sqrt(sigma_fpfh[j] / (float)(num_points - num_points_invalid));

	return b_invalid_vecvec;
}

vector<bool> CFPFH_PCL::getFPFH_unique(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_feature,
	vector<bool> b_invalidPoint_vec, vector<float> mean_fpfh_vec, vector<float> sigma_fpfh_vec, float beta_fpfh)
{
	vector<int> index_unique_vec;
	for (int j = 0; j < fpfh_feature->size(); j++)
	{
		if (b_invalidPoint_vec[j]) continue;
		vector<float> feature_vec_temp(std::begin(fpfh_feature->points.at(j).histogram),
			std::end(fpfh_feature->points.at(j).histogram));
		bool b_unique = false;
		for (int i = 0; i < feature_vec_temp.size(); i++)
		{
			float th_small = mean_fpfh_vec[i] - sigma_fpfh_vec[i] * beta_fpfh;
			float th_big = mean_fpfh_vec[i] + sigma_fpfh_vec[i] * beta_fpfh;

			if (feature_vec_temp[i] <= th_small || th_big <= feature_vec_temp[i])
			{
				b_unique = true;
				break;
			}
		}
		if (b_unique) index_unique_vec.push_back(j);
	}
	vector<bool> b_unique_vec;
	b_unique_vec.resize(fpfh_feature->size());
	fill(b_unique_vec.begin(), b_unique_vec.end(), false);
	for (int j = 0; j < index_unique_vec.size(); j++)
		b_unique_vec[index_unique_vec[j]] = true;
	return b_unique_vec;
}

//template <typename T_PointType>
//vector<vector<int>> CFPFH_PCL::getFPFH_unique_someRadius(vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_vec,
//	vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec, float radius_FPFH_center,
//	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> &fpfh_vec_output, bool b_cout = false)
//{
//	vector<float> radius_FPFH_vec;
//	radius_FPFH_vec.push_back(radius_FPFH_center * 0.75);
//	radius_FPFH_vec.push_back(radius_FPFH_center);
//	radius_FPFH_vec.push_back(radius_FPFH_center * 1.25);
//	vector<vector<int>> index_vecvec;
//	index_vecvec = getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_vec, fpfh_vec_output, b_cout);
//	return index_vecvec;
//}

pcl::Correspondences CFPFH_PCL::getCorrespondences_eachPairHaving(const pcl::Correspondences &corr_src_tgt,
	const pcl::Correspondences &corr_tgt_src)
{
	vector<vector<int>> index_corr_temp_vecvec;
	for (int j = 0; j < corr_src_tgt.size(); j++)
	{
		vector<int> corr_vec;
		corr_vec.push_back(corr_src_tgt[j].index_query);
		corr_vec.push_back(corr_src_tgt[j].index_match);
		index_corr_temp_vecvec.push_back(corr_vec);
	}
	for (int j = 0; j < corr_tgt_src.size(); j++)
	{
		vector<int> corr_vec;
		corr_vec.push_back(corr_tgt_src[j].index_match);
		corr_vec.push_back(corr_tgt_src[j].index_query);
		index_corr_temp_vecvec.push_back(corr_vec);
	}
	CTimeString::sortVector2d_2dimension(index_corr_temp_vecvec, 0, 1);
	vector<vector<int>> index_corr_vecvec;
	int i_src_before;
	int i_tgt_before;
	for (int j = 0; j < index_corr_temp_vecvec.size(); j++)
	{
		if (j != 0)
		{
			if (index_corr_temp_vecvec[j][0] == i_src_before
				&& index_corr_temp_vecvec[j][1] == i_tgt_before)
			{
				vector<int> corr_vec;
				corr_vec.push_back(index_corr_temp_vecvec[j][0]);
				corr_vec.push_back(index_corr_temp_vecvec[j][1]);
				index_corr_vecvec.push_back(corr_vec);
			}
		}
		i_src_before = index_corr_temp_vecvec[j][0];
		i_tgt_before = index_corr_temp_vecvec[j][1];
	}
	pcl::Correspondences corr_new;
	for (int j = 0; j < index_corr_vecvec.size(); j++)
	{
		int i_src = index_corr_vecvec[j][0];
		int i_tgt = index_corr_vecvec[j][1];
		for (int i = 0; i < corr_src_tgt.size(); i++)
		{
			if (!(corr_src_tgt[i].index_query == i_src
				&& corr_src_tgt[i].index_match == i_tgt))
				continue;
			corr_new.push_back(corr_src_tgt[i]);
		}
	}
	return corr_new;
}

pcl::Correspondences CFPFH_PCL::getNearestOfFPFH_eachPairHaving(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, int num_near, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src,
	pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt)
{
	pcl::Correspondences corr_src_tgt = getNearestOfFPFH(fpfh_src, num_near, kdtree_fpfh_tgt);
	pcl::Correspondences corr_tgt_src = getNearestOfFPFH(fpfh_tgt, num_near, kdtree_fpfh_src);
	pcl::Correspondences corr_new = getCorrespondences_eachPairHaving(corr_src_tgt, corr_tgt_src);
	return corr_new;
}

pcl::Correspondences CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, int num_near, vector<int> index_unique_vec_src, vector<int> index_unique_vec_tgt)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src_removed(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt_removed(new pcl::PointCloud<pcl::FPFHSignature33>);
	{
		pcl::copyPointCloud(*fpfh_src, *fpfh_src_removed);
		vector<bool> b_unique_vec;
		b_unique_vec.resize(fpfh_src->size());
		fill(b_unique_vec.begin(), b_unique_vec.end(), false);
		for (int j = 0; j < index_unique_vec_src.size(); j++)
			b_unique_vec[index_unique_vec_src[j]] = true;
		for (int j = fpfh_src_removed->size() - 1; j >= 0; j--)
			if (!b_unique_vec[j]) fpfh_src_removed->erase(fpfh_src_removed->begin() + j);
	}
	{
		pcl::copyPointCloud(*fpfh_tgt, *fpfh_tgt_removed);
		vector<bool> b_unique_vec;
		b_unique_vec.resize(fpfh_tgt->size());
		fill(b_unique_vec.begin(), b_unique_vec.end(), false);
		for (int j = 0; j < index_unique_vec_tgt.size(); j++)
			b_unique_vec[index_unique_vec_tgt[j]] = true;
		for (int j = fpfh_tgt_removed->size() - 1; j >= 0; j--)
			if (!b_unique_vec[j]) fpfh_tgt_removed->erase(fpfh_tgt_removed->begin() + j);
	}
	pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
	pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt(new pcl::KdTreeFLANN<pcl::FPFHSignature33>);
	kdtree_fpfh_src->setInputCloud(fpfh_src_removed);
	kdtree_fpfh_tgt->setInputCloud(fpfh_tgt_removed);
	pcl::Correspondences corrs_;
	corrs_ = getNearestOfFPFH_eachPairHaving(fpfh_src_removed, fpfh_tgt_removed, num_near, kdtree_fpfh_src, kdtree_fpfh_tgt);
	for (int j = 0; j < corrs_.size(); j++)
	{
		corrs_[j].index_query = index_unique_vec_src[corrs_[j].index_query];
		corrs_[j].index_match = index_unique_vec_tgt[corrs_[j].index_match];
	}
	return corrs_;
}