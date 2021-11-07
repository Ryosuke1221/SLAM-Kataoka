#include "GlobalFeatureRegistration.h"

void CGlobalFeatureRegistration::getFPFHMeanAndSigma(vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec,
	vector<float> &mean_fpfh, vector<float> &sigma_fpfh)
{
	mean_fpfh.clear();
	sigma_fpfh.clear();
	int num_hist;
	{
		//https://riptutorial.com/ja/cplusplus/example/2327/%E9%85%8D%E5%88%97%E3%82%92std----vector%E3%81%AB%E5%A4%89%E6%8F%9B%E3%81%99%E3%82%8B
		vector<float> feature_vec(std::begin(fpfh_vec.at(0)->points.at(0).histogram),
			std::end(fpfh_vec.at(0)->points.at(0).histogram));
		num_hist = feature_vec.size();
	}
	mean_fpfh.resize(num_hist);
	fill(mean_fpfh.begin(), mean_fpfh.end(), 0.);
	sigma_fpfh.resize(num_hist);
	fill(sigma_fpfh.begin(), sigma_fpfh.end(), 0.);

	int num_points = 0;
	for (int j = 0; j < fpfh_vec.size(); j++)
		num_points += fpfh_vec[j]->size();

	//calc mean
	for (int j = 0; j < fpfh_vec.size(); j++)
	{
		for (int i = 0; i < fpfh_vec[j]->points.size(); i++)
		{
			vector<float> feature_vec(std::begin(fpfh_vec.at(j)->points.at(i).histogram),
				std::end(fpfh_vec.at(j)->points.at(i).histogram));
			for (int k = 0; k < feature_vec.size(); k++)
				mean_fpfh[k] += feature_vec[k];
		}
	}
	for (int j = 0; j < num_hist; j++)
		mean_fpfh[j] /= (float)num_points;

	//cout << "num_points_invalid:" << num_points_invalid << endl;

	//calc sigma
	for (int j = 0; j < fpfh_vec.size(); j++)
	{
		for (int i = 0; i < fpfh_vec[j]->size(); i++)
		{
			vector<float> feature_vec(std::begin(fpfh_vec.at(j)->points.at(i).histogram),
				std::end(fpfh_vec.at(j)->points.at(i).histogram));
			for (int k = 0; k < feature_vec.size(); k++)
				sigma_fpfh[k] += pow(feature_vec[k] - mean_fpfh[k], 2.);
		}
	}
	for (int j = 0; j < num_hist; j++)
		sigma_fpfh[j] = sqrt(sigma_fpfh[j] / (float)num_points);
}

vector<bool> CGlobalFeatureRegistration::getFPFH_unique(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_feature,
	vector<float> mean_fpfh_vec, vector<float> sigma_fpfh_vec, float beta_fpfh)
{
	vector<int> index_unique_vec;
	for (int j = 0; j < fpfh_feature->size(); j++)
	{
		vector<float> feature_vec(std::begin(fpfh_feature->points.at(j).histogram),
			std::end(fpfh_feature->points.at(j).histogram));
		bool b_unique = false;
		for (int i = 0; i < feature_vec.size(); i++)
		{
			float th_small = mean_fpfh_vec[i] - sigma_fpfh_vec[i] * beta_fpfh;
			float th_big = mean_fpfh_vec[i] + sigma_fpfh_vec[i] * beta_fpfh;

			if (feature_vec[i] <= th_small || th_big <= feature_vec[i])
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

vector<vector<int>> CGlobalFeatureRegistration::getFPFH_unique_someRadius_inputFile(string dir_, vector<string> filenames_cloud_vec, bool b_cout)
{
	vector<vector<int>> index_vecvec;
	string filename_input;
	{
		vector<string> filenames_temp;
		CTimeString::getFileNames_extension(dir_, filenames_temp, "validPoint_FPFH");
		if (filenames_temp.size() == 0)
		{
			cout << "ERROR: no validPoint_FPFH found." << endl;
			throw std::runtime_error("ERROR: some validPoint_FPFH found.");
		}

		else if (filenames_temp.size() != 1)
		{
			cout << "ERROR: too many validPoint_FPFH found." << endl;
			throw std::runtime_error("ERROR: some validPoint_FPFH found.");
		}
		filename_input = filenames_temp[0];
	}
	vector<vector<string>> s_input_vecvec;
	s_input_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + filename_input);
	int index_cloud = 0;
	for (int j = 1; j < s_input_vecvec.size(); j++)
	{
		if (filenames_cloud_vec.size() == index_cloud) break;
		if (s_input_vecvec[j][0] == filenames_cloud_vec[index_cloud])
		{
			vector<int> index_vec;
			for (int i = 1; i < s_input_vecvec[j].size(); i++)
				index_vec.push_back(stoi(s_input_vecvec[j][i]));
			index_vecvec.push_back(index_vec);
			index_cloud++;
		}
	}
	return index_vecvec;
}

pcl::Correspondences CGlobalFeatureRegistration::getCorrespondences_eachPairHaving(const pcl::Correspondences &corr_src_tgt,
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

pcl::Correspondences CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_num(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src,
	pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt, int num_near)
{
	pcl::Correspondences corr_src_tgt = CFPFH_PCL::getNearestOfFPFH(fpfh_src, num_near, kdtree_fpfh_tgt);
	pcl::Correspondences corr_tgt_src = CFPFH_PCL::getNearestOfFPFH(fpfh_tgt, num_near, kdtree_fpfh_src);
	pcl::Correspondences corr_new = getCorrespondences_eachPairHaving(corr_src_tgt, corr_tgt_src);
	return corr_new;
}

pcl::Correspondences CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_num_remove(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt, int num_near)
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
	corrs_ = determineCorrespondences_featureFpfh_eachPairHaving_num(fpfh_src_removed, fpfh_tgt_removed, kdtree_fpfh_src, kdtree_fpfh_tgt, num_near);
	for (int j = 0; j < corrs_.size(); j++)
	{
		corrs_[j].index_query = index_unique_vec_src[corrs_[j].index_query];
		corrs_[j].index_match = index_unique_vec_tgt[corrs_[j].index_match];
	}
	return corrs_;
}

pcl::Correspondences CGlobalFeatureRegistration::determineCorrespondences_featureFpfh(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	int num_near_max, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh, float th_value)
{
	pcl::Correspondences corr_vec;
	vector<vector<float>> squaredDistance_vecvec;
	squaredDistance_vecvec.clear();
	vector<vector<int>> index_vecvec;
	for (int j = 0; j < fpfh_src->size(); j++)
	{
		vector<int> index_vec;
		vector<float> squaredDistance_vec;
		int found_neighs = kdtree_fpfh->nearestKSearch(*fpfh_src, j, num_near_max, index_vec, squaredDistance_vec);
		for (int i = 0; i < index_vec.size(); i++)
		{
			if (th_value < squaredDistance_vec[i]) continue;
			pcl::Correspondence corr_;
			corr_.index_query = j;
			corr_.index_match = index_vec[i];
			corr_.distance = squaredDistance_vec[i];
			corr_vec.push_back(corr_);
		}
	}
	return corr_vec;
}

pcl::Correspondences CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_src,
	pcl::KdTreeFLANN<pcl::FPFHSignature33>::Ptr kdtree_fpfh_tgt, int num_near_max, float th_value)
{
	pcl::Correspondences corr_src_tgt = determineCorrespondences_featureFpfh(fpfh_src, num_near_max, kdtree_fpfh_tgt, th_value);
	pcl::Correspondences corr_tgt_src = determineCorrespondences_featureFpfh(fpfh_tgt, num_near_max, kdtree_fpfh_src, th_value);
	pcl::Correspondences corr_new = getCorrespondences_eachPairHaving(corr_src_tgt, corr_tgt_src);
	return corr_new;
}

pcl::Correspondences CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_remove(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt, const vector<int> &index_unique_vec_src, const vector<int> &index_unique_vec_tgt,
	int num_near_max, float th_value)
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
	corrs_ = determineCorrespondences_featureFpfh_eachPairHaving(fpfh_src_removed, fpfh_tgt_removed, kdtree_fpfh_src, kdtree_fpfh_tgt, num_near_max, th_value);
	for (int j = 0; j < corrs_.size(); j++)
	{
		corrs_[j].index_query = index_unique_vec_src[corrs_[j].index_query];
		corrs_[j].index_match = index_unique_vec_tgt[corrs_[j].index_match];
	}
	return corrs_;
}

pcl::Correspondences CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_kdtreeArg_singleQuery(pcl::FPFHSignature33 point_query,
	const boost::shared_ptr<pcl::KdTreeFLANN<pcl::FPFHSignature33>> kdtree_tgt, float th_nearest_fpfh, int th_nearest_num, bool b_multipleNear)
{
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfh_src->push_back(point_query);
	pcl::Correspondences correspondences;
	std::vector<int> index(1);
	std::vector<float> distance(1);
	int found_neighs = kdtree_tgt->nearestKSearch(*fpfh_src, 0, th_nearest_num, index, distance);
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
	for (int i = correspondences.size() - 1; i >= 0; i--)
		if (th_nearest_fpfh < distance[i]) correspondences.erase(correspondences.begin() + i);
	if (correspondences.size() == 0) cout << "ERROR: no corr found" << endl;
	return correspondences;
}

vector<vector<int>> CGlobalFeatureRegistration::calcValidIndex_feature(const vector<vector<float>> &feature_vecvec, int num_bin_hist, bool b_showHistogram)
{
	float value_max_hist;
	float value_min_hist;
	vector<int> hist_vec_all;
	{
		cout << "show histogram of all features" << endl;
		vector<float> features_all;
		for (int j = 0; j < feature_vecvec.size(); j++)
			features_all.insert(features_all.end(), feature_vecvec[j].begin(), feature_vecvec[j].end());
		vector<float> feature_calcHistogram;
		for (int j = 0; j < features_all.size(); j++)
			feature_calcHistogram.push_back(features_all[j]);

		//histogram_all
		value_max_hist = -std::numeric_limits<float>::max();
		value_min_hist = std::numeric_limits<float>::max();

		for (int j = 0; j < feature_calcHistogram.size(); j++)
		{
			if (value_max_hist < feature_calcHistogram[j]) value_max_hist = feature_calcHistogram[j];
			if (value_min_hist > feature_calcHistogram[j]) value_min_hist = feature_calcHistogram[j];
		}
		cout << "value_max_hist:" << value_max_hist << endl;
		cout << "value_min_hist:" << value_min_hist << endl;
		//float range_hist = (value_max_hist - value_min_hist) / (float)num_bin_hist;
		hist_vec_all = CTimeString::getHistogram(feature_calcHistogram, value_max_hist, value_min_hist,
			num_bin_hist, true);
		cout << endl;
	}

	////output histogram
	//{
	//	string dir_ = "../../data/process13_DoDifferential";
	//	vector <vector<int>> hist_output_vecvec;
	//	hist_output_vecvec.push_back(hist_vec_all);
	//	CTimeString::getCSVFromVecVec(CTimeString::getTranspositionOfVecVec(hist_output_vecvec), dir_ + "/" + CTimeString::getTimeString() + "_histogram.csv");
	//}

	//remove points in biggest bin
	vector<vector<int>> index_valid_vecvec;
	for (int j = 0; j < feature_vecvec.size(); j++)
	{
		index_valid_vecvec.push_back(calcFeatureIndex_removingBiggestBin(feature_vecvec[j],
			hist_vec_all, value_max_hist, value_min_hist));
	}

	if (b_showHistogram)
	{
		cout << "show Histogram" << endl;
		for (int j = 0; j < feature_vecvec.size(); j++)
		{
			cout << "j:" << j << endl;
			vector<float> feature_calcHistogram;
			for (int i = 0; i < index_valid_vecvec[j].size(); i++)
				feature_calcHistogram.push_back(feature_vecvec[j][index_valid_vecvec[j][i]]);

			vector<int> hist_vec = CTimeString::getHistogram(feature_calcHistogram, value_max_hist, value_min_hist,
				num_bin_hist, true);
			cout << endl;
		}
	}

	return index_valid_vecvec;
}

void CGlobalFeatureRegistration::calcRanking_compareArg_eachValue(const vector<vector<pair<float, float>>> &compare_vecvec,
	vector<int> &frame_vec, vector<int> &corr_index_vec, vector<bool> &b_queryOrNot_vec, vector<float> &evaluation_vec, bool b_cout)
{
	vector<vector<float>> ranking_vecvec;
	for (int j = 0; j < compare_vecvec.size(); j++)
	{
		for (int i = 0; i < compare_vecvec[j].size(); i++)
		{
			vector<float> ranking_vec_query;
			ranking_vec_query.push_back((float)(j));						//frame
			ranking_vec_query.push_back((float)(i * 2 + 0));				//0:query, 1:match
			ranking_vec_query.push_back(compare_vecvec[j][i].first);	//evaluation
			ranking_vecvec.push_back(ranking_vec_query);
			vector<float> ranking_vec_match;
			ranking_vec_match.push_back((float)(j));						//frame
			ranking_vec_match.push_back((float)(i * 2 + 1));				//0:query, 1:match
			ranking_vec_match.push_back(compare_vecvec[j][i].second);	//evaluation
			ranking_vecvec.push_back(ranking_vec_match);
		}
	}
	CTimeString::sortVector2d(ranking_vecvec, 2);

	for (int j = 0; j < ranking_vecvec.size(); j++)
	{
		frame_vec.push_back(ranking_vecvec[j][0]);
		corr_index_vec.push_back((int)(ranking_vecvec[j][1] / 2));
		if (((int)ranking_vecvec[j][1]) % 2 == 0) b_queryOrNot_vec.push_back(true);
		else b_queryOrNot_vec.push_back(false);
		evaluation_vec.push_back(ranking_vecvec[j][2]);
	}

	if (!b_cout) return;

	for (int j = 0; j < ranking_vecvec.size(); j++)
	{
		if (j % 10 == 0)
		{
			cout << "j:" << j;
			cout << " frame:" << ranking_vecvec[j][0];
			cout << " corr_index:" << ((int)ranking_vecvec[j][1]) / 2;
			cout << " query or match:" << ((int)ranking_vecvec[j][1]) % 2;
			cout << " evaluation:" << ranking_vecvec[j][2] << endl;
		}
	}
	cout << endl;
}

vector<vector<int>> CGlobalFeatureRegistration::calcRanking_compareArg(const vector<vector<pair<float, float>>> &compare_vecvec, bool b_cout)
{
	vector<int> frame_vec;
	vector<int>corr_index_vec;
	vector<bool> b_queryOrNot_vec;
	vector<float> evaluation_vec;
	calcRanking_compareArg_eachValue(compare_vecvec, frame_vec, corr_index_vec, b_queryOrNot_vec, evaluation_vec, b_cout);

	vector<vector<pair<int, int>>> rank_query_match_vecvec;
	for (int j = 0; j < compare_vecvec.size(); j++)
	{
		vector<pair<int, int>> rank_vec;
		for (int i = 0; i < compare_vecvec[j].size(); i++)
			rank_vec.push_back(make_pair(-1, -1));
		rank_query_match_vecvec.push_back(rank_vec);
	}

	{
		float value_before = 0.;
		float value_now = 0.;
		int rank_last;
		for (int j = 0; j < frame_vec.size(); j++)
		{
			value_now = evaluation_vec[j];
			if (j == 0 || value_now != value_before)
				rank_last = j;
			if (b_queryOrNot_vec[j]) rank_query_match_vecvec[frame_vec[j]][corr_index_vec[j]].first = rank_last;
			else rank_query_match_vecvec[frame_vec[j]][corr_index_vec[j]].second = rank_last;
			value_before = value_now;
		}
	}

	//sort
	vector<vector<int>> rank_value_multiple_vecvec_forSort;
	{
		for (int j = 0; j < rank_query_match_vecvec.size(); j++)
		{
			for (int i = 0; i < rank_query_match_vecvec[j].size(); i++)
			{
				vector<int> rank_value_multiple_vec;
				rank_value_multiple_vec.push_back(j);															//frame
				rank_value_multiple_vec.push_back(i);															//index
				rank_value_multiple_vec.push_back(
					(rank_query_match_vecvec[j][i].first + 1) * (rank_query_match_vecvec[j][i].second + 1));	//multiple
				rank_value_multiple_vecvec_forSort.push_back(rank_value_multiple_vec);
			}
		}

	}
	CTimeString::sortVector2d(rank_value_multiple_vecvec_forSort, 2);

	vector<vector<int>> rank_output_vecvec;
	for (int j = 0; j < compare_vecvec.size(); j++)
	{
		vector<int> rank_output_vec;
		for (int i = 0; i < compare_vecvec[j].size(); i++)
			rank_output_vec.push_back(-1);
		rank_output_vecvec.push_back(rank_output_vec);
	}

	{
		float value_before = 0.;
		float value_now = 0.;
		int rank_last;
		for (int j = 0; j < rank_value_multiple_vecvec_forSort.size(); j++)
		{
			value_now = rank_value_multiple_vecvec_forSort[j][2];
			if (j == 0 || value_now != value_before)
				rank_last = j;
			rank_output_vecvec[rank_value_multiple_vecvec_forSort[j][0]][rank_value_multiple_vecvec_forSort[j][1]] = rank_last;
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

