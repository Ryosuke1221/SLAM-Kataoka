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
