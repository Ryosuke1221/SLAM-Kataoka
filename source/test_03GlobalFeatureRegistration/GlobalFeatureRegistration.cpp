#include "GlobalFeatureRegistration.h"

void CGlobalFeatureRegistration::mainProcess()
{
	int WhichProcess = 0;
	string filename1, filename2;
	bool b_finish = false;
	enum OPTION {
		EN_escape = 0,
		EN_FreeSpace,
		EN_FileProcess,
		EN_SequentShow,
		EN_DrawTrajectory,
		EN_DoMappingFromTrajectory,
		EN_SomePointclouds,
		EN_showFeatureValue,
		EN_FPFH_unique,
		EN_RigidTransformation_FPFH_Features_new,
		EN_RigidTransformation_FPFH_Features_allFrames,
		EN_PairEvaluation,
		EN_PairEvaluation2,
		EN_PairEvaluation3
	};

	while (!b_finish)
	{
		cout << endl;
		cout << "please input process number" << endl;
		cout << " " << EN_escape << ": escape" << endl;
		cout << " " << EN_FreeSpace << ": free space" << endl;
		cout << " " << EN_FileProcess << ": FileProcess" << endl;
		cout << " " << EN_SequentShow << ": sequent show" << endl;
		cout << " " << EN_DrawTrajectory << ": DrawTrajectory" << endl;
		cout << " " << EN_DoMappingFromTrajectory << ": DoMappingFromTrajectory" << endl;
		cout << " " << EN_SomePointclouds << ": SomePointclouds" << endl;
		cout << " " << EN_showFeatureValue << ": showFeatureValue" << endl;
		cout << " " << EN_FPFH_unique << ": FPFH_unique" << endl;
		cout << " " << EN_RigidTransformation_FPFH_Features_new << ": RigidTransformation_FPFH_Features_new" << endl;
		cout << " " << EN_RigidTransformation_FPFH_Features_allFrames << ": RigidTransformation_FPFH_Features_allFrames" << endl;
		cout << " " << EN_PairEvaluation << ": PairEvaluation" << endl;
		cout << " " << EN_PairEvaluation2 << ": PairEvaluation2" << endl;
		cout << " " << EN_PairEvaluation3 << ": PairEvaluation3" << endl;

		cout << "WhichProcess: ";
		cin >> WhichProcess;

		cout << endl;
		cout << "//////////////////////////////////" << endl;
		cout << endl;

		string dir_ = "../../data/data_test_03GlobalFeatureRegistration";

		switch (WhichProcess)
		{
		case EN_escape:
			//escape
			b_finish = true;
			break;

		case EN_FreeSpace:
			CGlobalFeatureRegistration::FreeSpace();
			break;

		case EN_FileProcess:
			FileProcess();
			break;

		case EN_SequentShow:
			show_sequent();
			break;

		case EN_DrawTrajectory:
			DrawTrajectory();
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory();
			break;

		case EN_SomePointclouds:
			DoDifferential_SomePointclouds(dir_);
			break;

		case EN_showFeatureValue:
			DoDifferential_showFeatureValue(dir_);
			break;

		case EN_FPFH_unique:
			FPFH_unique(dir_);
			break;

		case EN_RigidTransformation_FPFH_Features_new:
			DoDifferential_RigidTransformation_FPFH_Features_new(dir_);
			break;

		case EN_RigidTransformation_FPFH_Features_allFrames:
			DoDifferential_RigidTransformation_FPFH_Features_allFrames(dir_);
			break;

		case EN_PairEvaluation:
			DoDifferential_PairEvaluation(dir_);
			break;

		case EN_PairEvaluation2:
			DoDifferential_PairEvaluation2(dir_);
			break;

		case EN_PairEvaluation3:
			DoDifferential_PairEvaluation3(dir_);
			break;

		default:
			break;
		}

	}
}

void CGlobalFeatureRegistration::FreeSpace()
{

}

void CGlobalFeatureRegistration::DoDifferential_1pointcloud(string dir_)
{
	typedef typename pcl::PointXYZRGB T_PointType;

	string s_file_0 = "000XYZRGB_naraha.pcd";
	string s_file_1 = "005XYZRGB_naraha.pcd";

	float radius_differential = 0.5;
	//{
	//	cout << "input radius ->";
	//	float radius_temp;
	//	cin >> radius_temp;
	//	if (0 < radius_temp && radius_temp < 3) radius_differential = radius_temp;
	//	cout << "radius_differential:" << radius_differential << endl;
	//}

	pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_colored(new pcl::PointCloud<T_PointType>());
	//pcl::io::loadPCDFile(dir_ + "/" + s_file_0, *cloud_);
	pcl::io::loadPCDFile(dir_ + "/" + s_file_1, *cloud_);
	cloud_->is_dense = true;

	pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	kdtree_->setInputCloud(cloud_);
	vector<float> feature_vec;

	cout << "select feature:  Velodyne:1  NIR:0" << endl;
	bool b_useVelodyneFeature;
	cin >> b_useVelodyneFeature;

	for (int j = 0; j < cloud_->size(); j++)
	{
		if (!b_useVelodyneFeature)
			feature_vec.push_back((float)((int)cloud_->points[j].r));
		else
			feature_vec.push_back((float)((int)cloud_->points[j].g));
	}

	vector<float> featureDivergence_vec;
	featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_, feature_vec, kdtree_, radius_differential, 0.25);

	//for (int j = 0; j < featureDivergence_vec.size(); j++)
	//{
	//	if (j % 100 != 0) continue;
	//	cout << "j:" << j << "  divergence:" << featureDivergence_vec[j] << endl;
	//}
	//cout << endl;

	pcl::copyPointCloud(*cloud_, *cloud_colored);

	vector<int> hist_vec = CTimeString::getHistogram(featureDivergence_vec, 10, true);

	float feature_min = std::numeric_limits<float>::max();
	float feature_max = -std::numeric_limits<float>::max();
	float feature_mean = 0.;
	float feature_median;
	float feature_first_quartile;
	float feature_third_quartile;

	for (int j = 0; j < featureDivergence_vec.size(); j++)
	{
		if (feature_min > featureDivergence_vec[j]) feature_min = featureDivergence_vec[j];
		if (feature_max < featureDivergence_vec[j]) feature_max = featureDivergence_vec[j];
	}
	//calc median and quartile
	{
		vector<float> temp_vec = CTimeString::getMedian_Quartile(featureDivergence_vec);
		feature_first_quartile = temp_vec[0];
		feature_median = temp_vec[1];
		feature_third_quartile = temp_vec[2];
	}
	//calc mean
	{
		for (int j = 0; j < featureDivergence_vec.size(); j++)
			feature_mean += featureDivergence_vec[j];
		if (featureDivergence_vec.size() != 0) feature_mean /= (float)featureDivergence_vec.size();
		else feature_mean = 10000.;
	}

	cout << "feature_min:" << feature_min << endl;
	cout << "feature_max:" << feature_max << endl;
	cout << "feature_mean:" << feature_mean << endl;
	cout << "feature_first_quartile:" << feature_first_quartile << endl;
	cout << "feature_median:" << feature_median << endl;
	cout << "feature_third_quartile:" << feature_third_quartile << endl;


	float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	th_velodyne_min_color = -20.;
	//cout << "input th_velodyne_min_color ->";
	//cin >> th_velodyne_min_color;

	float th_velodyne_max_color;
	//th_velodyne_max_color = 50.;
	//th_velodyne_max_color = 20.;
	//th_velodyne_max_color = 120.;
	th_velodyne_max_color = 80.;
	//cout << "input th_velodyne_max_color ->";
	//cin >> th_velodyne_max_color;

	float th_nir_min_color;


	float th_nir_max_color;
	th_nir_max_color = 100.;

	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
		for (int j = 0; j < s_temp_vecvec.size(); j++)
			cout << "j:" << j << " " << "s_temp_vecvec[j].size():" << s_temp_vecvec[j].size() << endl;
		th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
		th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
		th_nir_max_color = stof(s_temp_vecvec[5][3]);
	}

	for (int j = 0; j < cloud_colored->size(); j++)
	{
		//if (fabs(featureDivergence_vec[j]) > 50.)
		//{
		//	cloud_colored->points[j].r = 0;
		//	cloud_colored->points[j].g = 255;
		//	cloud_colored->points[j].b = 0;
		//}
		//else
		//{
		//	cloud_colored->points[j].r = 255;
		//	cloud_colored->points[j].g = 255;
		//	cloud_colored->points[j].b = 255;
		//}


		vector<std::uint8_t> color_vec;
		//color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[j], feature_max, feature_min);

		if (b_useVelodyneFeature)
			color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[j], th_velodyne_max_color, th_velodyne_min_color);
		else
			color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[j], th_nir_max_color, feature_min);

		cloud_colored->points[j].r = color_vec[0];
		cloud_colored->points[j].g = color_vec[1];
		cloud_colored->points[j].b = color_vec[2];
	}

	for (int j = 0; j < cloud_colored->size(); j++)
	{
		if (fabs(featureDivergence_vec[j]) == 0.)
		{
			cloud_colored->points[j].r = 255;
			cloud_colored->points[j].g = 255;
			cloud_colored->points[j].b = 255;
		}

		if (b_useVelodyneFeature)
		{
			if (th_velodyne_min_color > featureDivergence_vec[j])
			{
				cloud_colored->points[j].r = 255;
				cloud_colored->points[j].g = 255;
				cloud_colored->points[j].b = 0;
			}
			else if (th_velodyne_max_color < featureDivergence_vec[j])
			{
				cloud_colored->points[j].r = 0;
				cloud_colored->points[j].g = 0;
				cloud_colored->points[j].b = 255;
			}
		}
		else
		{
			//th_velodyne_max_color
			if (th_nir_max_color < featureDivergence_vec[j])
			{
				cloud_colored->points[j].r = 255;
				cloud_colored->points[j].g = 130;
				cloud_colored->points[j].b = 0;
			}

		}
	}

	Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
		calcHomogeneousMatrixFromVector6d(40., 0., 0., 0., 0., 0.));
	pcl::transformPointCloud(*cloud_colored, *cloud_colored, trans_);
	*cloud_colored += *cloud_;
	//pcl::io::savePCDFile<T_PointType>(dir_save + "/cloud_colored.pcd", *cloud_colored);

	typedef typename CPointVisualization<T_PointType> CPV;

	CPV pv;
	pv.setWindowName("differential");

	while (1)
	{
		pv.setPointCloud(cloud_colored);
		pv.updateViewer();
		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
	}

	pv.closeViewer();
}

void CGlobalFeatureRegistration::DoDifferential_SomePointclouds(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	cout << "select feature:  NIR:0   Velodyne:1" << endl;
	bool b_useVelodyneFeature;
	cin >> b_useVelodyneFeature;

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	float th_velodyne_min_color;
	th_velodyne_min_color = -4.;
	float th_velodyne_max_color;
	th_velodyne_max_color = 3.6;
	float th_nir_min_color;
	th_nir_min_color = -20;
	float th_nir_max_color;
	th_nir_max_color = 70.;

	float sigma_weight;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	{
		vector<vector<string>> s_temp_vecvec;
		s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
		radius_differential = stof(s_temp_vecvec[1][3]);
		th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
		th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
		th_nir_min_color = stof(s_temp_vecvec[4][3]);
		th_nir_max_color = stof(s_temp_vecvec[5][3]);
		sigma_weight = stof(s_temp_vecvec[6][3]);
	}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;

	//feature
	vector<vector<float>> feature_vecvec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		vector<float> feature_vec;
		for (int i = 0; i < cloud_vec[j]->size(); i++)
		{
			if (!b_useVelodyneFeature)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			else
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
		}
		feature_vecvec.push_back(feature_vec);
	}

	//calc
	vector<vector<float>> featureDivergence_vecvec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		//cout << "j:" << j << endl;
		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
		kdtree_->setInputCloud(cloud_vec[j]);
		vector<float> featureDivergence_vec;
		featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec[j], kdtree_, radius_differential, sigma_weight, true);
		featureDivergence_vecvec.push_back(featureDivergence_vec);
	}

	float value_max_all = -std::numeric_limits<float>::max();
	float value_min_all = std::numeric_limits<float>::max();
	for (int j = 0; j < featureDivergence_vecvec.size(); j++)
	{
		for (int i = 0; i < featureDivergence_vecvec[j].size(); i++)
		{
			if (value_max_all < featureDivergence_vecvec[j][i]) value_max_all = featureDivergence_vecvec[j][i];
			if (value_min_all > featureDivergence_vecvec[j][i]) value_min_all = featureDivergence_vecvec[j][i];
		}
	}

	vector<vector<int>> index_valid_vecvec;
	index_valid_vecvec = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec, num_bin_hist, true);

	////botsu?
	//vector<pair<int, int>> corr_vec = CExtendableICP::determineCorrespondences_feature_histogram(featureDivergence_vecvec[1],
	//	featureDivergence_vecvec[0], num_bin_hist, value_max_hist, value_min_hist, range_hist, true);
	//cout << "corr_vec.size():" << corr_vec.size() << endl;

	pcl::Correspondences corr_new;
	vector<pcl::Correspondences> corr_new_vec;
	{
		int i_src = 1;
		int i_tgt = 0;
		int num_nearest = 10;
		//corr_new = CExtendableICP::determineCorrespondences_feature(featureDivergence_vecvec[i_src], featureDivergence_vecvec[i_tgt], num_nearest);
		corr_new = CExtendableICP::determineCorrespondences_featureScalar_num_remove(featureDivergence_vecvec[i_src], featureDivergence_vecvec[i_tgt],
			index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], num_nearest);
		cout << "corr_new.size():" << corr_new.size() << endl;
		corr_new_vec = CExtendableICP::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corr_new, 0.9);
		for (int j = 0; j < corr_new_vec.size(); j++)
			cout << "j:" << j << " corr_new_vec[j].size():" << corr_new_vec[j].size() << endl;
	}

	//show corr
	{
		//showing
		CPointVisualization<T_PointType> pv;
		pv.setWindowName("Pairs");
		pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[1], *cloud_src);
		pcl::copyPointCloud(*cloud_vec[0], *cloud_tgt);
		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
		}
		*cloud_show += *cloud_src;
		*cloud_show += *cloud_tgt;
		pv.setPointCloud(cloud_show);
		vector<std::uint8_t> color_vec;
		color_vec.push_back(100);
		color_vec.push_back(100);
		color_vec.push_back(100);
		cout << "corr_new.size():" << corr_new.size() << endl;
		//for (int j = corr_new.size() - 1; j >= 0; j--)
		//	if (j > 100) corr_new.erase(corr_new.begin() + j);
		cout << "corr_new.size():" << corr_new.size() << endl;
		int index_corr = 0;
		while (1)
		{
			if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_corr < corr_new_vec.size())
			{
				pcl::Correspondences corr_show;
				pv.drawCorrespondance(cloud_src, cloud_tgt, corr_new_vec[index_corr], color_vec);
				index_corr++;
			}

			if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
			pv.updateViewer();
		}
		pv.closeViewer();
	}

	//give color to pointcloud
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_colored(new pcl::PointCloud<T_PointType>());
		//pcl::copyPointCloud(*cloud_vec[j], *cloud_colored);
		for (int i = 0; i < index_valid_vecvec[j].size(); i++)
			cloud_colored->push_back(cloud_vec[j]->points[index_valid_vecvec[j][i]]);
		pcl::PointCloud<T_PointType>::Ptr cloud_ZValue(new pcl::PointCloud<T_PointType>());
		vector<float> feature_vec;
		for (int i = 0; i < index_valid_vecvec[j].size(); i++)
			feature_vec.push_back(featureDivergence_vecvec[j][index_valid_vecvec[j][i]]);

		for (int i = 0; i < index_valid_vecvec[j].size(); i++)
		{
			vector<std::uint8_t> color_vec;
			//color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyHSV(featureDivergence_vec[i], feature_max, feature_min);

			//if (b_useVelodyneFeature)
			//	color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], th_velodyne_max_color, th_velodyne_min_color);
			//else
			//	color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], th_nir_max_color, th_nir_min_color);
			if (b_useVelodyneFeature)
				color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], value_max_all, value_min_all);
			else
				color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vec[i], value_max_all, value_min_all);

			cloud_colored->points[i].r = color_vec[0];
			cloud_colored->points[i].g = color_vec[1];
			cloud_colored->points[i].b = color_vec[2];

			if (fabs(feature_vec[i]) == 0.)
			{
				cloud_colored->points[i].r = 255;
				cloud_colored->points[i].g = 255;
				cloud_colored->points[i].b = 255;
			}

			//if (b_useVelodyneFeature)
			//{
			//	if (th_velodyne_min_color > feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 100;
			//		cloud_colored->points[i].g = 100;
			//		cloud_colored->points[i].b = 100;
			//	}
			//	else if (th_velodyne_max_color < feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 255;
			//		cloud_colored->points[i].g = 255;
			//		cloud_colored->points[i].b = 0;
			//	}
			//}
			//else
			//{
			//	if (th_nir_min_color > feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 100;
			//		cloud_colored->points[i].g = 100;
			//		cloud_colored->points[i].b = 100;
			//	}
			//	else if (th_nir_max_color < feature_vec[i])
			//	{
			//		cloud_colored->points[i].r = 255;
			//		cloud_colored->points[i].g = 255;
			//		cloud_colored->points[i].b = 0;
			//	}
			//}
		}

		//give z by features
		pcl::copyPointCloud(*cloud_colored, *cloud_ZValue);
		*cloud_ZValue = *CExtendableICP::getPointCloud_ZAaxisByFeature(cloud_ZValue, feature_vec, 10.);

		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(40., 0., 0., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_colored, *cloud_colored, trans_);
			*cloud_vec[j] += *cloud_colored;
		}

		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(40., 0., 15., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_ZValue, *cloud_ZValue, trans_);
			*cloud_vec[j] += *cloud_ZValue;
		}
	}

	//showing
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("Differential");

	int index_cloud = 0;
	while (1)
	{

		if (((GetAsyncKeyState(VK_SPACE) & 1) == 1) && (index_cloud != cloud_vec.size()))
		{
			pv.setPointCloud(cloud_vec[index_cloud], filenames_cloud[index_cloud]);
			cout << "index:" << index_cloud << endl;
			index_cloud++;
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;

		pv.updateViewer();
	}
	pv.closeViewer();

}

void CGlobalFeatureRegistration::DoDifferential_showFeatureValue(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	cout << "select feature:  NIR:0   Velodyne:1" << endl;
	bool b_useVelodyneFeature;
	cin >> b_useVelodyneFeature;

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	//remove 255 in nir
	if (!b_useVelodyneFeature)
	{
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			for (int i = cloud_vec[j]->size() - 1; i >= 0; i--)
			{
				if (cloud_vec[j]->points[i].r == 255)
					cloud_vec[j]->points.erase(cloud_vec[j]->points.begin() + i);
			}
		}
	}

	//feature
	vector<vector<float>> feature_vecvec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		vector<float> feature_vec;
		for (int i = 0; i < cloud_vec[j]->size(); i++)
		{
			if (!b_useVelodyneFeature)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			else
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
		}
		feature_vecvec.push_back(feature_vec);
	}

	//calc
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		cout << "j:" << j << endl;
		vector<float> feature_calcHistogram;
		for (int i = 0; i < feature_vecvec[j].size(); i++)
			feature_calcHistogram.push_back(feature_vecvec[j][i]);
		vector<int> hist_vec = CTimeString::getHistogram(feature_calcHistogram, 80, true);
	}

	float feature_all_min = std::numeric_limits<float>::max();
	float feature_all_max = -std::numeric_limits<float>::max();
	{
		cout << "show histogram of all features" << endl;
		vector<float> features_all;
		for (int j = 0; j < feature_vecvec.size(); j++)
			features_all.insert(features_all.end(), feature_vecvec[j].begin(), feature_vecvec[j].end());
		vector<float> feature_calcHistogram;

		for (int j = 0; j < features_all.size(); j++)
		{
			feature_calcHistogram.push_back(features_all[j]);
			if (feature_all_min > features_all[j]) feature_all_min = features_all[j];
			if (feature_all_max < features_all[j]) feature_all_max = features_all[j];

		}
		vector<int> hist_vec = CTimeString::getHistogram(feature_calcHistogram, 80, true);
	}

	//give color (and z by features) to pointcloud
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_colored(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[j], *cloud_colored);

		pcl::PointCloud<T_PointType>::Ptr cloud_ZValue(new pcl::PointCloud<T_PointType>());

		for (int i = 0; i < cloud_vec[j]->size(); i++)
		{
			vector<std::uint8_t> color_vec;
			color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vecvec[j][i], feature_all_max, feature_all_min);
			cloud_colored->points[i].r = color_vec[0];
			cloud_colored->points[i].g = color_vec[1];
			cloud_colored->points[i].b = color_vec[2];
		}

		//give z by features
		pcl::copyPointCloud(*cloud_colored, *cloud_ZValue);
		*cloud_ZValue = *CExtendableICP::getPointCloud_ZAaxisByFeature(cloud_ZValue, feature_vecvec[j], 3.);

		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(40., 0., 0., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_colored, *cloud_colored, trans_);
			*cloud_vec[j] += *cloud_colored;

		}

		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(40., 0., 15., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_ZValue, *cloud_ZValue, trans_);
			*cloud_vec[j] += *cloud_ZValue;
		}

	}

	//showing
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("differential");

	int index_cloud = 0;
	while (1)
	{

		if (((GetAsyncKeyState(VK_SPACE) & 1) == 1) && (index_cloud != cloud_vec.size()))
		{
			pv.setPointCloud(cloud_vec[index_cloud], filenames_cloud[index_cloud]);
			cout << "index:" << index_cloud << endl;
			index_cloud++;

		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;

		pv.updateViewer();
	}
	pv.closeViewer();

}

void CGlobalFeatureRegistration::FPFH_unique(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	//debug
	//for (int j = cloud_vec.size() - 1; j >=1 ; j--)
	//	cloud_vec.erase(cloud_vec.begin() + j);

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;

	//calc normal
	//0.5
	//float voxel_size;
	//voxel_size = 0.1;

	float radius_normal_FPFH;
	radius_normal_FPFH = 0.5;

	const pcl::search::KdTree<T_PointType>::Ptr kdtree_(new pcl::search::KdTree<T_PointType>);
	const auto view_point = T_PointType(0.0, 10.0, 10.0);
	vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
	for (int j = 0; j < cloud_vec.size(); j++)
	{
		const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
		ne->setInputCloud(cloud_vec[j]);
		ne->setRadiusSearch(radius_normal_FPFH);
		ne->setSearchMethod(kdtree_);
		ne->setViewPoint(view_point.x, view_point.y, view_point.z);
		ne->compute(*normals);
		normals_vec.push_back(normals);
	}

	vector<vector<int>> index_vecvec;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	float radius_FPFH_center = 1.;
	index_vecvec = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
	for (int j = 0; j < index_vecvec.size(); j++)
	{
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_temp(new pcl::PointCloud<pcl::FPFHSignature33>);
		fpfh_temp = CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center);
		vector<bool> b_valid_vec;
		b_valid_vec.resize(cloud_vec[j]->size());
		fill(b_valid_vec.begin(), b_valid_vec.end(), false);
		for (int i = 0; i < index_vecvec[j].size(); i++)
			b_valid_vec[index_vecvec[j][i]] = true;
		for (int i = index_vecvec[j].size() - 1; i >= 0; i--)
			if (!b_valid_vec[i]) fpfh_temp->erase(fpfh_temp->begin() + i);
		fpfh_vec.push_back(fpfh_temp);
	}

	pcl::Correspondences corrs_;
	{
		int i_tgt = 0;
		int i_src = 1;
		int num_near = 10;
		corrs_ = CFPFH_PCL::determineCorrespondences_featureFpfh_eachPairHaving_num_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
			index_vecvec[i_src], index_vecvec[i_tgt], num_near);

		for (int j = 0; j < corrs_.size(); j++)
		{
			if (j % 1000 != 0) continue;
			cout << "j:" << j << "  query:" << corrs_[j].index_query << " match:" << corrs_[j].index_match << " distance:" << corrs_[j].distance << endl;
		}

	}
}

void CGlobalFeatureRegistration::DoDifferential_RigidTransformation_FPFH_Features(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	{
		vector<vector<float>> feature_vecvec_nir;
		vector<vector<float>> feature_vecvec_velodyne;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			feature_vecvec_nir.push_back(feature_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
			feature_vecvec_velodyne.push_back(feature_vec);
		}
		//calc Laplacian
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
			kdtree_->setInputCloud(cloud_vec[j]);
			vector<float> featureDivergence_vec;
			featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
			kdtree_->setInputCloud(cloud_vec[j]);
			vector<float> featureDivergence_vec;
			featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
		}
		bool b_showHistogram = false;
		b_showHistogram = true;
		index_valid_vecvec_nir = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	}


	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	string ts_start = CTimeString::getTimeString();
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);
		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));

	}
	string ts_end = CTimeString::getTimeString();
	cout << "elapsed time:" << CTimeString::getTimeElapsefrom2Strings(ts_start, ts_end) << endl;

	pcl::Correspondences corrs_nir;
	pcl::Correspondences corrs_velodyne;
	pcl::Correspondences corrs_fpfh;

	int i_tgt;
	int i_src;
	i_tgt = 0;
	i_src = 1;
	//i_tgt = 5;
	//i_src = 6;
	int num_nearest = 10;
	float th_nearest_nir = 0.5;
	float th_nearest_velodyne = 0.05;
	float th_nearest_fpfh = 1800.;

	//corrs_nir = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
	//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
	corrs_nir = CExtendableICP::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

	//corrs_velodyne = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
	//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
	corrs_velodyne = CExtendableICP::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

	//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
	corrs_fpfh = CFPFH_PCL::determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
		index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

	vector<pcl::Correspondences> corrs_vec;
	{
		pcl::Correspondences corrs_temp;
		corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
		corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
		corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
		corrs_vec = CExtendableICP::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
		//for (int j = 0; j < corrs_vec.size(); j++)
		//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	}

	//select biggest 10 corr
	{
		int num_select = 10;
		vector<vector<int>> index_biggest_vecvec;
		for (int j = 0; j < num_select; j++)
		{
			vector<int> index_biggest_vec;
			index_biggest_vec.push_back(0);
			index_biggest_vec.push_back(0);
			index_biggest_vecvec.push_back(index_biggest_vec);
		}
		for (int j = 0; j < corrs_vec.size(); j++)
		{
			vector<int> index_biggest_vec;
			index_biggest_vec.push_back(j);
			index_biggest_vec.push_back(corrs_vec[j].size());
			index_biggest_vecvec.push_back(index_biggest_vec);
			CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
			index_biggest_vecvec.pop_back();
		}
		vector<pcl::Correspondences> corrs_vec_new;
		for (int j = 0; j < index_biggest_vecvec.size(); j++)
			corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
		corrs_vec = corrs_vec_new;
		for (int j = 0; j < corrs_vec.size(); j++)
			cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	}

	//show corr
	{
		//showing
		CPointVisualization<T_PointType> pv;
		pv.setWindowName("Pairs");
		pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
		pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);
		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
		}
		*cloud_show += *cloud_src;
		*cloud_show += *cloud_tgt;
		pv.setPointCloud(cloud_show);
		vector<std::uint8_t> color_vec;
		color_vec.push_back(100);
		color_vec.push_back(100);
		color_vec.push_back(100);
		int index_corr = 0;
		while (1)
		{
			if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_corr < corrs_vec.size())
			{
				cout << "show corrs_vec[" << index_corr << "]" << endl;
				pcl::Correspondences corr_show;
				pv.drawCorrespondance(cloud_src, cloud_tgt, corrs_vec[index_corr], color_vec);
				for (int i = 0; i < corrs_vec[index_corr].size(); i++)
					cout << "i:" << i << " distance:" << corrs_vec[index_corr][i].distance << endl;
				cout << endl;
				index_corr++;
			}

			if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
			pv.updateViewer();
		}
		pv.closeViewer();
	}

	////transformation
	//for (int j = 0; j < corrs_vec.size(); j++)
	//{
	//	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	//	CExtendableICP::estimateRigidTransformation_static(cloud_vec[i_src], cloud_vec[i_tgt], corrs_vec[j], transformation_matrix);

	//}

}

void CGlobalFeatureRegistration::DoDifferential_RigidTransformation_FPFH_Features_new(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	//{
	//	vector<vector<float>> feature_vecvec_nir;
	//	vector<vector<float>> feature_vecvec_velodyne;
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
	//		feature_vecvec_nir.push_back(feature_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
	//		feature_vecvec_velodyne.push_back(feature_vec);
	//	}
	//	//calc Laplacian
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
	//	}
	//	bool b_showHistogram = false;
	//	b_showHistogram = true;
	//	index_valid_vecvec_nir = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
	//	index_valid_vecvec_velodyne = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	//}


	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	//vector<Eigen::Matrix4f> transformation_vec;
	vector<pcl::Correspondences> corr_biggest_vec;
	vector<pair<int, int>> index_pair_vec;

	//calc
	{
		int i_tgt = 0;
		int i_src = 1;

		int num_nearest = 10;
		float th_nearest_nir = 0.5;
		float th_nearest_velodyne = 0.05;
		float th_nearest_fpfh = 1800.;
		index_pair_vec.push_back(make_pair(i_tgt, i_src));
		cout << "i_tgt:" << i_tgt << endl;
		cout << "i_src:" << i_src << endl;
		//cout << "fpfh_vec[i_src]->size():" << fpfh_vec[i_src]->size() << endl;
		//cout << "fpfh_vec[i_tgt]->size():" << fpfh_vec[i_tgt]->size() << endl;
		//{
		//	int i_max = 0;
		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_src].size(); j++)
		//		if (i_max < index_valid_vecvec_FPFH[i_src][j]) i_max = index_valid_vecvec_FPFH[i_src][j];
		//	cout << "i_src max:" << i_max << endl;
		//}
		//{
		//	int i_max = 0;
		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_tgt].size(); j++)
		//		if (i_max < index_valid_vecvec_FPFH[i_tgt][j]) i_max = index_valid_vecvec_FPFH[i_tgt][j];
		//	cout << "i_tgt max:" << i_max << endl;
		//}

		pcl::Correspondences corrs_nir;
		pcl::Correspondences corrs_velodyne;
		pcl::Correspondences corrs_fpfh;

		//corrs_nir = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
		//corrs_nir = CExtendableICP::determineCorrespondences_feature_value_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

		//corrs_velodyne = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
		//corrs_velodyne = CExtendableICP::determineCorrespondences_feature_value_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
		corrs_fpfh = CFPFH_PCL::determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
			index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

		vector<pcl::Correspondences> corrs_vec;
		{
			pcl::Correspondences corrs_temp;
			//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
			//corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
			corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
			corrs_vec = CExtendableICP::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
			//for (int j = 0; j < corrs_vec.size(); j++)
			//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
		}

		//select biggest 10 corr
		{
			int num_select = 10;
			vector<vector<int>> index_biggest_vecvec;
			for (int j = 0; j < num_select; j++)
			{
				vector<int> index_biggest_vec;
				index_biggest_vec.push_back(0);
				index_biggest_vec.push_back(0);
				index_biggest_vecvec.push_back(index_biggest_vec);
			}
			for (int j = 0; j < corrs_vec.size(); j++)
			{
				vector<int> index_biggest_vec;
				index_biggest_vec.push_back(j);
				index_biggest_vec.push_back(corrs_vec[j].size());
				index_biggest_vecvec.push_back(index_biggest_vec);
				CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
				index_biggest_vecvec.pop_back();
			}
			//if (corrs_vec.size() == 0) continue;
			vector<pcl::Correspondences> corrs_vec_new;
			for (int j = 0; j < index_biggest_vecvec.size(); j++)
				corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
			corrs_vec = corrs_vec_new;
			for (int j = 0; j < corrs_vec.size(); j++)
				cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
			corr_biggest_vec.push_back(corrs_vec[0]);
		}

	}

	cout << "show" << endl;

	//vector<pcl::Correspondences> corr_biggest_vec;
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("Pairs");
	pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

	vector<std::uint8_t> color_corr_vec;
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);

	int index_framePair = 0;
	while (1)
	{
		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corr_biggest_vec.size())
		{
			int i_tgt = index_pair_vec[index_framePair].first;
			int i_src = index_pair_vec[index_framePair].second;
			pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
			pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

			//showing
			{
				Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
					calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
				pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
			}
			*cloud_show += *cloud_src;
			*cloud_show += *cloud_tgt;
			pv.setPointCloud(cloud_show);
			vector<std::uint8_t> color_corr_vec;
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);

			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			pcl::Correspondences corr_show;
			pv.drawCorrespondance(cloud_src, cloud_tgt, corr_biggest_vec[index_framePair], color_corr_vec);
			for (int i = 0; i < corr_biggest_vec[index_framePair].size(); i++)
				cout << "i:" << i << " distance:" << corr_biggest_vec[index_framePair][i].distance << endl;
			cout << endl;
			index_framePair++;
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
		pv.updateViewer();

	}
	pv.closeViewer();

}

void CGlobalFeatureRegistration::DoDifferential_RigidTransformation_FPFH_Features_allFrames(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	//{
	//	vector<vector<float>> feature_vecvec_nir;
	//	vector<vector<float>> feature_vecvec_velodyne;
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
	//		feature_vecvec_nir.push_back(feature_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
	//		feature_vecvec_velodyne.push_back(feature_vec);
	//	}
	//	//calc Laplacian
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
	//	}
	//	for (int j = 0; j < cloud_vec.size(); j++)
	//	{
	//		pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
	//		kdtree_->setInputCloud(cloud_vec[j]);
	//		vector<float> featureDivergence_vec;
	//		featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
	//		featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
	//	}
	//	bool b_showHistogram = false;
	//	b_showHistogram = true;
	//	index_valid_vecvec_nir = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
	//	index_valid_vecvec_velodyne = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	//}


	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	//vector<Eigen::Matrix4f> transformation_vec;
	vector<pcl::Correspondences> corr_biggest_vec;
	vector<pair<int, int>> index_pair_vec;
	for (int i_tgt = 0; i_tgt < cloud_vec.size(); i_tgt++)
	{
		for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
		{
			int num_nearest = 10;
			index_pair_vec.push_back(make_pair(i_tgt, i_src));
			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			//cout << "fpfh_vec[i_src]->size():" << fpfh_vec[i_src]->size() << endl;
			//cout << "fpfh_vec[i_tgt]->size():" << fpfh_vec[i_tgt]->size() << endl;
			//{
			//	int i_max = 0;
			//	for (int j = 0; j < index_valid_vecvec_FPFH[i_src].size(); j++)
			//		if (i_max < index_valid_vecvec_FPFH[i_src][j]) i_max = index_valid_vecvec_FPFH[i_src][j];
			//	cout << "i_src max:" << i_max << endl;
			//}
			//{
			//	int i_max = 0;
			//	for (int j = 0; j < index_valid_vecvec_FPFH[i_tgt].size(); j++)
			//		if (i_max < index_valid_vecvec_FPFH[i_tgt][j]) i_max = index_valid_vecvec_FPFH[i_tgt][j];
			//	cout << "i_tgt max:" << i_max << endl;
			//}

			pcl::Correspondences corrs_nir;
			pcl::Correspondences corrs_velodyne;
			pcl::Correspondences corrs_fpfh;

			//corrs_nir = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
			//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
			//corrs_velodyne = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
			//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
			corrs_fpfh = CFPFH_PCL::determineCorrespondences_featureFpfh_eachPairHaving_num_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
				index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);

			vector<pcl::Correspondences> corrs_vec;
			{
				pcl::Correspondences corrs_temp;
				//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
				//corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
				corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
				corrs_vec = CExtendableICP::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
				//for (int j = 0; j < corrs_vec.size(); j++)
				//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
			}

			//select biggest 10 corr
			{
				int num_select = 10;
				vector<vector<int>> index_biggest_vecvec;
				for (int j = 0; j < num_select; j++)
				{
					vector<int> index_biggest_vec;
					index_biggest_vec.push_back(0);
					index_biggest_vec.push_back(0);
					index_biggest_vecvec.push_back(index_biggest_vec);
				}
				for (int j = 0; j < corrs_vec.size(); j++)
				{
					vector<int> index_biggest_vec;
					index_biggest_vec.push_back(j);
					index_biggest_vec.push_back(corrs_vec[j].size());
					index_biggest_vecvec.push_back(index_biggest_vec);
					CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
					index_biggest_vecvec.pop_back();
				}
				if (corrs_vec.size() == 0) continue;
				vector<pcl::Correspondences> corrs_vec_new;
				for (int j = 0; j < index_biggest_vecvec.size(); j++)
					corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
				corrs_vec = corrs_vec_new;
				for (int j = 0; j < corrs_vec.size(); j++)
					cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
				corr_biggest_vec.push_back(corrs_vec[0]);
			}

		}
	}

	cout << "show" << endl;

	//vector<pcl::Correspondences> corr_biggest_vec;
	//vector<pair<int, int>> index_pair_vec;
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("Pairs");
	pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

	vector<std::uint8_t> color_corr_vec;
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);

	int index_framePair = 0;
	while (1)
	{
		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corr_biggest_vec.size())
		{
			int i_tgt = index_pair_vec[index_framePair].first;
			int i_src = index_pair_vec[index_framePair].second;
			pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
			pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

			//showing
			{
				Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
					calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
				pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
			}
			*cloud_show += *cloud_src;
			*cloud_show += *cloud_tgt;
			pv.setPointCloud(cloud_show);
			vector<std::uint8_t> color_corr_vec;
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);

			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			pcl::Correspondences corr_show;
			pv.drawCorrespondance(cloud_src, cloud_tgt, corr_biggest_vec[index_framePair], color_corr_vec);
			for (int i = 0; i < corr_biggest_vec[index_framePair].size(); i++)
				cout << "i:" << i << " distance:" << corr_biggest_vec[index_framePair][i].distance << endl;
			cout << endl;
			index_framePair++;
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
		pv.updateViewer();

	}
	pv.closeViewer();

}

void CGlobalFeatureRegistration::DoDifferential_PairEvaluation(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	bool b_useHistogramRemover = false;
	bool b_useDivergence = false;
	bool b_useCorrCompare = false;
	bool b_useGeometricConstraints = false;

	b_useHistogramRemover = true;
	b_useDivergence = true;

	b_useCorrCompare = true;
	b_useGeometricConstraints = true;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		cout << "input folder you want to calc ->";
		int i_folder;
		cin >> i_folder;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	{
		vector<vector<float>> feature_vecvec_nir;
		vector<vector<float>> feature_vecvec_velodyne;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			feature_vecvec_nir.push_back(feature_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
			feature_vecvec_velodyne.push_back(feature_vec);
		}
		//calc Laplacian
		if (b_useDivergence)
		{
			cout << "calc Divergence" << endl;
			for (int j = 0; j < cloud_vec.size(); j++)
			{
				pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
				kdtree_->setInputCloud(cloud_vec[j]);
				vector<float> featureDivergence_vec;
				featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
				featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
			}
			for (int j = 0; j < cloud_vec.size(); j++)
			{
				pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
				kdtree_->setInputCloud(cloud_vec[j]);
				vector<float> featureDivergence_vec;
				featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
				featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
			}
		}
		else
		{
			featureDivergence_vecvec_nir = feature_vecvec_nir;
			featureDivergence_vecvec_velodyne = feature_vecvec_velodyne;
		}
	}

	bool b_showHistogram = false;
	//b_showHistogram = true;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	for (int j = 0; j < featureDivergence_vecvec_nir.size(); j++)
	{
		vector<int> featureDivergence_vec;
		for (int i = 0; i < featureDivergence_vecvec_nir[j].size(); i++)
		{
			featureDivergence_vec.push_back(i);
		}
		index_valid_vecvec_nir.push_back(featureDivergence_vec);
	}
	for (int j = 0; j < featureDivergence_vecvec_velodyne.size(); j++)
	{
		vector<int> featureDivergence_vec;
		for (int i = 0; i < featureDivergence_vecvec_velodyne[j].size(); i++)
		{
			featureDivergence_vec.push_back(i);
		}
		index_valid_vecvec_velodyne.push_back(featureDivergence_vec);
	}
	if (b_useHistogramRemover)
	{
		cout << "calc HistogramRemover" << endl;
		index_valid_vecvec_nir = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	}

	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	//vector<Eigen::Matrix4f> transformation_vec;
	//vector<pcl::Correspondences> corr_biggest_vec;
	vector<pair<int, int>> index_pair_vec;

	vector<pcl::Correspondences> corrs_nir_vec;
	vector<pcl::Correspondences> corrs_velodyne_vec;
	vector<pcl::Correspondences> corrs_fpfh_vec;

	float th_nearest_nir;
	float th_nearest_velodyne;
	float th_nearest_fpfh = 1800.;

	if (b_useDivergence)
	{
		th_nearest_nir = 0.5;
		th_nearest_velodyne = 0.05;

	}
	else
	{
		th_nearest_nir = 0.5;
		th_nearest_velodyne = 0.05;
	}

	{
		cout << "calc corr" << endl;
		//int i_tgt = 0;
		int i_tgt = 5;
		int i_src = i_tgt + 1;

		int num_nearest = 10;
		index_pair_vec.push_back(make_pair(i_tgt, i_src));
		cout << "i_tgt:" << i_tgt << endl;
		cout << "i_src:" << i_src << endl;

		pcl::Correspondences corrs_nir;
		pcl::Correspondences corrs_velodyne;
		pcl::Correspondences corrs_fpfh;

		//corrs_nir = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		//	index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], num_nearest);
		corrs_nir = CExtendableICP::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
			index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

		//corrs_velodyne = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
		corrs_velodyne = CExtendableICP::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
			index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
		corrs_fpfh = CFPFH_PCL::determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
			index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

		corrs_nir_vec.push_back(corrs_nir);
		corrs_velodyne_vec.push_back(corrs_velodyne);
		corrs_fpfh_vec.push_back(corrs_fpfh);
	}

	if (b_useCorrCompare)
	{
		cout << "calc corr compare" << endl;
		cout << "nir" << endl;
		{
			vector<vector<int>> rank_nir_vecvec;
			rank_nir_vecvec = CExtendableICP::calcRanking_featureScalar(index_pair_vec, corrs_nir_vec, cloud_vec, featureDivergence_vecvec_nir, index_valid_vecvec_nir, th_nearest_nir, true);
			vector<pcl::Correspondences> corrs_vec_temp;
			for (int j = 0; j < rank_nir_vecvec.size(); j++)
			{
				pcl::Correspondences corrs_temp;
				cout << "rank_nir_vecvec.size():" << rank_nir_vecvec.size() << endl;
				for (int i = 0; i < rank_nir_vecvec[j].size(); i++)
				{
					cout << "rank_nir_vecvec[j].size():" << rank_nir_vecvec[j].size() << endl;
					if (rank_nir_vecvec[j][i] == 0) corrs_temp.push_back(corrs_nir_vec[j][i]);
					corrs_vec_temp.push_back(corrs_temp);
				}
			}
			corrs_nir_vec.clear();
			corrs_nir_vec = corrs_vec_temp;
			cout << "corrs_nir_vec.back().size():" << corrs_nir_vec.back().size() << endl;
		}

		cout << "velodyne" << endl;
		{
			vector<vector<int>> rank_velodyne_vecvec;
			rank_velodyne_vecvec = CExtendableICP::calcRanking_featureScalar(index_pair_vec, corrs_velodyne_vec, cloud_vec, featureDivergence_vecvec_velodyne, index_valid_vecvec_velodyne, th_nearest_velodyne, true);
			vector<pcl::Correspondences> corrs_vec_temp;
			for (int j = 0; j < rank_velodyne_vecvec.size(); j++)
			{
				pcl::Correspondences corrs_temp;
				for (int i = 0; i < rank_velodyne_vecvec[j].size(); i++)
				{
					if (rank_velodyne_vecvec[j][i] == 0) corrs_temp.push_back(corrs_velodyne_vec[j][i]);
					corrs_vec_temp.push_back(corrs_temp);
				}
			}
			corrs_velodyne_vec.clear();
			corrs_velodyne_vec = corrs_vec_temp;
			cout << "corrs_velodyne_vec.back().size():" << corrs_velodyne_vec.back().size() << endl;
		}
	}

	//sum all features
	//vector<pcl::Correspondences> corrs_nir_vec;
	//vector<pair<int, int>> index_pair_vec;
	vector<vector<pcl::Correspondences>> corrs_all_vecvec;
	for (int j = 0; j < index_pair_vec.size(); j++)
	{
		int i_tgt = index_pair_vec[j].first;
		int i_src = index_pair_vec[j].second;
		pcl::Correspondences corrs_temp;
		corrs_temp.insert(corrs_temp.end(), corrs_nir_vec[j].begin(), corrs_nir_vec[j].end());
		corrs_temp.insert(corrs_temp.end(), corrs_velodyne_vec[j].begin(), corrs_velodyne_vec[j].end());
		corrs_temp.insert(corrs_temp.end(), corrs_fpfh_vec[j].begin(), corrs_fpfh_vec[j].end());

		if (b_useGeometricConstraints)
		{
			cout << "calc GeometricConstraints" << endl;
			corrs_all_vecvec.push_back(CExtendableICP::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9));
		}
		else
		{
			vector<pcl::Correspondences> corrs_temp_vec;
			corrs_temp_vec.push_back(corrs_temp);
			corrs_all_vecvec.push_back(corrs_temp_vec);
		}
		//for (int j = 0; j < corrs_vec.size(); j++)
		//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	}

	int num_select = 10;
	cout << "biggest corrs  num_select:" << num_select << endl;
	if (b_useGeometricConstraints)
		for (int j = 0; j < corrs_all_vecvec.size(); j++)
		{
			cout << "frame:" << j << endl;
			vector<vector<int>> index_biggest_vecvec;
			for (int i = 0; i < num_select; i++)
			{
				vector<int> index_biggest_vec;
				index_biggest_vec.push_back(0);
				index_biggest_vec.push_back(0);
				index_biggest_vecvec.push_back(index_biggest_vec);
			}

			for (int i = 0; i < corrs_all_vecvec[j].size(); i++)
			{
				vector<int> index_biggest_vec;
				index_biggest_vec.push_back(i);
				index_biggest_vec.push_back(corrs_all_vecvec[j][i].size());
				index_biggest_vecvec.push_back(index_biggest_vec);
				CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
				index_biggest_vecvec.pop_back();
			}

			vector<pcl::Correspondences> corrs_vec_new;
			for (int i = 0; i < index_biggest_vecvec.size(); i++)
				corrs_vec_new.push_back(corrs_all_vecvec[j][index_biggest_vecvec[i][0]]);
			corrs_all_vecvec[j] = corrs_vec_new;
			for (int i = 0; i < corrs_vec_new.size(); i++)
				cout << "i: corrs_vec_new[i].size():" << corrs_vec_new[i].size() << endl;
		}

	cout << "show" << endl;

	//vector<pcl::Correspondences> corr_biggest_vec;
	//vector<pair<int, int>> index_pair_vec;
	CPointVisualization<T_PointType> pv;
	pv.setWindowName("Pairs");
	pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

	vector<std::uint8_t> color_corr_vec;
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);
	color_corr_vec.push_back(100);

	int index_framePair = 0;
	while (1)
	{
		if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corrs_all_vecvec.back().size())
		{
			int i_tgt = index_pair_vec.back().first;
			int i_src = index_pair_vec.back().second;
			pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
			pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

			//showing
			{
				Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
					calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
				pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
			}
			*cloud_show += *cloud_src;
			*cloud_show += *cloud_tgt;
			pv.setPointCloud(cloud_show);
			vector<std::uint8_t> color_corr_vec;
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);

			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			pcl::Correspondences corr_show;
			pv.drawCorrespondance(cloud_src, cloud_tgt, corrs_all_vecvec.back()[index_framePair], color_corr_vec);
			for (int i = 0; i < corrs_all_vecvec.back()[index_framePair].size(); i++)
				cout << "i:" << i << " distance:" << corrs_all_vecvec.back()[index_framePair][i].distance << endl;
			cout << endl;
			index_framePair++;
		}

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
		pv.updateViewer();

	}
	pv.closeViewer();


	//for (int j = 0; j < index_pair_vec.size(); j++)
	//{
	//	int i_tgt = index_pair_vec[j].first;
	//	int i_src = index_pair_vec[j].second;

	//	int num_nearest = 10;
	//	cout << "i_tgt:" << i_tgt << endl;
	//	cout << "i_src:" << i_src << endl;


	//}

	//for (int i_tgt = 0; i_tgt < cloud_vec.size(); i_tgt++)
	//{
	//	for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
	//	{
	//		int num_nearest = 10;
	//		index_pair_vec.push_back(make_pair(i_tgt, i_src));
	//		cout << "i_tgt:" << i_tgt << endl;
	//		cout << "i_src:" << i_src << endl;
	//		//cout << "fpfh_vec[i_src]->size():" << fpfh_vec[i_src]->size() << endl;
	//		//cout << "fpfh_vec[i_tgt]->size():" << fpfh_vec[i_tgt]->size() << endl;
	//		//{
	//		//	int i_max = 0;
	//		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_src].size(); j++)
	//		//		if (i_max < index_valid_vecvec_FPFH[i_src][j]) i_max = index_valid_vecvec_FPFH[i_src][j];
	//		//	cout << "i_src max:" << i_max << endl;
	//		//}
	//		//{
	//		//	int i_max = 0;
	//		//	for (int j = 0; j < index_valid_vecvec_FPFH[i_tgt].size(); j++)
	//		//		if (i_max < index_valid_vecvec_FPFH[i_tgt][j]) i_max = index_valid_vecvec_FPFH[i_tgt][j];
	//		//	cout << "i_tgt max:" << i_max << endl;
	//		//}

	//		//select biggest 10 corr
	//		{
	//			int num_select = 10;
	//			vector<vector<int>> index_biggest_vecvec;
	//			for (int j = 0; j < num_select; j++)
	//			{
	//				vector<int> index_biggest_vec;
	//				index_biggest_vec.push_back(0);
	//				index_biggest_vec.push_back(0);
	//				index_biggest_vecvec.push_back(index_biggest_vec);
	//			}
	//			for (int j = 0; j < corrs_vec.size(); j++)
	//			{
	//				vector<int> index_biggest_vec;
	//				index_biggest_vec.push_back(j);
	//				index_biggest_vec.push_back(corrs_vec[j].size());
	//				index_biggest_vecvec.push_back(index_biggest_vec);
	//				CTimeString::sortVector2d(index_biggest_vecvec, 1, false);
	//				index_biggest_vecvec.pop_back();
	//			}
	//			if (corrs_vec.size() == 0) continue;
	//			vector<pcl::Correspondences> corrs_vec_new;
	//			for (int j = 0; j < index_biggest_vecvec.size(); j++)
	//				corrs_vec_new.push_back(corrs_vec[index_biggest_vecvec[j][0]]);
	//			corrs_vec = corrs_vec_new;
	//			for (int j = 0; j < corrs_vec.size(); j++)
	//				cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
	//			corr_biggest_vec.push_back(corrs_vec[0]);
	//		}

	//	}
	//}

	//cout << "show" << endl;

	////vector<pcl::Correspondences> corr_biggest_vec;
	////vector<pair<int, int>> index_pair_vec;
	//CPointVisualization<T_PointType> pv;
	//pv.setWindowName("Pairs");
	//pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
	//pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
	//pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

	//vector<std::uint8_t> color_corr_vec;
	//color_corr_vec.push_back(100);
	//color_corr_vec.push_back(100);
	//color_corr_vec.push_back(100);

	//int index_framePair = 0;
	//while (1)
	//{
	//	if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corr_biggest_vec.size())
	//	{
	//		int i_tgt = index_pair_vec[index_framePair].first;
	//		int i_src = index_pair_vec[index_framePair].second;
	//		pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
	//		pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

	//		//showing
	//		{
	//			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
	//				calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
	//			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
	//		}
	//		*cloud_show += *cloud_src;
	//		*cloud_show += *cloud_tgt;
	//		pv.setPointCloud(cloud_show);
	//		vector<std::uint8_t> color_corr_vec;
	//		color_corr_vec.push_back(100);
	//		color_corr_vec.push_back(100);
	//		color_corr_vec.push_back(100);

	//		cout << "i_tgt:" << i_tgt << endl;
	//		cout << "i_src:" << i_src << endl;
	//		pcl::Correspondences corr_show;
	//		pv.drawCorrespondance(cloud_src, cloud_tgt, corr_biggest_vec[index_framePair], color_corr_vec);
	//		for (int i = 0; i < corr_biggest_vec[index_framePair].size(); i++)
	//			cout << "i:" << i << " distance:" << corr_biggest_vec[index_framePair][i].distance << endl;
	//		cout << endl;
	//		index_framePair++;
	//	}

	//	if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
	//	pv.updateViewer();

	//}
	//pv.closeViewer();

}

void CGlobalFeatureRegistration::DoDifferential_PairEvaluation2(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;

	bool b_useHistogramRemover = false;
	bool b_useDivergence = false;
	bool b_useCorrCompare = false;
	bool b_useGeometricConstraints = false;
	bool b_useParameterAdjustment = false;

	//b_useHistogramRemover = true;
	//b_useDivergence = true;

	b_useCorrCompare = true;
	b_useGeometricConstraints = true;

	b_useParameterAdjustment = true;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		//cout << "input folder you want to calc ->";
		int i_folder;
		//cin >> i_folder;
		i_folder = 2;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	float radius_differential = 0.5;

	//float th_velodyne_min_color;
	//th_velodyne_min_color = -4.;
	//float th_velodyne_max_color;
	//th_velodyne_max_color = 3.6;
	//float th_nir_min_color;
	//th_nir_min_color = -20;
	//float th_nir_max_color;
	//th_nir_max_color = 70.;

	float sigma_weight = 0.25;
	int num_bin_hist = 80;

	bool b_remove0value_nir = false;
	//b_remove0value_nir = true;

	//{
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter.csv");
	//	radius_differential = stof(s_temp_vecvec[1][3]);
	//	th_velodyne_min_color = stof(s_temp_vecvec[2][3]);
	//	th_velodyne_max_color = stof(s_temp_vecvec[3][3]);
	//	th_nir_min_color = stof(s_temp_vecvec[4][3]);
	//	th_nir_max_color = stof(s_temp_vecvec[5][3]);
	//	sigma_weight = stof(s_temp_vecvec[6][3]);
	//}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;


	//feature
	vector<vector<float>> feature_vecvec_nir;
	vector<vector<float>> feature_vecvec_velodyne;
	vector<vector<float>> featureDivergence_vecvec_nir;
	vector<vector<float>> featureDivergence_vecvec_velodyne;
	{
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			feature_vecvec_nir.push_back(feature_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
			feature_vecvec_velodyne.push_back(feature_vec);
		}
		//calc Laplacian
		if (b_useDivergence)
		{
			cout << "calc Divergence" << endl;
			for (int j = 0; j < cloud_vec.size(); j++)
			{
				pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
				kdtree_->setInputCloud(cloud_vec[j]);
				vector<float> featureDivergence_vec;
				featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
				featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
			}
			for (int j = 0; j < cloud_vec.size(); j++)
			{
				pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
				kdtree_->setInputCloud(cloud_vec[j]);
				vector<float> featureDivergence_vec;
				featureDivergence_vec = CExtendableICP::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
				featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
			}
		}
	}

	bool b_showHistogram = false;
	//b_showHistogram = true;
	vector<vector<int>> index_valid_vecvec_nir;
	vector<vector<int>> index_valid_vecvec_velodyne;
	for (int j = 0; j < featureDivergence_vecvec_nir.size(); j++)
	{
		vector<int> featureDivergence_vec;
		for (int i = 0; i < featureDivergence_vecvec_nir[j].size(); i++)
		{
			featureDivergence_vec.push_back(i);
		}
		index_valid_vecvec_nir.push_back(featureDivergence_vec);
	}
	for (int j = 0; j < featureDivergence_vecvec_velodyne.size(); j++)
	{
		vector<int> featureDivergence_vec;
		for (int i = 0; i < featureDivergence_vecvec_velodyne[j].size(); i++)
		{
			featureDivergence_vec.push_back(i);
		}
		index_valid_vecvec_velodyne.push_back(featureDivergence_vec);
	}
	if (b_useHistogramRemover && b_useDivergence)
	{
		cout << "calc HistogramRemover" << endl;
		index_valid_vecvec_nir = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CExtendableICP::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
	}

	//FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	//vector<Eigen::Matrix4f> transformation_vec;
	//vector<pcl::Correspondences> corr_biggest_vec;
	vector<pair<int, int>> index_pair_vec;
	//for (int i_tgt = 0; i_tgt < cloud_vec.size(); i_tgt++)
	//{
	//	for (int i_src = i_tgt + 1; i_src < cloud_vec.size(); i_src++)
	//		index_pair_vec.push_back(make_pair(i_tgt, i_src));
	//}
	{
		int i_tgt = 5;
		int i_src = 6;
		index_pair_vec.push_back(make_pair(i_tgt, i_src));
	}

	vector<pcl::Correspondences> corrs_nir_vec;
	vector<pcl::Correspondences> corrs_velodyne_vec;
	vector<pcl::Correspondences> corrs_fpfh_vec;

	float th_nearest_nir;
	float th_rank_rate_nir;

	float th_nearest_velodyne;
	float th_rank_rate_velodyne;

	float th_nearest_fpfh;
	int num_nearest_fpfh;
	float th_rank_rate_fpfh;

	float th_geometricConstraint = 0.8;

	//for (int j = 0; j < index_pair_vec.size(); j++)
	//{
	//	cout << "calc corr" << endl;
	//	//int i_tgt = 0;
	//	int i_tgt = index_pair_vec[j].first;
	//	int i_src = index_pair_vec[j].second;

	//	cout << "i_tgt:" << i_tgt << endl;
	//	cout << "i_src:" << i_src << endl;

	//	pcl::Correspondences corrs_fpfh;

	//	//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//	//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
	//	corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove_value(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//		index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest_fpfh, th_nearest_fpfh);

	//	corrs_fpfh_vec.push_back(corrs_fpfh);
	//}

	bool b_first = true;

	while (1)
	{
		if (b_useDivergence)
		{
			th_nearest_nir = 0.5;
			th_rank_rate_nir = 0.05;
			th_nearest_velodyne = 0.05;
			//th_rank_rate_velodyne
		}
		else
		{
			th_nearest_nir = 10.;
			th_rank_rate_nir = 0.5;
			//cout << "input th_nearest_nir:";
			//cin >> th_nearest_nir;
			th_nearest_velodyne = 10.;
			th_rank_rate_velodyne = 0.5;

			th_nearest_fpfh = 1800.;
			num_nearest_fpfh = 10;
			th_rank_rate_fpfh = 0.5;
		}

		if (b_useParameterAdjustment)
		{
			if (!b_first)
			{
				int aa;
				cout << "input txt:";
				cin >> aa;
			}
			b_first = false;
			vector<vector<string>> s_temp_vecvec;
			s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter2.csv");
			th_nearest_nir = stof(s_temp_vecvec[1][3]);
			th_rank_rate_nir = stof(s_temp_vecvec[2][3]);
			th_nearest_velodyne = stof(s_temp_vecvec[3][3]);
			th_rank_rate_velodyne = stof(s_temp_vecvec[4][3]);
			th_geometricConstraint = stof(s_temp_vecvec[5][3]);
		}

		cout << "nir" << endl;
		if (b_useHistogramRemover && b_useDivergence)
			CExtendableICP::determineCorrespondences_allFramesRanking_featureScalar_remove(featureDivergence_vecvec_nir, cloud_vec, index_pair_vec, th_nearest_nir, th_rank_rate_nir, index_valid_vecvec_nir, corrs_nir_vec, true);
		else
			CExtendableICP::determineCorrespondences_allFramesRanking_featureScalar(feature_vecvec_nir, cloud_vec, index_pair_vec, th_nearest_nir, th_rank_rate_nir, corrs_nir_vec, true);

		cout << "velodyne" << endl;
		if (b_useHistogramRemover && b_useDivergence)
			CExtendableICP::determineCorrespondences_allFramesRanking_featureScalar_remove(featureDivergence_vecvec_velodyne, cloud_vec, index_pair_vec, th_nearest_velodyne, th_rank_rate_nir, index_valid_vecvec_velodyne, corrs_velodyne_vec, true);
		else
			CExtendableICP::determineCorrespondences_allFramesRanking_featureScalar(feature_vecvec_velodyne, cloud_vec, index_pair_vec, th_nearest_velodyne, th_rank_rate_nir, corrs_velodyne_vec, true);

		cout << "fpfh" << endl;
		CFPFH_PCL::determineCorrespondences_allFramesRanking_featureFpfh_remove(fpfh_vec, cloud_vec, index_pair_vec, th_nearest_fpfh, num_nearest_fpfh, th_rank_rate_fpfh,
			index_valid_vecvec_FPFH, corrs_fpfh_vec, true);

		//sum all features
		vector<vector<pcl::Correspondences>> corrs_all_vecvec;
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			pcl::Correspondences corrs_temp;
			corrs_temp.insert(corrs_temp.end(), corrs_nir_vec[j].begin(), corrs_nir_vec[j].end());
			corrs_temp.insert(corrs_temp.end(), corrs_velodyne_vec[j].begin(), corrs_velodyne_vec[j].end());
			corrs_temp.insert(corrs_temp.end(), corrs_fpfh_vec[j].begin(), corrs_fpfh_vec[j].end());

			if (b_useGeometricConstraints)
			{
				cout << "calc GeometricConstraints" << endl;
				cout << "i_tgt:" << i_tgt << endl;
				cout << "i_src:" << i_src << endl;
				corrs_all_vecvec.push_back(CExtendableICP::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, th_geometricConstraint, true));
			}
			else
			{
				vector<pcl::Correspondences> corrs_temp_vec;
				corrs_temp_vec.push_back(corrs_temp);
				corrs_all_vecvec.push_back(corrs_temp_vec);
			}
			//for (int j = 0; j < corrs_vec.size(); j++)
			//	cout << "j:" << j << " corrs_vec[j].size():" << corrs_vec[j].size() << endl;
		}

		int num_select = 10;
		cout << "biggest corrs  num_select:" << num_select << endl;
		if (b_useGeometricConstraints)
			for (int j = 0; j < corrs_all_vecvec.size(); j++)
			{
				cout << "frame:" << j << endl;
				vector<pcl::Correspondences> corrs_vec_new;

				if (corrs_all_vecvec[j].size() > num_select)
				{
					for (int i = 0; i < num_select; i++)
						corrs_vec_new.push_back(corrs_all_vecvec[j][i]);
					corrs_all_vecvec[j] = corrs_vec_new;
				}
			}

		//vector<pcl::Correspondences> corr_biggest_vec;
		//vector<pair<int, int>> index_pair_vec;
		CPointVisualization<T_PointType> pv;
		pv.setWindowName("Pairs");
		pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

		cout << "show corr and cloud" << endl;
		int index_framePair = 0;
		while (1)
		{
			if ((GetAsyncKeyState(VK_SPACE) & 1) == 1 && index_framePair < corrs_all_vecvec.back().size())
			{
				int i_tgt = index_pair_vec.back().first;
				int i_src = index_pair_vec.back().second;
				pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
				pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);

				//showing
				{
					Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
						calcHomogeneousMatrixFromVector6d(0., 0., 10., 0., 0., 0.));
					pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
				}
				*cloud_show += *cloud_src;
				*cloud_show += *cloud_tgt;
				pv.setPointCloud(cloud_show);
				vector<std::uint8_t> color_corr_vec;
				color_corr_vec.push_back(100);
				color_corr_vec.push_back(100);
				color_corr_vec.push_back(100);

				cout << "i_tgt:" << i_tgt << endl;
				cout << "i_src:" << i_src << endl;
				pcl::Correspondences corr_show;
				pv.drawCorrespondance(cloud_src, cloud_tgt, corrs_all_vecvec.back()[index_framePair], color_corr_vec);
				for (int i = 0; i < corrs_all_vecvec.back()[index_framePair].size(); i++)
					cout << "i:" << i << " distance:" << corrs_all_vecvec.back()[index_framePair][i].distance << endl;
				cout << endl;
				index_framePair++;
			}

			if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
			pv.updateViewer();

		}
		pv.closeViewer();

		if (!b_useParameterAdjustment) break;
	}

}

void CGlobalFeatureRegistration::DoDifferential_PairEvaluation3(string dir_)
{
	typedef pcl::PointXYZRGB T_PointType;
	bool b_useParameterAdjustment = false;
	bool b_show_AllFrame_AllPairs = false;
	bool b_useGeometricConstraints = false;
	bool b_useRigidTransformation = false;

	bool b_useNir = false;
	bool b_useVelodyne = false;
	bool b_useFPFH = false;

	bool b_useColorfullCorr = false;

	bool b_changeColor_nir = false;
	bool b_changeColor_velodyne = false;

	b_useParameterAdjustment = true;
	b_show_AllFrame_AllPairs = true;
	b_useGeometricConstraints = true;

	b_useNir = true;
	b_useVelodyne = true;
	b_useFPFH = true;

	//b_useColorfullCorr = true;

	b_changeColor_nir = true;

	b_useRigidTransformation = true;

	if (!b_useNir)  b_changeColor_nir = false;

	string s_folder;
	{
		vector<string> filenames_folder;

		CTimeString::getFileNames_folder(dir_, filenames_folder);
		for (int i = 0; i < filenames_folder.size(); i++)
		{
			string s_i = to_string(i);
			if (s_i.size() < 2) s_i = " " + s_i;
			cout << "i:" << s_i << " " << filenames_folder[i] << endl;
		}
		cout << endl;
		//cout << "input folder you want to calc ->";
		int i_folder;
		//cin >> i_folder;
		i_folder = 2;
		s_folder = filenames_folder[i_folder];

	}

	//input pointcloud
	vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			cloud_vec.push_back(cloud);
		}
	}

	for (int j = 0; j < cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << cloud_vec[j]->size() << endl;
	cout << endl;

	//feature
	vector<vector<float>> feature_vecvec_nir;
	vector<vector<float>> feature_vecvec_velodyne;
	if (b_useNir)
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].r));
			feature_vecvec_nir.push_back(feature_vec);
		}
	if (b_useVelodyne)
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			vector<float> feature_vec;
			for (int i = 0; i < cloud_vec[j]->size(); i++)
				feature_vec.push_back((float)((int)cloud_vec[j]->points[i].g));
			feature_vecvec_velodyne.push_back(feature_vec);
		}

	if (b_changeColor_nir)
	{
		float value_max = -std::numeric_limits<float>::max();
		float value_min = std::numeric_limits<float>::max();
		for (int j = 0; j < feature_vecvec_nir.size(); j++)
		{
			for (int i = 0; i < feature_vecvec_nir[j].size(); i++)
			{
				if (value_max < feature_vecvec_nir[j][i]) value_max = feature_vecvec_nir[j][i];
				if (value_min > feature_vecvec_nir[j][i]) value_min = feature_vecvec_nir[j][i];
			}
		}
		for (int j = 0; j < feature_vecvec_nir.size(); j++)
		{
			for (int i = 0; i < feature_vecvec_nir[j].size(); i++)
			{
				vector<std::uint8_t> color_vec;
				color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(feature_vecvec_nir[j][i], value_max, value_min);
				T_PointType point_ = cloud_vec[j]->points[i];
				point_.r = color_vec[0];
				point_.g = color_vec[1];
				point_.b = color_vec[2];
				cloud_vec[j]->points[i] = point_;
			}
		}
	}

	//calc valid point of FPFH
	vector<vector<int>> index_valid_vecvec_FPFH;
	vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	if (b_useFPFH)
	{
		float radius_normal_FPFH;
		radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < cloud_vec.size(); j++)
			fpfh_vec.push_back(CFPFH_PCL::computeFPFH(cloud_vec[j], cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}

	vector<pair<int, int>> index_pair_vec;

	//{
	//	int i_tgt = 5;
	//	int i_src = 6;
	//	index_pair_vec.push_back(make_pair(i_tgt, i_src));
	//}
	//{
	//	int i_tgt = 5;
	//	int i_src = 7;
	//	index_pair_vec.push_back(make_pair(i_tgt, i_src));
	//}
	for (int j = 0; j < cloud_vec.size() - 1; j++)
	{
		for (int i = j + 1; i < cloud_vec.size(); i++)
		{
			int i_tgt = j;
			int i_src = i;
			if (!(j == 5
				|| j == 6
				|| j == 7
				|| j == 8
				|| j == 11
				|| j == 12
				|| j == 16
				)) continue;
			if (!(i == 5
				|| i == 6
				|| i == 7
				|| i == 8
				|| i == 11
				|| i == 12
				|| i == 16
				)) continue;

			//if (!(j == 5
			//	|| j == 6
			//	|| j == 7
			//	|| j == 8
			//	)) continue;
			//if (!(i == 5
			//	|| i == 6
			//	|| i == 7
			//	|| i == 8
			//	)) continue;

			index_pair_vec.push_back(make_pair(i_tgt, i_src));
		}
	}

	float th_nearest_nir;
	float th_rank_rate_nir;

	float th_nearest_velodyne;
	float th_rank_rate_velodyne;

	float th_nearest_fpfh;
	int num_nearest_fpfh;
	float th_rank_rate_fpfh;

	float th_geometricConstraint = 0.8;

	int i_method_rigidTransformation;

	bool b_first = true;

	while (1)
	{
		th_nearest_nir = 10.;
		//th_rank_rate_nir = 0.5;
		//th_rank_rate_nir = 1.;
		th_rank_rate_nir = 0.2;
		//cout << "input th_nearest_nir:";
		//cin >> th_nearest_nir;
		th_nearest_velodyne = 10.;
		//th_rank_rate_velodyne = 0.5;
		th_rank_rate_velodyne = 1.;

		th_nearest_fpfh = 1800.;
		num_nearest_fpfh = 10;
		//th_rank_rate_fpfh = 0.5;
		th_rank_rate_fpfh = 0.5;

		if (b_useParameterAdjustment)
		{
			if (!b_first)
			{
				int aa;
				cout << "input txt:";
				cin >> aa;
			}
			b_first = false;
			vector<vector<string>> s_temp_vecvec;
			s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter2.csv");
			th_nearest_nir = stof(s_temp_vecvec[1][3]);
			th_rank_rate_nir = stof(s_temp_vecvec[2][3]);
			th_nearest_velodyne = stof(s_temp_vecvec[3][3]);
			th_rank_rate_velodyne = stof(s_temp_vecvec[4][3]);
			th_rank_rate_fpfh = stof(s_temp_vecvec[5][3]);
			th_geometricConstraint = stof(s_temp_vecvec[6][3]);
			i_method_rigidTransformation = stoi(s_temp_vecvec[7][3]);
			b_useRigidTransformation = (bool)stoi(s_temp_vecvec[8][3]);
		}

		cout << "calc pairs" << endl;
		vector<pcl::Correspondences> corrs_nir_vec;
		vector<pcl::Correspondences> corrs_velodyne_vec;
		vector<pcl::Correspondences> corrs_fpfh_vec;
		vector<vector<float>> evaluation_corr_vecvec_nir;
		vector<vector<float>> evaluation_corr_vecvec_velodyne;
		vector<vector<float>> evaluation_corr_vecvec_fpfh;
		if (b_useNir)
		{
			cout << "nir" << endl;
			CExtendableICP::determineCorrespondences_allFramesRanking_featureScalar(feature_vecvec_nir, cloud_vec, index_pair_vec, th_nearest_nir, th_rank_rate_nir, corrs_nir_vec, evaluation_corr_vecvec_nir);
		}
		if (b_useVelodyne)
		{
			cout << "velodyne" << endl;
			CExtendableICP::determineCorrespondences_allFramesRanking_featureScalar(feature_vecvec_velodyne, cloud_vec, index_pair_vec, th_nearest_velodyne, th_rank_rate_nir, corrs_velodyne_vec, evaluation_corr_vecvec_velodyne);
		}
		if (b_useFPFH)
		{
			cout << "fpfh" << endl;
			CFPFH_PCL::determineCorrespondences_allFramesRanking_featureFpfh_remove(fpfh_vec, cloud_vec, index_pair_vec, th_nearest_fpfh, num_nearest_fpfh, th_rank_rate_fpfh,
				index_valid_vecvec_FPFH, corrs_fpfh_vec, evaluation_corr_vecvec_fpfh);
		}

		cout << "corr" << endl;
		for (int j = 0; j < corrs_nir_vec.size(); j++)
			cout << "j:" << j << " size:" << corrs_nir_vec[j].size() << endl;
		cout << "evaluation" << endl;
		for (int j = 0; j < evaluation_corr_vecvec_nir.size(); j++)
			cout << "j:" << j << " size:" << evaluation_corr_vecvec_nir[j].size() << endl;

		if (b_useGeometricConstraints) cout << "calc GeometricConstraints" << endl;
		vector<vector<pcl::Correspondences>> corrs_all_vecvec;
		for (int j = 0; j < index_pair_vec.size(); j++)
		{
			int i_tgt = index_pair_vec[j].first;
			int i_src = index_pair_vec[j].second;
			pcl::Correspondences corrs_temp;
			if (b_useNir) corrs_temp.insert(corrs_temp.end(), corrs_nir_vec[j].begin(), corrs_nir_vec[j].end());
			if (b_useVelodyne) corrs_temp.insert(corrs_temp.end(), corrs_velodyne_vec[j].begin(), corrs_velodyne_vec[j].end());
			if (b_useFPFH) corrs_temp.insert(corrs_temp.end(), corrs_fpfh_vec[j].begin(), corrs_fpfh_vec[j].end());
			cout << "i_tgt:" << i_tgt << endl;
			cout << "i_src:" << i_src << endl;
			if (b_useGeometricConstraints)
				corrs_all_vecvec.push_back(CExtendableICP::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, th_geometricConstraint, true));
			else
			{
				vector<pcl::Correspondences> corrs_vec_temp;
				corrs_vec_temp.push_back(corrs_temp);
				corrs_all_vecvec.push_back(corrs_vec_temp);
			}
		}

		if (b_show_AllFrame_AllPairs && !b_useRigidTransformation)
		{
			CPointVisualization<T_PointType> pv;
			pv.setWindowName("Pairs");
			pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
			pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
			pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

			cout << "show corr and cloud" << endl;

			bool b_first_vis = true;

			int index_frame_pair = 0;
			int index_pair = 0;

			bool b_updatePair = false;
			bool b_updateFramePair = false;

			int i_tgt_vis;
			int i_src_vis;

			vector<std::uint8_t> color_corr_vec;
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);
			color_corr_vec.push_back(100);
			vector<vector<std::uint8_t>> color_corr_vecvec;

			cout << "Press SPACE" << endl;
			while (1)
			{
				bool b_next_pair = false;
				bool b_next_frame_pair = false;
				b_next_pair = (GetAsyncKeyState(VK_SPACE) & 1) == 1;
				b_next_frame_pair = (GetAsyncKeyState(VK_LSHIFT) & 1) == 1;

				if (b_next_pair)
				{
					if (corrs_all_vecvec[index_frame_pair].size() > index_pair + 1)
					{
						index_pair++;
						//update pair
						b_updatePair = true;
					}
					else
					{
						if (corrs_all_vecvec.size() > index_frame_pair + 1)
						{
							index_frame_pair++;
							//update frame_pair
							b_updateFramePair = true;

							index_pair = 0;
							//update pair
							b_updatePair = true;
						}
						else
						{
							//no process
						}
					}
				}

				if (b_next_frame_pair && corrs_all_vecvec.size() > index_frame_pair + 1)
				{
					index_frame_pair++;
					//update frame_pair
					b_updateFramePair = true;

					index_pair = 0;
					//update pair
					b_updatePair = true;
				}


				if (b_updateFramePair || b_first_vis)
				{
					i_tgt_vis = index_pair_vec[index_frame_pair].first;
					i_src_vis = index_pair_vec[index_frame_pair].second;
					cout << "i_tgt:" << i_tgt_vis << endl;
					cout << "i_src:" << i_src_vis << endl;
					cout << "(There are " << corrs_all_vecvec[index_frame_pair].size() << " correspondences)" << endl;
					pcl::copyPointCloud(*cloud_vec[i_src_vis], *cloud_src);
					pcl::copyPointCloud(*cloud_vec[i_tgt_vis], *cloud_tgt);
					//showing
					{
						//Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
						//	calcHomogeneousMatrixFromVector6d(0., 0., 20., 0., 0., 0.));
						Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(
							calcHomogeneousMatrixFromVector6d(10., 0., 20., 0., 0., 0.));
						pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
					}
					cloud_show->clear();
					*cloud_show += *cloud_src;
					*cloud_show += *cloud_tgt;
					pv.setPointCloud(cloud_show);
					b_updateFramePair = false;
				}

				if (b_updatePair || b_first_vis)
				{
					//pcl::Correspondences corr_show;
					if (!b_useGeometricConstraints)
					{
						color_corr_vecvec.clear();
						if (b_useNir)
							for (int j = 0; j < evaluation_corr_vecvec_nir[index_frame_pair].size(); j++)
							{

								color_corr_vecvec.push_back(CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(
									evaluation_corr_vecvec_nir[index_frame_pair][j], evaluation_corr_vecvec_nir[index_frame_pair].back(),
									evaluation_corr_vecvec_nir[index_frame_pair][0]));
							}
						if (b_useVelodyne)
							for (int j = 0; j < evaluation_corr_vecvec_velodyne[index_frame_pair].size(); j++)
							{

								color_corr_vecvec.push_back(CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(
									evaluation_corr_vecvec_velodyne[index_frame_pair][j], evaluation_corr_vecvec_velodyne[index_frame_pair].back(),
									evaluation_corr_vecvec_velodyne[index_frame_pair][0]));
							}
						if (b_useFPFH)
							for (int j = 0; j < evaluation_corr_vecvec_fpfh[index_frame_pair].size(); j++)
							{

								color_corr_vecvec.push_back(CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(
									evaluation_corr_vecvec_fpfh[index_frame_pair][j], evaluation_corr_vecvec_fpfh[index_frame_pair].back(),
									evaluation_corr_vecvec_fpfh[index_frame_pair][0]));
							}
					}

					if (corrs_all_vecvec[index_frame_pair].size() != 0)
					{
						cout << "index_pair:" << index_pair << endl;
						if (b_useGeometricConstraints || !b_useColorfullCorr) pv.drawCorrespondance(cloud_src, cloud_tgt, corrs_all_vecvec[index_frame_pair][index_pair], color_corr_vec);
						else pv.drawCorrespondance(cloud_src, cloud_tgt, corrs_all_vecvec[index_frame_pair][index_pair], color_corr_vecvec);
						cout << "showing " << corrs_all_vecvec[index_frame_pair][index_pair].size() << " pairs" << endl;
					}
					b_updatePair = false;
				}

				b_first_vis = false;

				if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
				pv.updateViewer();
			}

			pv.closeViewer();
		}

		//b_useRigidTransformation
		else
		{
			vector<vector<float>> evaluation_corrsCluster_vecvec;
			vector<pcl::Correspondences> corrs_output_vec;

			for (int j = 0; j < index_pair_vec.size(); j++)
			{
				int i_tgt = index_pair_vec[j].first;
				int i_src = index_pair_vec[j].second;
				corrs_output_vec.push_back(CExtendableICP::determineCorrespondences_geometricConstraint_evaluateCluster(
					cloud_vec[i_src], cloud_vec[i_tgt], corrs_all_vecvec[j], i_method_rigidTransformation));
			}
			vector<Eigen::Matrix4d> transformation_vec;
			for (int j = 0; j < index_pair_vec.size(); j++)
			{
				int i_tgt = index_pair_vec[j].first;
				int i_src = index_pair_vec[j].second;
				Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
				CExtendableICP::estimateRigidTransformation_static(cloud_vec[i_src], cloud_vec[i_tgt], corrs_output_vec[j], transformation_matrix);
				transformation_vec.push_back(transformation_matrix.cast<double>());
			}
			CPointVisualization<T_PointType> pv;
			pv.setWindowName("Transformation");
			bool b_first_vis = true;
			bool b_updateFramePair = false;

			int index_frame_pair = 0;

			pcl::PointCloud<T_PointType>::Ptr cloud_show(new pcl::PointCloud<T_PointType>());
			pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
			pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

			cout << "show transformation result" << endl;
			while (1)
			{
				bool b_next_frame_pair = false;
				b_next_frame_pair = (GetAsyncKeyState(VK_SPACE) & 1) == 1;

				if ((b_next_frame_pair && index_frame_pair + 1 < index_pair_vec.size()) && !b_first_vis)
				{
					b_updateFramePair = true;
					index_frame_pair++;
				}

				if (b_updateFramePair || b_first_vis)
				{
					int i_tgt = index_pair_vec[index_frame_pair].first;
					int i_src = index_pair_vec[index_frame_pair].second;
					cout << "showing:  i_tgt:" << i_tgt << "  i_src:" << i_src << endl;
					pcl::copyPointCloud(*cloud_vec[i_tgt], *cloud_tgt);
					pcl::copyPointCloud(*cloud_vec[i_src], *cloud_src);
					//color
					for (int j = 0; j < cloud_tgt->size(); j++)
					{
						cloud_tgt->points[j].r = 255;
						cloud_tgt->points[j].g = 0;
						cloud_tgt->points[j].b = 0;
					}
					for (int j = 0; j < cloud_src->size(); j++)
					{
						cloud_src->points[j].r = 0;
						cloud_src->points[j].g = 255;
						cloud_src->points[j].b = 0;
					}
					pcl::transformPointCloud(*cloud_src, *cloud_src, transformation_vec[index_frame_pair]);
					cloud_show->clear();
					*cloud_show += *cloud_tgt;
					*cloud_show += *cloud_src;
					pv.setPointCloud(cloud_show);
					b_updateFramePair = false;
				}

				b_first_vis = false;

				if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
				pv.updateViewer();

			}
			pv.closeViewer();

		}

		if (!b_useParameterAdjustment) break;
	}

}

