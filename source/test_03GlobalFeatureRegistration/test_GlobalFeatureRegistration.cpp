#include "test_GlobalFeatureRegistration.h"

void CGlobalFeatureRegistration_test::mainProcess()
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
		EN_PairEvaluation3,
		EN_VariParamaters_GlobalRegistration,
		EN_CompareGlobalRegistration,
		EN_ICP_VariParamaters,
		EN_CompareICP
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
		cout << " " << EN_VariParamaters_GlobalRegistration << ": VariParamaters_GlobalRegistration" << endl;
		cout << " " << EN_CompareGlobalRegistration << ": CompareGlobalRegistration" << endl;
		cout << " " << EN_ICP_VariParamaters << ": ICP_VariParamaters" << endl;
		cout << " " << EN_CompareICP << ": CompareICP" << endl;

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
			CGlobalFeatureRegistration_test::FreeSpace();
			break;

		case EN_FileProcess:
			FileProcess(dir_);
			break;

		case EN_SequentShow:
			show_sequent_PointTypes(dir_);
			break;

		case EN_DrawTrajectory:
			DrawTrajectory(dir_ + "/DrawTrajectory");
			break;

		case EN_DoMappingFromTrajectory:
			DoMappingFromTrajectory(dir_ + "/MappingFromTrajectory");
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

		case EN_VariParamaters_GlobalRegistration:
			variParamaters(dir_);
			break;

		case EN_CompareGlobalRegistration:
			compareGlobalRegistration(dir_ + "/Result_01varyParameters/_Comparison");
			break;

		case EN_ICP_VariParamaters:
			align_ICP_fromGlobalRegistration_variParamaters(dir_);
			break;

		case EN_CompareICP:
			compareICP(dir_ + "/Result_02_ICP_varyParameters/_Comparison");

		default:
			break;
		}

	}
}

void CGlobalFeatureRegistration_test::FreeSpace()
{

}

void CGlobalFeatureRegistration_test::DoDifferential_1pointcloud(string dir_)
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
	featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_, feature_vec, kdtree_, radius_differential, 0.25);

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

void CGlobalFeatureRegistration_test::DoDifferential_SomePointclouds(string dir_)
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
		featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec[j], kdtree_, radius_differential, sigma_weight, true);
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
	index_valid_vecvec = CGlobalFeatureRegistration::calcValidIndex_feature(featureDivergence_vecvec, num_bin_hist, true);

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
		corr_new = CGlobalFeatureRegistration::determineCorrespondences_featureScalar_num_remove(featureDivergence_vecvec[i_src], featureDivergence_vecvec[i_tgt],
			index_valid_vecvec[i_src], index_valid_vecvec[i_tgt], num_nearest);
		cout << "corr_new.size():" << corr_new.size() << endl;
		corr_new_vec = CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corr_new, 0.9);
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
		*cloud_ZValue = *CGlobalFeatureRegistration::getPointCloud_ZAaxisByFeature(cloud_ZValue, feature_vec, 10.);

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

void CGlobalFeatureRegistration_test::DoDifferential_showFeatureValue(string dir_)
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
		*cloud_ZValue = *CGlobalFeatureRegistration::getPointCloud_ZAaxisByFeature(cloud_ZValue, feature_vecvec[j], 3.);

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

void CGlobalFeatureRegistration_test::FPFH_unique(string dir_)
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
	index_vecvec = CGlobalFeatureRegistration::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
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
		corrs_ = CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_num_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
			index_vecvec[i_src], index_vecvec[i_tgt], num_near);

		for (int j = 0; j < corrs_.size(); j++)
		{
			if (j % 1000 != 0) continue;
			cout << "j:" << j << "  query:" << corrs_[j].index_query << " match:" << corrs_[j].index_match << " distance:" << corrs_[j].distance << endl;
		}

	}
}

void CGlobalFeatureRegistration_test::DoDifferential_RigidTransformation_FPFH_Features(string dir_)
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
			featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
		}
		for (int j = 0; j < cloud_vec.size(); j++)
		{
			pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
			kdtree_->setInputCloud(cloud_vec[j]);
			vector<float> featureDivergence_vec;
			featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
			featureDivergence_vecvec_velodyne.push_back(featureDivergence_vec);
		}
		bool b_showHistogram = false;
		b_showHistogram = true;
		index_valid_vecvec_nir = CGlobalFeatureRegistration::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CGlobalFeatureRegistration::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
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
		index_valid_vecvec_FPFH = CGlobalFeatureRegistration::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);
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
	corrs_nir = CGlobalFeatureRegistration::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
		index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

	//corrs_velodyne = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
	//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
	corrs_velodyne = CGlobalFeatureRegistration::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

	//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
	//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
	corrs_fpfh = CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
		index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

	vector<pcl::Correspondences> corrs_vec;
	{
		pcl::Correspondences corrs_temp;
		corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
		corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
		corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
		corrs_vec = CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
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

void CGlobalFeatureRegistration_test::DoDifferential_RigidTransformation_FPFH_Features_new(string dir_)
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
		index_valid_vecvec_FPFH = CGlobalFeatureRegistration::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

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
		corrs_fpfh = CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
			index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest, th_nearest_fpfh);

		vector<pcl::Correspondences> corrs_vec;
		{
			pcl::Correspondences corrs_temp;
			//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
			//corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
			corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
			corrs_vec = CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
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

void CGlobalFeatureRegistration_test::DoDifferential_RigidTransformation_FPFH_Features_allFrames(string dir_)
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
		index_valid_vecvec_FPFH = CGlobalFeatureRegistration::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

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
			corrs_fpfh = CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_num_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
				index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);

			vector<pcl::Correspondences> corrs_vec;
			{
				pcl::Correspondences corrs_temp;
				//corrs_temp.insert(corrs_temp.end(), corrs_nir.begin(), corrs_nir.end());
				//corrs_temp.insert(corrs_temp.end(), corrs_velodyne.begin(), corrs_velodyne.end());
				corrs_temp.insert(corrs_temp.end(), corrs_fpfh.begin(), corrs_fpfh.end());
				corrs_vec = CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9);
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

void CGlobalFeatureRegistration_test::DoDifferential_PairEvaluation(string dir_)
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
				featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
				featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
			}
			for (int j = 0; j < cloud_vec.size(); j++)
			{
				pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
				kdtree_->setInputCloud(cloud_vec[j]);
				vector<float> featureDivergence_vec;
				featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
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
		index_valid_vecvec_nir = CGlobalFeatureRegistration::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CGlobalFeatureRegistration::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
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
		index_valid_vecvec_FPFH = CGlobalFeatureRegistration::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

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
		corrs_nir = CGlobalFeatureRegistration::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_nir[i_src], featureDivergence_vecvec_nir[i_tgt],
			index_valid_vecvec_nir[i_src], index_valid_vecvec_nir[i_tgt], th_nearest_nir);

		//corrs_velodyne = CExtendableICP::determineCorrespondences_feature_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
		//	index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], num_nearest);
		corrs_velodyne = CGlobalFeatureRegistration::determineCorrespondences_featureScalar_remove(featureDivergence_vecvec_velodyne[i_src], featureDivergence_vecvec_velodyne[i_tgt],
			index_valid_vecvec_velodyne[i_src], index_valid_vecvec_velodyne[i_tgt], th_nearest_velodyne);

		//corrs_fpfh = CFPFH_PCL::getNearestOfFPFH_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
		//	index_valid_vecvec_FPFH[i_src], index_valid_vecvec_FPFH[i_tgt], num_nearest);
		corrs_fpfh = CGlobalFeatureRegistration::determineCorrespondences_featureFpfh_eachPairHaving_remove(fpfh_vec[i_src], fpfh_vec[i_tgt],
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
			rank_nir_vecvec = CGlobalFeatureRegistration::calcRanking_featureScalar(index_pair_vec, corrs_nir_vec, cloud_vec, featureDivergence_vecvec_nir, index_valid_vecvec_nir, th_nearest_nir, true);
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
			rank_velodyne_vecvec = CGlobalFeatureRegistration::calcRanking_featureScalar(index_pair_vec, corrs_velodyne_vec, cloud_vec, featureDivergence_vecvec_velodyne, index_valid_vecvec_velodyne, th_nearest_velodyne, true);
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
			corrs_all_vecvec.push_back(CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, 0.9));
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

void CGlobalFeatureRegistration_test::DoDifferential_PairEvaluation2(string dir_)
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
				featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_nir[j], kdtree_, radius_differential, sigma_weight, true);
				featureDivergence_vecvec_nir.push_back(featureDivergence_vec);
			}
			for (int j = 0; j < cloud_vec.size(); j++)
			{
				pcl::KdTreeFLANN<T_PointType>::Ptr kdtree_(new pcl::KdTreeFLANN<T_PointType>);
				kdtree_->setInputCloud(cloud_vec[j]);
				vector<float> featureDivergence_vec;
				featureDivergence_vec = CGlobalFeatureRegistration::getPointCloud_featureDivergence(cloud_vec[j], feature_vecvec_velodyne[j], kdtree_, radius_differential, sigma_weight, true);
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
		index_valid_vecvec_nir = CGlobalFeatureRegistration::calcValidIndex_feature(featureDivergence_vecvec_nir, num_bin_hist, b_showHistogram);
		index_valid_vecvec_velodyne = CGlobalFeatureRegistration::calcValidIndex_feature(featureDivergence_vecvec_velodyne, num_bin_hist, b_showHistogram);
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
		index_valid_vecvec_FPFH = CGlobalFeatureRegistration::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

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
			CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureScalar_remove(featureDivergence_vecvec_nir, cloud_vec, index_pair_vec, th_nearest_nir, th_rank_rate_nir, index_valid_vecvec_nir, corrs_nir_vec, true);
		else
			CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureScalar(feature_vecvec_nir, cloud_vec, index_pair_vec, th_nearest_nir, th_rank_rate_nir, corrs_nir_vec, true);

		cout << "velodyne" << endl;
		if (b_useHistogramRemover && b_useDivergence)
			CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureScalar_remove(featureDivergence_vecvec_velodyne, cloud_vec, index_pair_vec, th_nearest_velodyne, th_rank_rate_nir, index_valid_vecvec_velodyne, corrs_velodyne_vec, true);
		else
			CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureScalar(feature_vecvec_velodyne, cloud_vec, index_pair_vec, th_nearest_velodyne, th_rank_rate_nir, corrs_velodyne_vec, true);

		cout << "fpfh" << endl;
		CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureFpfh_remove(fpfh_vec, cloud_vec, index_pair_vec, th_nearest_fpfh, num_nearest_fpfh, th_rank_rate_fpfh,
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
				corrs_all_vecvec.push_back(CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint(cloud_vec[i_src], cloud_vec[i_tgt], corrs_temp, th_geometricConstraint, true));
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

void CGlobalFeatureRegistration_test::DoDifferential_PairEvaluation3(string dir_)
{
	//typedef pcl::PointXYZRGB T_PointType;
	//bool b_useParameterAdjustment = false;
	//bool b_show_AllFrame_AllPairs = false;
	//bool b_useGeometricConstraints = false;
	//bool b_useRigidTransformation = false;

	//bool b_useNir = false;
	//bool b_useVelodyne = false;
	//bool b_useFPFH = false;

	//bool b_useColorfullCorr = false;

	//bool b_changeColor_nir = false;
	//bool b_changeColor_velodyne = false;

	//bool b_useOldFPFH = false;

	//b_useParameterAdjustment = true;
	//b_useGeometricConstraints = true;

	//b_useNir = true;
	//b_useVelodyne = true;
	//b_useFPFH = true;

	////b_useOldFPFH = true;

	//cout << "1: use FPFH   0: use proposed  ->";
	//cin >> b_useOldFPFH;

	//b_changeColor_nir = true;

	//b_useRigidTransformation = true;

	//if (!b_useRigidTransformation) b_show_AllFrame_AllPairs = true;

	//if (!b_useNir)  b_changeColor_nir = false;

	//inputData(dir_, b_useNir, b_useVelodyne, b_changeColor_nir, b_useFPFH, b_useOldFPFH);

	//vector<pair<int, int>> index_pair_vec;

	////{
	////	int i_tgt = 5;
	////	int i_src = 6;
	////	index_pair_vec.push_back(make_pair(i_tgt, i_src));
	////}
	////{
	////	int i_tgt = 5;
	////	int i_src = 7;
	////	index_pair_vec.push_back(make_pair(i_tgt, i_src));
	////}
	//for (int j = 0; j < M_cloud_vec.size() - 1; j++)
	//{
	//	for (int i = j + 1; i < M_cloud_vec.size(); i++)
	//	{
	//		int i_tgt = j;
	//		int i_src = i;
	//		if (!(j == 5
	//			|| j == 6
	//			|| j == 7
	//			//|| j == 8
	//			|| j == 11
	//			//|| j == 12
	//			//|| j == 16
	//			)) continue;
	//		if (!(i == 5
	//			|| i == 6
	//			|| i == 7
	//			//|| i == 8
	//			|| i == 11
	//			//|| i == 12
	//			//|| i == 16
	//			)) continue;

	//		//if (!(j == 5
	//		//	|| j == 6
	//		//	|| j == 7
	//		//	|| j == 8
	//		//	)) continue;
	//		//if (!(i == 5
	//		//	|| i == 6
	//		//	|| i == 7
	//		//	|| i == 8
	//		//	)) continue;

	//		index_pair_vec.push_back(make_pair(i_tgt, i_src));
	//	}
	//}

	//float th_nearest_nir;
	//float th_rank_rate_nir;
	//float th_nearest_velodyne;
	//float th_rank_rate_velodyne;
	//float th_nearest_fpfh;
	//int num_nearest_fpfh;
	//float th_rank_rate_fpfh;
	//int i_method_rigidTransformation;
	//float th_geometricConstraint;


	//bool b_first = true;

	//while (1)
	//{
	//	th_nearest_nir = 10.;
	//	//th_rank_rate_nir = 0.5;
	//	//th_rank_rate_nir = 1.;
	//	th_rank_rate_nir = 0.2;
	//	//cout << "input th_nearest_nir:";
	//	//cin >> th_nearest_nir;
	//	th_nearest_velodyne = 10.;
	//	//th_rank_rate_velodyne = 0.5;
	//	th_rank_rate_velodyne = 1.;

	//	th_nearest_fpfh = 1800.;
	//	num_nearest_fpfh = 10;
	//	//th_rank_rate_fpfh = 0.5;
	//	th_rank_rate_fpfh = 0.5;

	//	th_geometricConstraint = 0.8;

	//	if (b_useParameterAdjustment)
	//	{
	//		if (!b_first)
	//		{
	//			int aa;
	//			cout << "input txt:";
	//			cin >> aa;
	//		}
	//		b_first = false;
	//		vector<vector<string>> s_temp_vecvec;
	//		s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter2.csv");
	//		th_nearest_nir = stof(s_temp_vecvec[1][3]);
	//		th_rank_rate_nir = stof(s_temp_vecvec[2][3]);
	//		th_nearest_velodyne = stof(s_temp_vecvec[3][3]);
	//		th_rank_rate_velodyne = stof(s_temp_vecvec[4][3]);
	//		th_rank_rate_fpfh = stof(s_temp_vecvec[5][3]);
	//		th_geometricConstraint = stof(s_temp_vecvec[6][3]);
	//		i_method_rigidTransformation = stoi(s_temp_vecvec[7][3]);
	//		b_useRigidTransformation = (bool)stoi(s_temp_vecvec[8][3]);
	//	}


	//	if (b_useOldFPFH)
	//	{
	//		DoOldFPFHRegistration(index_pair_vec);
	//	}
	//	else
	//	{
	//		DoFeatureRegistration(index_pair_vec, th_nearest_nir, th_rank_rate_nir,
	//			th_nearest_velodyne, th_rank_rate_velodyne,
	//			th_nearest_fpfh, num_nearest_fpfh, th_rank_rate_fpfh,
	//			i_method_rigidTransformation, th_geometricConstraint,
	//			b_useNir, b_useVelodyne, b_useFPFH,
	//			b_useGeometricConstraints);
	//	}

	//	//evaluation
	//	DoEvaluation(dir_, index_pair_vec, b_useOldFPFH);

	//	if(b_useRigidTransformation)
	//		showRigidTransformation(index_pair_vec);

	//	else if(b_show_AllFrame_AllPairs)
	//		showAllPairs(index_pair_vec, b_useNir, b_useVelodyne, b_useFPFH, b_useGeometricConstraints, b_useColorfullCorr);

	//	if (!b_useParameterAdjustment) break;
	//}

}

void CGlobalFeatureRegistration_test::inputData(string dir_, vector<float> parameter_oldFPFH_vec,  bool b_useNir, bool b_useVelodyne, bool b_changeColor_nir,
	bool b_useFPFH, bool b_useProposed)
{
	M_cloud_vec.clear();
	M_feature_vecvec_nir.clear();
	M_feature_vecvec_velodyne.clear();
	M_index_valid_vecvec_FPFH.clear();
	M_fpfh_vec.clear();
	M_transformation_vec.clear();
	M_trajectory_true_vec.clear();

	//input true trajectory
	{
		string filename_true = "transformation_fin.csv";
		vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
		for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
				trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
				trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
			M_trajectory_true_vec.push_back(Pos_temp);
		}
	}

	string s_folder;
	//what folder? -> "0:00_nir", "1:01_velodyne", or "2:03_all"
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
	//vector<pcl::PointCloud<T_PointType>::Ptr> cloud_vec;
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			M_cloud_vec.push_back(cloud);
		}
	}
	for (int j = 0; j < M_cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << M_cloud_vec[j]->size() << endl;
	cout << endl;

	//feature
	//vector<vector<float>> feature_vecvec_nir;
	//vector<vector<float>> feature_vecvec_velodyne;
	//if (b_useNir)
	//	for (int j = 0; j < M_cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < M_cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)M_cloud_vec[j]->points[i].r));
	//		M_feature_vecvec_nir.push_back(feature_vec);
	//	}
	//if (b_useVelodyne)
	//	for (int j = 0; j < M_cloud_vec.size(); j++)
	//	{
	//		vector<float> feature_vec;
	//		for (int i = 0; i < M_cloud_vec[j]->size(); i++)
	//			feature_vec.push_back((float)((int)M_cloud_vec[j]->points[i].g));
	//		M_feature_vecvec_velodyne.push_back(feature_vec);
	//	}

	for (int j = 0; j < M_cloud_vec.size(); j++)
	{
		bool b_onlyVelodyneData = false;
		int num_r255 = 0;
		for (int i = 0; i < M_cloud_vec[j]->size(); i++)
			if ((int)M_cloud_vec[j]->points[i].r == 255) num_r255++;
		if (num_r255 == M_cloud_vec[j]->size())  b_onlyVelodyneData = true;
		//velodyne
		{
			vector<float> feature_vec;
			if (b_useVelodyne)
			{
				for (int i = 0; i < M_cloud_vec[j]->size(); i++)
					feature_vec.push_back((float)((int)M_cloud_vec[j]->points[i].g));
			}
			M_feature_vecvec_velodyne.push_back(feature_vec);
		}
		//nir
		{
			vector<float> feature_vec;
			if (b_useNir && !b_onlyVelodyneData)
			{
				for (int i = 0; i < M_cloud_vec[j]->size(); i++)
					feature_vec.push_back((float)((int)M_cloud_vec[j]->points[i].r));
			}
			M_feature_vecvec_nir.push_back(feature_vec);
		}
	}

	if (!b_useNir)  b_changeColor_nir = false;
	if (b_changeColor_nir)
	{
		float value_max = -std::numeric_limits<float>::max();
		float value_min = std::numeric_limits<float>::max();
		for (int j = 0; j < M_feature_vecvec_nir.size(); j++)
		{
			for (int i = 0; i < M_feature_vecvec_nir[j].size(); i++)
			{
				if (value_max < M_feature_vecvec_nir[j][i]) value_max = M_feature_vecvec_nir[j][i];
				if (value_min > M_feature_vecvec_nir[j][i]) value_min = M_feature_vecvec_nir[j][i];
			}
		}
		//for (int j = 0; j < M_feature_vecvec_nir.size(); j++)
		//{
		//	for (int i = 0; i < M_feature_vecvec_nir[j].size(); i++)
		//	{
		//		vector<std::uint8_t> color_vec;
		//		color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(M_feature_vecvec_nir[j][i], value_max, value_min);
		//		T_PointType point_ = M_cloud_vec[j]->points[i];
		//		point_.r = color_vec[0];
		//		point_.g = color_vec[1];
		//		point_.b = color_vec[2];
		//		M_cloud_vec[j]->points[i] = point_;
		//	}
		//}
		for (int j = 0; j < M_feature_vecvec_nir.size(); j++)
		{
			vector<std::uint8_t> color_vec;
			if (M_feature_vecvec_nir[j].size() == 0)
			{
				color_vec.push_back((std::uint8_t)255);
				color_vec.push_back((std::uint8_t)0);
				color_vec.push_back((std::uint8_t)0);
				for (int i = 0; i < M_cloud_vec[j]->size(); i++)
				{
					T_PointType point_ = M_cloud_vec[j]->points[i];
					point_.r = color_vec[0];
					point_.g = color_vec[1];
					point_.b = color_vec[2];
					M_cloud_vec[j]->points[i] = point_;
				}
			}
			else
			{
				for (int i = 0; i < M_cloud_vec[j]->size(); i++)
				{
					color_vec = CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(M_feature_vecvec_nir[j][i], value_max, value_min);
					T_PointType point_ = M_cloud_vec[j]->points[i];
					point_.r = color_vec[0];
					point_.g = color_vec[1];
					point_.b = color_vec[2];
					M_cloud_vec[j]->points[i] = point_;
				}
			}
		}
	}

	//calc valid point of FPFH
	//vector<vector<int>> index_valid_vecvec_FPFH;
	//vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> fpfh_vec;
	if (b_useFPFH && b_useProposed)
	{
		float radius_normal_FPFH = parameter_oldFPFH_vec[1];
		//radius_normal_FPFH = 0.5;
		const pcl::search::KdTree<T_PointType>::Ptr kdtree_ne(new pcl::search::KdTree<T_PointType>);
		const auto view_point_ne = T_PointType(0.0, 10.0, 10.0);
		vector<pcl::PointCloud<pcl::Normal>::Ptr> normals_vec;
		for (int j = 0; j < M_cloud_vec.size(); j++)
		{
			const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
			ne->setInputCloud(M_cloud_vec[j]);
			ne->setRadiusSearch(radius_normal_FPFH);
			ne->setSearchMethod(kdtree_ne);
			ne->setViewPoint(view_point_ne.x, view_point_ne.y, view_point_ne.z);
			ne->compute(*normals);
			normals_vec.push_back(normals);
		}
		float radius_FPFH_center = parameter_oldFPFH_vec[2];;
		//radius_FPFH_center = 1.;
		//original
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius(cloud_vec, normals_vec, radius_FPFH_center, true);
		//output to file
		//index_valid_vecvec_FPFH = CFPFH_PCL::getFPFH_unique_someRadius_outputFile(cloud_vec, normals_vec, radius_FPFH_center, dir_, filenames_cloud, true);
		//input from file
		M_index_valid_vecvec_FPFH = CGlobalFeatureRegistration::getFPFH_unique_someRadius_inputFile(dir_, filenames_cloud);

		for (int j = 0; j < M_cloud_vec.size(); j++)
			M_fpfh_vec.push_back(CFPFH_PCL::computeFPFH(M_cloud_vec[j], M_cloud_vec[j], normals_vec[j], radius_FPFH_center));
	}
}

void CGlobalFeatureRegistration_test::DoOldFPFHRegistration(vector<pair<int, int>> index_pair_vec, vector<float> parameter_vec)
{
	float voxel_size;
	float radius_normal_FPFH, radius_FPFH;
	float MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
	int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;

	//voxel_size = 0.1;
	//radius_normal_FPFH = 0.5;
	//radius_FPFH = 1.;
	//MaxCorrespondenceDistance_SAC = 0.3;
	//SimilarityThreshold_SAC = 0.05;
	//InlierFraction_SAC = 0.2;
	//MaximumIterations_SAC = 500;
	//NumberOfSamples_SAC = 10;
	//CorrespondenceRandomness_SAC = 10;
	voxel_size = parameter_vec[0];
	radius_normal_FPFH = parameter_vec[1];
	radius_FPFH = parameter_vec[2];
	MaxCorrespondenceDistance_SAC = parameter_vec[3];
	SimilarityThreshold_SAC = parameter_vec[4];
	InlierFraction_SAC = parameter_vec[5];
	MaximumIterations_SAC = parameter_vec[6];
	NumberOfSamples_SAC = parameter_vec[7];
	CorrespondenceRandomness_SAC = parameter_vec[8];

	//int max_RANSAC;
	//max_RANSAC = 5;

	for (int j = 0; j < index_pair_vec.size(); j++)
	{
		int i_tgt, i_src;

		i_tgt = index_pair_vec[j].first;
		i_src = index_pair_vec[j].second;

		bool b_hasConverged = false;
		vector<int> inlier_;
		float fitnessscore;
		Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();

		cout << "i_tgt:" << i_tgt << " i_src" << i_src << endl;

		//compute fpfh
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());

		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
		{
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(M_cloud_vec[i_tgt]);
			sor->filter(*cloud_tgt);
			fpfh_tgt = CFPFH_PCL::computeFPFH<T_PointType>(cloud_tgt, M_cloud_vec[i_tgt], radius_normal_FPFH, radius_FPFH);
		}
		{
			const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>);
			sor->setLeafSize(voxel_size, voxel_size, voxel_size);
			sor->setInputCloud(M_cloud_vec[i_src]);
			sor->filter(*cloud_src);
			fpfh_src = CFPFH_PCL::computeFPFH<T_PointType>(cloud_src, M_cloud_vec[i_src], radius_normal_FPFH, radius_FPFH);
		}

		//b_hasConverged = CFPFH_PCL::align_SAC_AI_RANSAC<T_PointType>(transform_, inlier_, fitnessscore, frame_failed,
		//	M_cloud_vec[i_src], fpfh_src, M_cloud_vec[i_tgt], fpfh_tgt,
		//	voxel_size, MaxCorrespondenceDistance_SAC, SimilarityThreshold_SAC, InlierFraction_SAC,
		//	MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC, max_RANSAC, b_cout_RANSAC);

		boost::shared_ptr<pcl::PointCloud<T_PointType>> temp_(new pcl::PointCloud<T_PointType>);
		pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33> align;
		align.setInputSource(cloud_src);
		align.setSourceFeatures(fpfh_src);
		align.setInputTarget(cloud_tgt);
		align.setTargetFeatures(fpfh_tgt);
		align.setMaximumIterations(MaximumIterations_SAC);
		align.setNumberOfSamples(NumberOfSamples_SAC);
		align.setCorrespondenceRandomness(CorrespondenceRandomness_SAC);
		align.setSimilarityThreshold(SimilarityThreshold_SAC);				//th of corr rejecter
		align.setMaxCorrespondenceDistance(MaxCorrespondenceDistance_SAC);	//related to th of computing fitness score
		align.setInlierFraction(InlierFraction_SAC);						//th of inlier number
		//align->setMinSampleDistance(min_sample_distance_);	//function not found
		align.align(*temp_);

		//result
		transform_ = align.getFinalTransformation().cast<double>();
		inlier_ = align.getInliers();
		fitnessscore = align.getFitnessScore();

		b_hasConverged = align.hasConverged();

		cout << "transform_:" << endl;
		cout << transform_ << endl;
		M_transformation_vec.push_back(transform_);
	}

}

void CGlobalFeatureRegistration_test::DoFeatureRegistration(vector<pair<int, int>> index_pair_vec, vector<float> parameter_vec, 
	bool b_useNir, bool b_useVelodyne, bool b_useFPFH)
{
	M_corrs_all_vecvec.clear();
	M_evaluation_corr_vecvec_nir.clear();
	M_evaluation_corr_vecvec_velodyne.clear();
	M_evaluation_corr_vecvec_fpfh.clear();
	M_corrs_output_vec.clear();

	float th_nearest_nir;
	float th_rank_rate_nir;
	float th_nearest_velodyne;
	float th_rank_rate_velodyne;
	float th_nearest_fpfh;
	int num_nearest_fpfh;
	float th_rank_rate_fpfh;
	int i_method_rigidTransformation;
	float th_geometricConstraint;
	
	bool b_useGeometricConstraints;

	th_nearest_nir = parameter_vec[0];
	th_rank_rate_nir = parameter_vec[1];
	th_nearest_velodyne = parameter_vec[2];
	th_rank_rate_velodyne = parameter_vec[3];
	th_nearest_fpfh = parameter_vec[4];
	num_nearest_fpfh = (int)parameter_vec[5];
	th_rank_rate_fpfh = parameter_vec[6];
	i_method_rigidTransformation = (int)parameter_vec[7];
	th_geometricConstraint = parameter_vec[8];

	//if (parameter_vec[0] == 1.) b_useNir = true;
	//else b_useNir = false;
	//if (parameter_vec[0] == 1.) b_useVelodyne = true;
	//else b_useVelodyne = false;
	//if (parameter_vec[0] == 1.) b_useFPFH = true;
	//else b_useFPFH = false;
	//if (parameter_vec[0] == 1.) b_useGeometricConstraints = true;
	//else b_useGeometricConstraints = false;
	b_useNir = true;
	b_useVelodyne = true;
	b_useFPFH = true;
	b_useGeometricConstraints = true;

	cout << "calc pairs" << endl;
	if (b_useNir)
	{
		cout << "nir" << endl;
		M_corrs_nir_vec.clear();
		CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureScalar(M_feature_vecvec_nir, M_cloud_vec, index_pair_vec, th_nearest_nir, th_rank_rate_nir, M_corrs_nir_vec, M_evaluation_corr_vecvec_nir);
		for (int j = 0; j < M_corrs_nir_vec.size(); j++)
			if (M_corrs_nir_vec[j].size() == 0)
			{
				cout << "No correspondence found in ";
				cout << "i_tgt:" << index_pair_vec[j].first;
				cout << ", i_src:" << index_pair_vec[j].second << endl;;
			}
	}
	if (b_useVelodyne)
	{
		cout << "velodyne" << endl;
		M_corrs_velodyne_vec.clear();
		CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureScalar(M_feature_vecvec_velodyne, M_cloud_vec, index_pair_vec, th_nearest_velodyne, th_rank_rate_nir, M_corrs_velodyne_vec, M_evaluation_corr_vecvec_velodyne);
		for (int j = 0; j < M_corrs_velodyne_vec.size(); j++)
			if (M_corrs_velodyne_vec[j].size() == 0)
			{
				cout << "No correspondence found in ";
				cout << "i_tgt:" << index_pair_vec[j].first;
				cout << ", i_src:" << index_pair_vec[j].second << endl;;
			}
	}
	if (b_useFPFH)
	{
		cout << "fpfh" << endl;
		M_corrs_fpfh_vec.clear();
		CGlobalFeatureRegistration::determineCorrespondences_allFramesRanking_featureFpfh_remove(M_fpfh_vec, M_cloud_vec, index_pair_vec, th_nearest_fpfh, num_nearest_fpfh, th_rank_rate_fpfh,
			M_index_valid_vecvec_FPFH, M_corrs_fpfh_vec, M_evaluation_corr_vecvec_fpfh);
		for (int j = 0; j < M_corrs_fpfh_vec.size(); j++)
			if (M_corrs_fpfh_vec[j].size() == 0)
			{
				cout << "No correspondence found in ";
				cout << "i_tgt:" << index_pair_vec[j].first;
				cout << ", i_src:" << index_pair_vec[j].second << endl;;
			}
	}

	cout << "corr" << endl;
	for (int j = 0; j < M_corrs_nir_vec.size(); j++)
		cout << "j:" << j << " size:" << M_corrs_nir_vec[j].size() << endl;
	cout << "evaluation" << endl;
	for (int j = 0; j < M_evaluation_corr_vecvec_nir.size(); j++)
		cout << "j:" << j << " size:" << M_evaluation_corr_vecvec_nir[j].size() << endl;

	if (b_useGeometricConstraints) cout << "calc GeometricConstraints" << endl;
	for (int j = 0; j < index_pair_vec.size(); j++)
	{
		int i_tgt = index_pair_vec[j].first;
		int i_src = index_pair_vec[j].second;
		pcl::Correspondences corrs_temp;
		if (b_useNir) corrs_temp.insert(corrs_temp.end(), M_corrs_nir_vec[j].begin(), M_corrs_nir_vec[j].end());
		if (b_useVelodyne) corrs_temp.insert(corrs_temp.end(), M_corrs_velodyne_vec[j].begin(), M_corrs_velodyne_vec[j].end());
		if (b_useFPFH) corrs_temp.insert(corrs_temp.end(), M_corrs_fpfh_vec[j].begin(), M_corrs_fpfh_vec[j].end());
		cout << "i_tgt:" << i_tgt;
		cout << ", i_src:" << i_src << endl;
		if (b_useGeometricConstraints)
			M_corrs_all_vecvec.push_back(CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint(M_cloud_vec[i_src], M_cloud_vec[i_tgt], corrs_temp, th_geometricConstraint, true));
		else
		{
			vector<pcl::Correspondences> corrs_vec_temp;
			corrs_vec_temp.push_back(corrs_temp);
			M_corrs_all_vecvec.push_back(corrs_vec_temp);
		}
	}

	for (int j = 0; j < index_pair_vec.size(); j++)
	{
		int i_tgt = index_pair_vec[j].first;
		int i_src = index_pair_vec[j].second;
		M_corrs_output_vec.push_back(CGlobalFeatureRegistration::determineCorrespondences_geometricConstraint_evaluateCluster(
			M_cloud_vec[i_src], M_cloud_vec[i_tgt], M_corrs_all_vecvec[j], i_method_rigidTransformation));
	}
	for (int j = 0; j < index_pair_vec.size(); j++)
	{
		int i_tgt = index_pair_vec[j].first;
		int i_src = index_pair_vec[j].second;
		Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
		estimateRigidTransformation_static(M_cloud_vec[i_src], M_cloud_vec[i_tgt], M_corrs_output_vec[j], transformation_matrix);
		M_transformation_vec.push_back(transformation_matrix.cast<double>());
	}

}

void CGlobalFeatureRegistration_test::showAllPairs(vector<pair<int, int>> index_pair_vec, bool b_useNir, bool b_useVelodyne, bool b_useFPFH,
	bool b_useGeometricConstraints, bool b_useColorfullCorr)
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
			if (M_corrs_all_vecvec[index_frame_pair].size() > index_pair + 1)
			{
				index_pair++;
				//update pair
				b_updatePair = true;
			}
			else
			{
				if (M_corrs_all_vecvec.size() > index_frame_pair + 1)
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

		if (b_next_frame_pair && M_corrs_all_vecvec.size() > index_frame_pair + 1)
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
			cout << "(There are " << M_corrs_all_vecvec[index_frame_pair].size() << " correspondences)" << endl;
			pcl::copyPointCloud(*M_cloud_vec[i_src_vis], *cloud_src);
			pcl::copyPointCloud(*M_cloud_vec[i_tgt_vis], *cloud_tgt);
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
					for (int j = 0; j < M_evaluation_corr_vecvec_nir[index_frame_pair].size(); j++)
					{

						color_corr_vecvec.push_back(CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(
							M_evaluation_corr_vecvec_nir[index_frame_pair][j], M_evaluation_corr_vecvec_nir[index_frame_pair].back(),
							M_evaluation_corr_vecvec_nir[index_frame_pair][0]));
					}
				if (b_useVelodyne)
					for (int j = 0; j < M_evaluation_corr_vecvec_velodyne[index_frame_pair].size(); j++)
					{

						color_corr_vecvec.push_back(CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(
							M_evaluation_corr_vecvec_velodyne[index_frame_pair][j], M_evaluation_corr_vecvec_velodyne[index_frame_pair].back(),
							M_evaluation_corr_vecvec_velodyne[index_frame_pair][0]));
					}
				if (b_useFPFH)
					for (int j = 0; j < M_evaluation_corr_vecvec_fpfh[index_frame_pair].size(); j++)
					{

						color_corr_vecvec.push_back(CPointVisualization<T_PointType>::getRGBwithValuebyPseudoColor(
							M_evaluation_corr_vecvec_fpfh[index_frame_pair][j], M_evaluation_corr_vecvec_fpfh[index_frame_pair].back(),
							M_evaluation_corr_vecvec_fpfh[index_frame_pair][0]));
					}
			}

			if (M_corrs_all_vecvec[index_frame_pair].size() != 0)
			{
				cout << "index_pair:" << index_pair << endl;
				if (b_useGeometricConstraints || !b_useColorfullCorr) pv.drawCorrespondance(cloud_src, cloud_tgt, M_corrs_all_vecvec[index_frame_pair][index_pair], color_corr_vec);
				else pv.drawCorrespondance(cloud_src, cloud_tgt, M_corrs_all_vecvec[index_frame_pair][index_pair], color_corr_vecvec);
				cout << "showing " << M_corrs_all_vecvec[index_frame_pair][index_pair].size() << " pairs" << endl;
			}
			b_updatePair = false;
		}

		b_first_vis = false;

		if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) break;
		pv.updateViewer();
	}

	pv.closeViewer();

}

void CGlobalFeatureRegistration_test::showRigidTransformation(vector<pair<int, int>> index_pair_vec)
{
	vector<vector<float>> evaluation_corrsCluster_vecvec;
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
			pcl::copyPointCloud(*M_cloud_vec[i_tgt], *cloud_tgt);
			pcl::copyPointCloud(*M_cloud_vec[i_src], *cloud_src);
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
			pcl::transformPointCloud(*cloud_src, *cloud_src, M_transformation_vec[index_frame_pair]);
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

void CGlobalFeatureRegistration_test::fillParameterToTXT(vector<float> parameter_oldFPFH_vec, vector<float> parameter_featureRegistration_vec)
{
	//M_s_output_vecvec <- parameter_oldFPFH_vec
	//M_s_output_vecvec <- parameter_featureRegistration_vec

	{
		vector<string> s_vec;
		s_vec.push_back("Parameter_oldFPFH");
		M_s_output_vecvec.push_back(s_vec);
	}

	for (int j = 0; j < M_name_parameter_vec.size(); j++)
	{
		if (j == 9)
		{
			vector<string> s_vec;
			s_vec.push_back("Parameter_featureRegistration");
			s_vec.push_back("");
			s_vec.push_back("");
			s_vec.push_back("");
			M_s_output_vecvec.push_back(s_vec);
		}

		vector<string> s_vec;
		s_vec.push_back(M_name_parameter_vec[j]);
		s_vec.push_back("");
		s_vec.push_back("");
		s_vec.push_back("");

		if (j < 9)
		{
			if(j == 6 || j == 7 || j == 8)
				s_vec.push_back(to_string((int)parameter_oldFPFH_vec[j]));
			else
				s_vec.push_back(to_string(parameter_oldFPFH_vec[j]));
		}
		else
		{
			if(j - 9 == 5 || j - 9 == 7)
				s_vec.push_back(to_string((int)parameter_featureRegistration_vec[j - 9]));
			else 
				s_vec.push_back(to_string(parameter_featureRegistration_vec[j - 9]));
		}

		M_s_output_vecvec.push_back(s_vec);

	}

	//voxel_size = parameter_oldFPFH_vec[0];
	//radius_normal_FPFH = parameter_oldFPFH_vec[1];
	//radius_FPFH = parameter_oldFPFH_vec[2];
	//MaxCorrespondenceDistance_SAC = parameter_oldFPFH_vec[3];
	//SimilarityThreshold_SAC = parameter_oldFPFH_vec[4];
	//InlierFraction_SAC = parameter_oldFPFH_vec[5];
	//MaximumIterations_SAC = parameter_oldFPFH_vec[6];
	//NumberOfSamples_SAC = parameter_oldFPFH_vec[7];
	//CorrespondenceRandomness_SAC = parameter_oldFPFH_vec[8];

	//th_nearest_nir = parameter_featureRegistration_vec[0];
	//th_rank_rate_nir = parameter_featureRegistration_vec[1];
	//th_nearest_velodyne = parameter_featureRegistration_vec[2];
	//th_rank_rate_velodyne = parameter_featureRegistration_vec[3];
	//th_nearest_fpfh = parameter_featureRegistration_vec[4];
	//num_nearest_fpfh = (int)parameter_featureRegistration_vec[5];
	//th_rank_rate_fpfh = parameter_featureRegistration_vec[6];
	//i_method_rigidTransformation = (int)parameter_featureRegistration_vec[7];
	//th_geometricConstraint = parameter_featureRegistration_vec[8];
}

vector<pair<int, int>> CGlobalFeatureRegistration_test::getFramePairVec(string dir_)
{
	vector<pair<int, int>> frame_pair_vec;

	vector<vector<bool>> b_ignore_vecvec;
	vector<vector<bool>> b_ahead_vecvec;

	//input from text
	vector<vector<string>> s_matrix_vecvec;
	{
		//input text
		vector<vector<string>> s_matrix_vecvec_temp;
		s_matrix_vecvec_temp = CTimeString::getVecVecFromCSV_string(dir_ + "/matrix_ignore_ahead.csv");
		for (int j = 1; j < s_matrix_vecvec_temp.size(); j++)
		{
			vector<string> s_frame_vec;
			for (int i = 1; i < s_matrix_vecvec_temp[j].size(); i++)
			{
				s_frame_vec.push_back(s_matrix_vecvec_temp[j][i]);
			}
			s_matrix_vecvec.push_back(s_frame_vec);
		}

		//init ignore and ahead
		for (int j = 0; j < s_matrix_vecvec.size(); j++)
		{
			vector<bool> b_vecvec;
			for (int i = 0; i < s_matrix_vecvec.size(); i++)
				b_vecvec.push_back(false);
			b_ignore_vecvec.push_back(b_vecvec);
		}
		for (int j = 0; j < s_matrix_vecvec.size(); j++)
		{
			vector<bool> b_vecvec;
			for (int i = 0; i < s_matrix_vecvec.size(); i++)
				b_vecvec.push_back(false);
			b_ahead_vecvec.push_back(b_vecvec);
		}

		//extract from vecvec
		for (int i_tgt = 0; i_tgt < s_matrix_vecvec.size(); i_tgt++)
		{
			for (int i_src = 0; i_src < s_matrix_vecvec.size(); i_src++)
			{
				string s_value = s_matrix_vecvec[i_tgt][i_src];
				if (s_value.size() == 0 || s_value == "-")
					continue;
				else if (stoi(s_value) == -1)
				{
					b_ignore_vecvec[i_tgt][i_src] = true;
					cout << "i_tgt:" << i_tgt << " i_src:" << i_src << "  ignored" << endl;
				}
				else if (stoi(s_value) == 1)
				{
					b_ahead_vecvec[i_tgt][i_src] = true;
					cout << "i_tgt:" << i_tgt << " i_src:" << i_src << "  aheaded" << endl;
				}
			}
		}
	}

	//ahead loop
	for (int i_tgt = 0; i_tgt < s_matrix_vecvec.size() - 1; i_tgt++)
	{
		for (int i_src = i_tgt + 1; i_src < s_matrix_vecvec.size(); i_src++)
		{
			if (!b_ahead_vecvec[i_tgt][i_src]) continue;
			frame_pair_vec.push_back(make_pair(i_tgt, i_src));
			b_ignore_vecvec[i_tgt][i_src] = true;
		}
	}

	//normal loop
	for (int i_tgt = 0; i_tgt < s_matrix_vecvec.size() - 1; i_tgt++)
	{
		for (int i_src = i_tgt + 1; i_src < s_matrix_vecvec.size(); i_src++)
		{
			if (b_ignore_vecvec[i_tgt][i_src]) continue;
			frame_pair_vec.push_back(make_pair(i_tgt, i_src));
			b_ignore_vecvec[i_tgt][i_src] = true;
		}
	}

	cout << endl;
	return frame_pair_vec;

}

vector<vector<string>> CGlobalFeatureRegistration_test::DoEvaluation(string dir_save, vector<pair<int, int>> index_pair_vec, bool b_useProposed,
	bool b_useNir, bool b_useVelodyne, bool b_useFPFH, bool b_cout)
{
	cout << "DoEvaluation" << endl;
	vector<vector<string>> s_output_vecvec;
	{
		vector<string> s_output_vec;
		s_output_vec.push_back("i_tgt");
		s_output_vec.push_back("i_src");
		s_output_vec.push_back("isProposed");
		s_output_vec.push_back("b_usedNIR");
		s_output_vec.push_back("b_usedVelodyne");
		s_output_vec.push_back("b_usedFPFH");
		s_output_vec.push_back("X");
		s_output_vec.push_back("Y");
		s_output_vec.push_back("Z");
		s_output_vec.push_back("Roll");
		s_output_vec.push_back("Pitch");
		s_output_vec.push_back("Yaw");
		s_output_vec.push_back("isConverged");
		s_output_vec.push_back("corr_nir_size");
		s_output_vec.push_back("corr_velodyne_size");
		s_output_vec.push_back("corr_fpfh_size");
		s_output_vec.push_back("corr_output_size");
		s_output_vec.push_back("e_euqulid");
		s_output_vec.push_back("e_error_PointCloudDistance");
		s_output_vec.push_back("e_error_PointCloudDistance_median");
		s_output_vec.push_back("e_error_beta");
		s_output_vec.push_back("e_error_angle_normal");
		s_output_vec.push_back("mean_nearest");
		s_output_vec.push_back("median_nearest");
		s_output_vecvec.push_back(s_output_vec);
	}

	for (int j = 0; j < index_pair_vec.size(); j++)
	{
		vector<string> s_output_vec;
		int i_tgt = index_pair_vec[j].first;
		int i_src = index_pair_vec[j].second;


		Eigen::Matrix4d transformation_ = Eigen::Matrix4d::Identity();
		transformation_ = M_transformation_vec[j];


		Eigen::Matrix4d transformation_true = Eigen::Matrix4d::Identity();
		transformation_true =
			calcHomogeneousMatrixFromVector6d(M_trajectory_true_vec[i_tgt]).inverse() *
			calcHomogeneousMatrixFromVector6d(M_trajectory_true_vec[i_src]);

		float error_euqulid;
		error_euqulid = sqrt(
			pow(transformation_(0, 3) - transformation_true(0, 3), 2.)
			+ pow(transformation_(1, 3) - transformation_true(1, 3), 2.)
			+ pow(transformation_(2, 3) - transformation_true(2, 3), 2.)
		);

		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*M_cloud_vec[i_src], *cloud_src);
		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(transformation_);
			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);

		}
		pcl::PointCloud<T_PointType>::Ptr cloud_src_true(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*M_cloud_vec[i_src], *cloud_src_true);
		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(transformation_true);
			pcl::transformPointCloud(*cloud_src_true, *cloud_src_true, trans_);
		}

		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*M_cloud_vec[i_tgt], *cloud_tgt);

		double mean_nearest = getMeanDistanceVectorOfNearestPointCloud(cloud_src, cloud_tgt);
		double median_nearest = getMedianDistanceVectorOfNearestPointCloud(cloud_src, cloud_tgt);

		vector<float> error_PointCloudDistance_vec;
		for (int i = 0; i < cloud_src->size(); i++)
		{
			error_PointCloudDistance_vec.push_back(
				sqrt(
					pow(cloud_src->points[i].x - cloud_src_true->points[i].x, 2.)
					+ pow(cloud_src->points[i].y - cloud_src_true->points[i].y, 2.)
					+ pow(cloud_src->points[i].z - cloud_src_true->points[i].z, 2.))
			);
		}

		float error_PointCloudDistance = 0.;
		for (int i = 0; i < error_PointCloudDistance_vec.size(); i++)
			error_PointCloudDistance += error_PointCloudDistance_vec[i];
		if (error_PointCloudDistance_vec.size() != 0) error_PointCloudDistance 
			/= (float)error_PointCloudDistance_vec.size();

		float error_PointCloudDistance_median = 0.;
		error_PointCloudDistance_median = CTimeString::getMedian(error_PointCloudDistance_vec);

		float error_beta;
		float error_angle_normal;
		vector<float> error_angle_vec;
		//error_angle_vec = getAngleError(transformation_true, transformation_);
		error_angle_vec = getAngleError(transformation_, transformation_true);
		error_beta = error_angle_vec[0];
		error_angle_normal = error_angle_vec[1];

		Eigen::Vector6d pos_vec = Eigen::Vector6d::Zero();
		pos_vec = calcVector6dFromHomogeneousMatrix(transformation_);

		bool b_isConverged = true;
		{
			int num_zero = 0;
			for (int j = 0; j < pos_vec.rows(); j++)
			{
				if (pos_vec(j, 0) == 0.) num_zero++;
			}
			if (num_zero == 6) b_isConverged = false;
		}

		bool b_usedNIR = b_useNir;
		bool b_usedVelodyne = b_useVelodyne;
		bool b_usedFPFH = b_useFPFH;

		int corr_nir_size = 0;
		int corr_velodyne_size = 0;
		int corr_fpfh_size = 0;
		int corr_output_size = 0;
		if (!b_useProposed)
		{
			b_usedNIR = false;
			b_usedVelodyne = false;
			b_usedFPFH = true;
		}
		else
		{
			corr_nir_size = M_corrs_nir_vec[j].size();
			corr_velodyne_size = M_corrs_velodyne_vec[j].size();
			corr_fpfh_size = M_corrs_fpfh_vec[j].size();
			corr_output_size = M_corrs_output_vec[j].size();
			if (corr_nir_size == 0) b_usedNIR = false;
			if (corr_velodyne_size == 0) b_usedVelodyne = false;
			if (corr_fpfh_size == 0) b_usedFPFH = false;
		}

		if (b_cout)
		{
			cout << endl;
			cout << "i_tgt:" << i_tgt;
			cout << ", i_src:" << i_src << endl;

			cout << "transformation_:" << endl;
			cout << transformation_ << endl;

			cout << "transformation_true:" << endl;
			cout << transformation_true << endl;
		}

		s_output_vec.push_back(to_string(i_tgt));
		s_output_vec.push_back(to_string(i_src));
		s_output_vec.push_back(to_string((int)b_useProposed));
		s_output_vec.push_back(to_string((int)b_usedNIR));
		s_output_vec.push_back(to_string((int)b_usedVelodyne));
		s_output_vec.push_back(to_string((int)b_usedFPFH));
		s_output_vec.push_back(to_string(pos_vec(0, 0)));
		s_output_vec.push_back(to_string(pos_vec(1, 0)));
		s_output_vec.push_back(to_string(pos_vec(2, 0)));
		s_output_vec.push_back(to_string(pos_vec(3, 0)));
		s_output_vec.push_back(to_string(pos_vec(4, 0)));
		s_output_vec.push_back(to_string(pos_vec(5, 0)));
		s_output_vec.push_back(to_string((int)b_isConverged));
		s_output_vec.push_back(to_string(corr_nir_size));
		s_output_vec.push_back(to_string(corr_velodyne_size));
		s_output_vec.push_back(to_string(corr_fpfh_size));
		s_output_vec.push_back(to_string(corr_output_size));
		s_output_vec.push_back(to_string(error_euqulid));
		s_output_vec.push_back(to_string(error_PointCloudDistance));
		s_output_vec.push_back(to_string(error_PointCloudDistance_median));
		s_output_vec.push_back(to_string(error_beta));
		s_output_vec.push_back(to_string(error_angle_normal));
		s_output_vec.push_back(to_string(mean_nearest));			//mean of distance of nearest neighbor(after registration)
		s_output_vec.push_back(to_string(median_nearest));			//median of distance of nearest neighbor(after registration)

		pcl::PointCloud<T_PointType>::Ptr cloud_output(new pcl::PointCloud<T_PointType>());
		cloud_output->clear();
		pcl::copyPointCloud(*M_cloud_vec[i_tgt], *cloud_output);
		//tgt -> red
		for (int i = 0; i < cloud_output->size(); i++)
		{
			cloud_output->points[i].r = 255;
			cloud_output->points[i].g = 0;
			cloud_output->points[i].b = 0;
		}
		//src -> green
		for (int i = 0; i < cloud_src->size(); i++)
		{
			cloud_src->points[i].r = 0;
			cloud_src->points[i].g = 255;
			cloud_src->points[i].b = 0;
			cloud_output->push_back(cloud_src->points[i]);
		}

		string filename_pcd;
		{
			string s_tgt = to_string(i_tgt);
			if (s_tgt.size() < 2) s_tgt = "0" + s_tgt;
			s_tgt = "tgt" + s_tgt;
			string s_src = to_string(i_src);
			if (s_src.size() < 2) s_src = "0" + s_src;
			s_src = "src" + s_src;
			filename_pcd = s_tgt + s_src + "_result";

			if (!b_useProposed) filename_pcd += "_00conventional";
			else  filename_pcd += "_01proposed";
			filename_pcd += ".pcd";
		}

		if(b_isConverged) pcl::io::savePCDFile<T_PointType>(dir_save + "/" + filename_pcd, *cloud_output);
		s_output_vecvec.push_back(s_output_vec);
	}

	{
		vector<string> s_vec;
		s_output_vecvec.push_back(s_vec);
	}
	{
		vector<string> s_vec;
		s_vec.push_back("time_elapsed:");
		s_vec.push_back("");
		s_vec.push_back(M_t_elapsed);
		s_output_vecvec.push_back(s_vec);
	}

	return s_output_vecvec;
}

void CGlobalFeatureRegistration_test::alignAllFrames(string dir_, 
	vector<float> parameter_oldFPFH_vec, vector<float> parameter_featureRegistration_vec, int i_method)
{
	typedef pcl::PointXYZRGB T_PointType;

	//vector<vector<string>> s_output_vecvec;

	string s_folder = "Result_01varyParameters";

	bool b_show_AllFrame_AllPairs = false;
	bool b_useRigidTransformation = false;

	bool b_useColorfullCorr = false;

	bool b_changeColor_nir = false;
	bool b_changeColor_velodyne = false;

	bool b_useProposed = false;
	if (i_method == 0) b_useProposed = false;
	else if (i_method == 1) b_useProposed = true;

	bool b_useNir = false;
	bool b_useVelodyne = false;
	bool b_useFPFH = false;
	b_useNir = true;
	b_useVelodyne = true;
	b_useFPFH = true;

	b_changeColor_nir = true;

	//b_useRigidTransformation = true;

	//if (!b_useRigidTransformation) b_show_AllFrame_AllPairs = true;

	M_s_output_vecvec.clear();

	string t_start = CTimeString::getTimeString();

	inputData(dir_, parameter_oldFPFH_vec, b_useNir, b_useVelodyne, b_changeColor_nir, b_useFPFH, b_useProposed);

	fillParameterToTXT(parameter_oldFPFH_vec, parameter_featureRegistration_vec);

	cout << "M_s_output_vecvec:" << endl;
	for (int j = 0; j < M_s_output_vecvec.size(); j++)
	{
		for (int i = 0; i < M_s_output_vecvec[j].size(); i++)
			cout << M_s_output_vecvec[j][i] << "  ";
		cout << endl;
	}
	cout << endl;

	vector<pair<int, int>> index_pair_vec;

	//vector<pair<int, int>> getFramePairVec(string dir_)
	//index_pair_vec = getFramePairVec(dir_);

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
	for (int j = 0; j < M_cloud_vec.size() - 1; j++)
	{
		for (int i = j + 1; i < M_cloud_vec.size(); i++)
		{
			int i_tgt = j;
			int i_src = i;

			bool b_JValid = false;
			bool b_IValid = false;

			if (i_tgt == 0) b_JValid = true; if (i_src == 0) b_IValid = true;
			if (i_tgt == 1) b_JValid = true; if (i_src == 1) b_IValid = true;
			if (i_tgt == 2) b_JValid = true; if (i_src == 2) b_IValid = true;
			if (i_tgt == 3) b_JValid = true; if (i_src == 3) b_IValid = true;
			if (i_tgt == 4) b_JValid = true; if (i_src == 4) b_IValid = true;
			if (i_tgt == 5) b_JValid = true; if (i_src == 5) b_IValid = true;		//NIR
			if (i_tgt == 6) b_JValid = true; if (i_src == 6) b_IValid = true;		//NIR
			if (i_tgt == 7) b_JValid = true; if (i_src == 7) b_IValid = true;		//NIR
			if (i_tgt == 8) b_JValid = true; if (i_src == 8) b_IValid = true;		//NIR
			if (i_tgt == 9) b_JValid = true; if (i_src == 9) b_IValid = true;
			if (i_tgt == 10) b_JValid = true; if (i_src == 10) b_IValid = true;
			if (i_tgt == 11) b_JValid = true; if (i_src == 11) b_IValid = true;		//NIR
			if (i_tgt == 12) b_JValid = true; if (i_src == 12) b_IValid = true;		//NIR
			if (i_tgt == 13) b_JValid = true; if (i_src == 13) b_IValid = true;
			if (i_tgt == 14) b_JValid = true; if (i_src == 14) b_IValid = true;
			if (i_tgt == 15) b_JValid = true; if (i_src == 15) b_IValid = true;
			if (i_tgt == 16) b_JValid = true; if (i_src == 16) b_IValid = true;		//NIR

			////if (i_tgt == 0) b_JValid = true; if (i_src == 0) b_IValid = true;
			////if (i_tgt == 1) b_JValid = true; if (i_src == 1) b_IValid = true;
			////if (i_tgt == 2) b_JValid = true; if (i_src == 2) b_IValid = true;
			////if (i_tgt == 3) b_JValid = true; if (i_src == 3) b_IValid = true;
			////if (i_tgt == 4) b_JValid = true; if (i_src == 4) b_IValid = true;
			//if (i_tgt == 5) b_JValid = true; if (i_src == 5) b_IValid = true;		//NIR
			//if (i_tgt == 6) b_JValid = true; if (i_src == 6) b_IValid = true;		//NIR
			//if (i_tgt == 7) b_JValid = true; if (i_src == 7) b_IValid = true;		//NIR
			//if (i_tgt == 8) b_JValid = true; if (i_src == 8) b_IValid = true;		//NIR
			////if (i_tgt == 9) b_JValid = true; if (i_src == 9) b_IValid = true;
			////if (i_tgt == 10) b_JValid = true; if (i_src == 10) b_IValid = true;
			//if (i_tgt == 11) b_JValid = true; if (i_src == 11) b_IValid = true;		//NIR
			//if (i_tgt == 12) b_JValid = true; if (i_src == 12) b_IValid = true;		//NIR
			////if (i_tgt == 13) b_JValid = true; if (i_src == 13) b_IValid = true;
			////if (i_tgt == 14) b_JValid = true; if (i_src == 14) b_IValid = true;
			////if (i_tgt == 15) b_JValid = true; if (i_src == 15) b_IValid = true;
			//if (i_tgt == 16) b_JValid = true; if (i_src == 16) b_IValid = true;		//NIR

			////if (i_tgt == 0) b_JValid = true; if (i_src == 0) b_IValid = true;
			////if (i_tgt == 1) b_JValid = true; if (i_src == 1) b_IValid = true;
			////if (i_tgt == 2) b_JValid = true; if (i_src == 2) b_IValid = true;
			////if (i_tgt == 3) b_JValid = true; if (i_src == 3) b_IValid = true;
			////if (i_tgt == 4) b_JValid = true; if (i_src == 4) b_IValid = true;
			//if (i_tgt == 5) b_JValid = true; if (i_src == 5) b_IValid = true;		//NIR
			//if (i_tgt == 6) b_JValid = true; if (i_src == 6) b_IValid = true;		//NIR
			//if (i_tgt == 7) b_JValid = true; if (i_src == 7) b_IValid = true;		//NIR
			////if (i_tgt == 8) b_JValid = true; if (i_src == 8) b_IValid = true;		//NIR
			////if (i_tgt == 9) b_JValid = true; if (i_src == 9) b_IValid = true;
			////if (i_tgt == 10) b_JValid = true; if (i_src == 10) b_IValid = true;
			//if (i_tgt == 11) b_JValid = true; if (i_src == 11) b_IValid = true;		//NIR
			////if (i_tgt == 12) b_JValid = true; if (i_src == 12) b_IValid = true;		//NIR
			////if (i_tgt == 13) b_JValid = true; if (i_src == 13) b_IValid = true;
			////if (i_tgt == 14) b_JValid = true; if (i_src == 14) b_IValid = true;
			////if (i_tgt == 15) b_JValid = true; if (i_src == 15) b_IValid = true;
			////if (i_tgt == 16) b_JValid = true; if (i_src == 16) b_IValid = true;		//NIR

			//if (i_tgt == 0) b_JValid = true; if (i_src == 0) b_IValid = true;
			////if (i_tgt == 1) b_JValid = true; if (i_src == 1) b_IValid = true;
			////if (i_tgt == 2) b_JValid = true; if (i_src == 2) b_IValid = true;
			////if (i_tgt == 3) b_JValid = true; if (i_src == 3) b_IValid = true;
			////if (i_tgt == 4) b_JValid = true; if (i_src == 4) b_IValid = true;
			//if (i_tgt == 5) b_JValid = true; if (i_src == 5) b_IValid = true;		//NIR
			//if (i_tgt == 6) b_JValid = true; if (i_src == 6) b_IValid = true;		//NIR
			//if (i_tgt == 7) b_JValid = true; if (i_src == 7) b_IValid = true;		//NIR
			////if (i_tgt == 8) b_JValid = true; if (i_src == 8) b_IValid = true;		//NIR
			////if (i_tgt == 9) b_JValid = true; if (i_src == 9) b_IValid = true;
			////if (i_tgt == 10) b_JValid = true; if (i_src == 10) b_IValid = true;
			////if (i_tgt == 11) b_JValid = true; if (i_src == 11) b_IValid = true;		//NIR
			////if (i_tgt == 12) b_JValid = true; if (i_src == 12) b_IValid = true;		//NIR
			////if (i_tgt == 13) b_JValid = true; if (i_src == 13) b_IValid = true;
			////if (i_tgt == 14) b_JValid = true; if (i_src == 14) b_IValid = true;
			////if (i_tgt == 15) b_JValid = true; if (i_src == 15) b_IValid = true;
			////if (i_tgt == 16) b_JValid = true; if (i_src == 16) b_IValid = true;		//NIR

			if (!b_JValid) continue;
			if (!b_IValid) continue;

			index_pair_vec.push_back(make_pair(i_tgt, i_src));
		}
	}

	cout << "index_pair_vec:" << endl;
	for (int j = 0; j < index_pair_vec.size(); j++)
		cout << "i_tgt:" << index_pair_vec[j].first << ", i_src:" << index_pair_vec[j].second << endl;
	cout << endl;

	//float th_nearest_nir;
	//float th_rank_rate_nir;
	//float th_nearest_velodyne;
	//float th_rank_rate_velodyne;
	//float th_nearest_fpfh;
	//int num_nearest_fpfh;
	//float th_rank_rate_fpfh;
	//int i_method_rigidTransformation;
	//float th_geometricConstraint;


	//th_nearest_nir = 10.;
	////th_rank_rate_nir = 0.5;
	////th_rank_rate_nir = 1.;
	//th_rank_rate_nir = 0.2;
	////cout << "input th_nearest_nir:";
	////cin >> th_nearest_nir;
	//th_nearest_velodyne = 10.;
	////th_rank_rate_velodyne = 0.5;
	//th_rank_rate_velodyne = 1.;

	//th_nearest_fpfh = 1800.;
	//num_nearest_fpfh = 10;
	////th_rank_rate_fpfh = 0.5;
	//th_rank_rate_fpfh = 0.5;

	//if (b_useParameterAdjustment)
	//{
	//	if (!b_first)
	//	{
	//		int aa;
	//		cout << "input txt:";
	//		cin >> aa;
	//	}
	//	b_first = false;
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter2.csv");
	//	th_nearest_nir = stof(s_temp_vecvec[1][3]);
	//	th_rank_rate_nir = stof(s_temp_vecvec[2][3]);
	//	th_nearest_velodyne = stof(s_temp_vecvec[3][3]);
	//	th_rank_rate_velodyne = stof(s_temp_vecvec[4][3]);
	//	th_rank_rate_fpfh = stof(s_temp_vecvec[5][3]);
	//	th_geometricConstraint = stof(s_temp_vecvec[6][3]);
	//	i_method_rigidTransformation = stoi(s_temp_vecvec[7][3]);
	//	b_useRigidTransformation = (bool)stoi(s_temp_vecvec[8][3]);
	//}


	//th_geometricConstraint = 0.8;

	bool b_useGeometricConstraints = false;
	b_useGeometricConstraints = true;

	//if (b_useParameterAdjustment)
	//{
	//	if (!b_first)
	//	{
	//		int aa;
	//		cout << "input txt:";
	//		cin >> aa;
	//	}
	//	b_first = false;
	//	vector<vector<string>> s_temp_vecvec;
	//	s_temp_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/parameter2.csv");
	//	th_nearest_nir = stof(s_temp_vecvec[1][3]);
	//	th_rank_rate_nir = stof(s_temp_vecvec[2][3]);
	//	th_nearest_velodyne = stof(s_temp_vecvec[3][3]);
	//	th_rank_rate_velodyne = stof(s_temp_vecvec[4][3]);
	//	th_rank_rate_fpfh = stof(s_temp_vecvec[5][3]);
	//	th_geometricConstraint = stof(s_temp_vecvec[6][3]);
	//	i_method_rigidTransformation = stoi(s_temp_vecvec[7][3]);
	//	b_useRigidTransformation = (bool)stoi(s_temp_vecvec[8][3]);
	//}


	if (!b_useProposed)
		DoOldFPFHRegistration(index_pair_vec, parameter_oldFPFH_vec);
	else
		DoFeatureRegistration(index_pair_vec, parameter_featureRegistration_vec, b_useNir, b_useVelodyne, b_useFPFH);
	cout << endl;
	//result: M_transformation_vec

	string t_end = CTimeString::getTimeString();
	string t_elapsed = CTimeString::getTimeElapsefrom2Strings(t_start, t_end);
	cout << "t_elapsed(all frames):" << t_elapsed << endl;
	M_t_elapsed = t_elapsed;


	//s_output_vecvec <- Result
	{
		vector<string> s_vec;
		M_s_output_vecvec.push_back(s_vec);
	}
	{
		vector<string> s_vec;
		s_vec.push_back("Result");
		M_s_output_vecvec.push_back(s_vec);
	}

	string s_newfoldername = CTimeString::getTimeString();
	if (!b_useProposed)
		s_newfoldername += "_conventional";
	else
		s_newfoldername += "_proposed";

	//makenewfolder
	CTimeString::makenewfolder(dir_ + "/" + s_folder, s_newfoldername);
	//evaluation
	vector<vector<string>> s_value_vecvec;
	s_value_vecvec = DoEvaluation(dir_ + "/" + s_folder + "/" + s_newfoldername, index_pair_vec, b_useProposed, b_useNir, b_useVelodyne, b_useFPFH);

	M_s_output_vecvec.insert(M_s_output_vecvec.end(), s_value_vecvec.begin(), s_value_vecvec.end());

	////regular saving csv
	//string s_elapsed_frame = CTimeString::getTimeElapsefrom2Strings(time_regular, time_end_frame);
	//cout << "time_elapsed from last .csv output: " << s_elapsed_frame << endl;
	//cout << "time_elapsed from start:            " << CTimeString::getTimeElapsefrom2Strings(time_start, time_end_frame) << endl;
	//int elapsed_millisec = CTimeString::getTimeElapsefrom2Strings_millisec(time_regular, time_end_frame);
	//int elapsed_minute = (int)(((float)elapsed_millisec / 1000.) / 60.);
	//if (elapsed_minute >= th_minute_CSV)
	//{
	//	//save
	//	CTimeString::getCSVFromVecVec(M_s_output_vecvec, dir_ + "/" + s_newfoldername + "/" + time_regular + "_output.csv");
	//	time_regular = CTimeString::getTimeString();
	//	//clear s_output_vecvec
	//	M_s_output_vecvec.clear();
	//	GR_addToOutputString_OutputHeader_FPFH(M_s_output_vecvec);
	//}
	//cout << endl;

	//if (i_frame_pair % 5 == 0 && !b_changeParameter)
	//{
	//	cout << "Parameter list" << endl;
	//	CTimeString::showParameter(parameter_vec, M_name_parameter_vec);
	//	cout << endl;
	//}


	CTimeString::getCSVFromVecVec(M_s_output_vecvec, dir_ + "/" + s_folder + "/" + s_newfoldername + "/" + s_newfoldername + "_output.csv");

	if (b_useRigidTransformation)
		showRigidTransformation(index_pair_vec);

	else if (b_show_AllFrame_AllPairs)
		showAllPairs(index_pair_vec, b_useNir, b_useVelodyne, b_useFPFH, b_useGeometricConstraints, b_useColorfullCorr);

}

void CGlobalFeatureRegistration_test::variParamaters(string dir_)
{
	bool b_create_new_pattern_file = false;
	cout << "do you create new pattern?  Yes:1  No:0" << endl;
	cout << "->";
	cin >> b_create_new_pattern_file;

	//M_name_parameter_vec
	M_name_parameter_vec.clear();
	M_name_parameter_vec.push_back("voxel_size");						//0
	M_name_parameter_vec.push_back("radius_normal_FPFH");				//1
	M_name_parameter_vec.push_back("radius_FPFH");						//2
	M_name_parameter_vec.push_back("MaxCorrespondenceDistance_SAC");	//3
	M_name_parameter_vec.push_back("SimilarityThreshold_SAC");			//4
	M_name_parameter_vec.push_back("InlierFraction_SAC");				//5
	M_name_parameter_vec.push_back("MaximumIterations_SAC");			//6
	M_name_parameter_vec.push_back("NumberOfSamples_SAC");				//7
	M_name_parameter_vec.push_back("CorrespondenceRandomness_SAC");		//8
	//
	M_name_parameter_vec.push_back("th_nearest_nir");					//0
	M_name_parameter_vec.push_back("th_rank_rate_nir");					//1
	M_name_parameter_vec.push_back("th_nearest_velodyne");				//2
	M_name_parameter_vec.push_back("th_rank_rate_velodyne");			//3
	M_name_parameter_vec.push_back("th_nearest_fpfh");					//4
	M_name_parameter_vec.push_back("num_nearest_fpfh");					//5
	M_name_parameter_vec.push_back("th_rank_rate_fpfh");				//6
	M_name_parameter_vec.push_back("i_method_rigidTransformation");		//7
	M_name_parameter_vec.push_back("th_geometricConstraint");			//8

	if (b_create_new_pattern_file)
	{
		vector<vector<float>> pattern_vecvec_new;
		//input parameter
		vector<vector<float>> parameter_vecvec;
		//CTimeString::changeParameter_2dimension(parameter_vec_vec, name_parameter_vec, parameter_vec_arg);
		CTimeString::changeParameter_2dimension(parameter_vecvec, M_name_parameter_vec, 
			dir_ + "/Result_01varyParameters/parameter_vecvec.csv", 1, 4, -1, -1);
		pattern_vecvec_new = CTimeString::calcVectorPairPattern(parameter_vecvec);
		//write new parameter_vec_vec
		{
			vector<vector<string>> s_vec_vec;
			//header
			{
				vector<string> s_header_vec;
				//s_header_vec.push_back("Parameter");
				for (int i = 0; i < M_name_parameter_vec.size(); i++)
					s_header_vec.push_back(M_name_parameter_vec[i]);
				s_vec_vec.push_back(s_header_vec);
			}
			for (int j = 0; j < pattern_vecvec_new.size(); j++)
			{
				vector<string> s_vec;
				for (int i = 0; i < pattern_vecvec_new[j].size(); i++)
					s_vec.push_back(to_string(pattern_vecvec_new[j][i]));
				s_vec_vec.push_back(s_vec);
			}
			CTimeString::getCSVFromVecVec(s_vec_vec, dir_ + "/Result_01varyParameters/pattern_vecvec.csv");
		}
	}

	cout << "press 1 and Enter if you have closed file" << endl;
	{
		int aa;
		cin >> aa;
	}

	//read pattern_vec_vec.csv
	vector<vector<float>> pattern_vec_vec;
	{
		vector<vector<string>> s_vec_vec;
		s_vec_vec = CTimeString::getVecVecFromCSV_string(dir_ + "/Result_01varyParameters/pattern_vecvec.csv");
		for (int j = 1; j < s_vec_vec.size(); j++)
		{
			vector<float> pattern_vec;
			for (int i = 0; i < s_vec_vec[j].size(); i++)
				pattern_vec.push_back(stof(s_vec_vec[j][i]));
			pattern_vec_vec.push_back(pattern_vec);
		}
	}
	cout << "show pattern" << endl;
	for (int j = 0; j < pattern_vec_vec.size(); j++)
	{
		cout << j << ":";
		for (int i = 0; i < pattern_vec_vec[j].size(); i++)
		{
			string s_value;
			s_value = to_string(pattern_vec_vec[j][i]);
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			cout << "  " << s_value;
		}
		cout << endl;
	}

	for (int j = 0; j < pattern_vec_vec.size(); j++)
	{
		vector<float> parameter_vec = pattern_vec_vec[j];
		vector<float> parameter_oldFPFH_vec;//
		vector<float> parameter_featureRegistration_vec;
		parameter_oldFPFH_vec.insert(parameter_oldFPFH_vec.end(), parameter_vec.begin(), parameter_vec.begin() + 9);	//0~8
		parameter_featureRegistration_vec.insert(parameter_featureRegistration_vec.end(), parameter_vec.begin() + 9, parameter_vec.end());	//9~last
		cout << "start pattern:" << j << endl;
		//alignAllFrames(dir_, parameter_oldFPFH_vec, parameter_featureRegistration_vec, 0);
		alignAllFrames(dir_, parameter_oldFPFH_vec, parameter_featureRegistration_vec, 1);
	}

	cout << endl;

}

void CGlobalFeatureRegistration_test::compareGlobalRegistration(string dir_)
{
	M_name_parameter_vec.clear();

	bool b_isGotParameterName = false;
	bool b_isProposed = false;

	//read folder name
	vector<string> s_folder_vec;
	CTimeString::getFileNames_folder(dir_, s_folder_vec);
	for (int j = s_folder_vec.size() - 1; j >= 0; j--)
		if (s_folder_vec[j] == "_Ignore") s_folder_vec.erase(s_folder_vec.begin() + j);

	vector<vector<string>> s_output_vecvec;
	for (int j = 0; j < s_folder_vec.size(); j++)
	{
		int frame_max = 0;

		//read text
		vector<vector<string>> s_txt_vecvec;
		{
			vector<string> s_filename_vec;
			CTimeString::getFileNames_extension(dir_ + "/" + s_folder_vec[j], s_filename_vec, ".csv");
			if (s_filename_vec.size() == 0)
			{
				cout << "ERROR: No .csv found and continuing." << endl;
				continue;
			}
			s_txt_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder_vec[j] + "/" + s_filename_vec[0]);
			int i_find = s_filename_vec[0].find("conventional");
			if (i_find == std::string::npos) b_isProposed = true;

		}

		//get information of parameter
		vector<float> parameter_vec;
		{
			vector<vector<string>> s_vecvec_temp =
				CTimeString::getMatrixData_fromSpecificAreaOfMatrix(s_txt_vecvec, "Parameter_oldFPFH", 1, "Result", -2, 1);
			for (int i = 0; i < s_vecvec_temp.size(); i++)
			{
				if (i == 9) continue;
				if (!b_isGotParameterName)
					M_name_parameter_vec.push_back(s_vecvec_temp[i][0]);
				parameter_vec.push_back(stof(s_vecvec_temp[i][4]));
			}
			if (M_name_parameter_vec.size() != 0) b_isGotParameterName = true;
		}

		//get information of result
		vector<vector<string>> s_result_vecvec;
		s_result_vecvec = CTimeString::getMatrixData_fromSpecificAreaOfMatrix(s_txt_vecvec, "Result", 2, "time_elapsed:", -2, 0);

		//estimation
		vector<vector<int>> framePair_success_vecvec;
		for (int i = 0; i < s_result_vecvec.size(); i++)
		{
			int i_tgt, i_src;
			i_tgt = stoi(s_result_vecvec[i][0]);
			i_src = stoi(s_result_vecvec[i][1]);
			if (i_src > frame_max) frame_max = i_src;
			bool b_isConverged = (bool)(stoi(s_result_vecvec[i][12]));
			float e_error_PointCloudDistance = stof(s_result_vecvec[i][18]);

			//float th_successOfGlobalRegistration_distance = 8.;
			float th_successOfGlobalRegistration_distance = 1.5;
			bool b_estimatedSuccess = false;
			if (e_error_PointCloudDistance < th_successOfGlobalRegistration_distance && b_isConverged) b_estimatedSuccess = true;
			if (b_estimatedSuccess)
			{
				vector<int> framePair_success_vec;
				framePair_success_vec.push_back(i_tgt);
				framePair_success_vec.push_back(i_src);
				framePair_success_vecvec.push_back(framePair_success_vec);
			}
		}

		//calc cluster size
		vector<vector<int>> cluster_vecvec;
		cluster_vecvec = CTimeString::getIntCluster_SomeToSome(framePair_success_vecvec);

		//output to string
		vector<string> s_output_vec;
		s_output_vec.push_back(s_folder_vec[j]);
		for (int i = 0; i < parameter_vec.size(); i++)
			s_output_vec.push_back(to_string(parameter_vec[i]));

		//b_isProposed
		s_output_vec.push_back(to_string((int)b_isProposed));

		int num_allFramePairs = s_result_vecvec.size();
		int num_succeededFramePairs = framePair_success_vecvec.size();
		s_output_vec.push_back(to_string(num_allFramePairs));
		s_output_vec.push_back(to_string(num_succeededFramePairs));

		//succeededFramePairs
		{
			string s_output = "";
			for (int i = 0; i < framePair_success_vecvec.size(); i++)
			{
				s_output +=
					to_string(framePair_success_vecvec[i][0]) + "-"
					+ to_string(framePair_success_vecvec[i][1]) + " ";
			}
			s_output_vec.push_back(s_output);
		}

		//biggestCluster
		if (cluster_vecvec.size() > 0)
		{
			string s_output;
			for (int i = 0; i < cluster_vecvec[0].size(); i++)
				s_output += to_string(cluster_vecvec[0][i]) + " ";
			s_output_vec.push_back(s_output);
		}

		//size_biggestCluster
		if (cluster_vecvec.size() > 0)
			s_output_vec.push_back(to_string(cluster_vecvec[0].size()));

		//second_biggestCluster
		if (cluster_vecvec.size() > 1)
		{
			string s_output;
			for (int i = 0; i < cluster_vecvec[1].size(); i++)
				s_output += to_string(cluster_vecvec[1][i]) + " ";
			s_output_vec.push_back(s_output);
		}

		//frames_notContainded
		//need debug
		vector<int> frames_notContainded_vec;
		frames_notContainded_vec.clear();
		for (int i = 0; i <= frame_max; i++)
			frames_notContainded_vec.push_back(i);
		if (cluster_vecvec.size() > 0)
		{
			//cout << "frame_max:" << frame_max << endl;
			for (int i = cluster_vecvec[0].size() - 1; i >= 0; i--)
				frames_notContainded_vec.erase(frames_notContainded_vec.begin() + cluster_vecvec[0][i]);
		}
		{
			//for (int i = 0; i < cluster_vecvec[0].size(); i++)
			//	cout << cluster_vecvec[0][i] << " ";
			//cout << endl;
			//for (int i = 0; i < frames_notContainded_vec.size(); i++)
			//	cout << frames_notContainded_vec[i] << " ";
			//cout << endl;
			string s_output = "";
			for (int i = 0; i < frames_notContainded_vec.size(); i++)
				s_output += to_string(frames_notContainded_vec[i]) + " ";
			s_output_vec.push_back(s_output);
		}


		//calc matrix?

		s_output_vecvec.push_back(s_output_vec);
		cout << endl;
	}

	//header
	{
		vector<string> s_output_vec;
		s_output_vec.push_back("");
		for (int j = 0; j < M_name_parameter_vec.size(); j++)
			s_output_vec.push_back(M_name_parameter_vec[j]);
		s_output_vec.push_back("b_isProposed");
		s_output_vec.push_back("num_allFramePairs");
		s_output_vec.push_back("num_succeededFramePairs");
		s_output_vec.push_back("succeededFramePairs");
		s_output_vec.push_back("biggestCluster");
		s_output_vec.push_back("size_biggestCluster");
		s_output_vec.push_back("second_biggestCluster");
		s_output_vec.push_back("frames_notContainded");
		s_output_vecvec.insert(s_output_vecvec.begin(), s_output_vec);
	}

	//output
	vector<vector<string>> s_output_vecvec_transposed;
	s_output_vecvec_transposed = CTimeString::getTranspositionOfVecVec(s_output_vecvec);
	CTimeString::getCSVFromVecVec(s_output_vecvec_transposed, dir_ + "/" + CTimeString::getTimeString() + "_comparison.csv");
}

void CGlobalFeatureRegistration_test::inputData_ICP(string dir_)
{
	M_cloud_vec.clear();
	M_transformation_vec.clear();
	M_trajectory_true_vec.clear();
	M_chara_vecvec.clear();
	M_b_isConverged_vec.clear();
	M_b_estimatedSuccess_vec.clear();

	string s_folder;
	//what folder? -> "0:00_nir", "1:01_velodyne", or "2:03_all"
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
	vector<string> filenames_cloud;
	{
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder, filenames_cloud, ".pcd");
		for (int i = 0; i < filenames_cloud.size(); i++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>());
			pcl::io::loadPCDFile(dir_ + "/" + s_folder + "/" + filenames_cloud[i], *cloud);
			cloud->is_dense = true;
			M_cloud_vec.push_back(cloud);
		}
	}
	for (int j = 0; j < M_cloud_vec.size(); j++)
		cout << "j:" << j << " cloud_vec[j]->size():" << M_cloud_vec[j]->size() << endl;
	cout << endl;

	//input true trajectory
	{
		string filename_true = "transformation_fin.csv";
		vector<vector<double>> trajectory_vecvec_temp = CTimeString::getVecVecFromCSV(dir_ + "/" + filename_true);
		for (int i = 0; i < trajectory_vecvec_temp.size(); i++)
		{
			Eigen::Vector6d Pos_temp = Eigen::Vector6d::Zero();
			Pos_temp << trajectory_vecvec_temp[i][1], trajectory_vecvec_temp[i][2],
				trajectory_vecvec_temp[i][3], trajectory_vecvec_temp[i][4],
				trajectory_vecvec_temp[i][5], trajectory_vecvec_temp[i][6];
			M_trajectory_true_vec.push_back(Pos_temp);
		}
	}

	//input character
	for (int i = 0; i < M_cloud_vec.size(); i++)
	{
		vector<int> chara_vec;
		chara_vec = CExtendableICP::ICP_Chara_GetCharaData(M_cloud_vec[i]);
		M_chara_vecvec.push_back(chara_vec);
	}

}

void CGlobalFeatureRegistration_test::inputData_ICP_initPos(string dir_, bool b_calcOnlyBiggestCluster, float th_successOfGlobalRegistration_distance, bool &b_isProposed)
{
	M_initPos_vec.clear();

	b_isProposed = false;

	//read text
	vector<vector<string>> s_txt_vecvec;
	{
		vector<string> s_filename_vec;
		CTimeString::getFileNames_extension(dir_, s_filename_vec, ".csv");
		if (s_filename_vec.size() == 0)
		{
			cout << "ERROR: No .csv found and continuing." << endl;
			return;
		}
		s_txt_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_filename_vec[0]);
		int i_find = s_filename_vec[0].find("conventional");
		if (i_find == std::string::npos) b_isProposed = true;
	}

	//input initPos
	{
		vector<vector<string>> s_input_vecvec;
		s_input_vecvec = CTimeString::getMatrixData_fromSpecificAreaOfMatrix(s_txt_vecvec, "Result", 2, "time_elapsed:", -2, 23);
		for (int j = 0; j < s_input_vecvec.size(); j++)
		{
			bool b_isConverged = false;
			b_isConverged =(bool)stoi(s_input_vecvec[j][12]);
			float e_error_PointCloudDistance = stof(s_input_vecvec[j][18]);
			bool b_success = false;
			if (e_error_PointCloudDistance < th_successOfGlobalRegistration_distance && b_isConverged) b_success = true;
			SInitPos_ICP initPos_;
			if (b_success)
			{
				int i_tgt = stoi(s_input_vecvec[j][0]);
				int i_src = stoi(s_input_vecvec[j][1]);
				initPos_.i_tgt = i_tgt;
				initPos_.i_src = i_src;
				initPos_.Init_Vector <<
					stod(s_input_vecvec[j][6])
					, stod(s_input_vecvec[j][7])
					, stod(s_input_vecvec[j][8])
					, stod(s_input_vecvec[j][9])
					, stod(s_input_vecvec[j][10])
					, stod(s_input_vecvec[j][11]);
				M_initPos_vec.push_back(initPos_);
			}
		}
	}

	if (!b_calcOnlyBiggestCluster) return;

	vector<int> frames_all;
	for (int j = 0; j < M_cloud_vec.size(); j++)
		frames_all.push_back(j);

	//calc frames that should be deleted
	vector<vector<int>> value_vecvec;

	for (int j = 0; j < M_initPos_vec.size(); j++)
	{
		vector<int> value_vec;
		value_vec.push_back(M_initPos_vec[j].i_tgt);
		value_vec.push_back(M_initPos_vec[j].i_src);
		value_vecvec.push_back(value_vec);
	}
	cout << "before" << endl;
	for (int j = 0; j < value_vecvec.size(); j++)
	{
		cout << "j:" << j;
		for (int i = 0; i < value_vecvec[j].size(); i++)
			cout << " " << value_vecvec[j][i];
		cout << endl;
	}
	cout << endl;

	vector<vector<int>> value_vecvec_new;
	value_vecvec_new = CTimeString::getIntCluster_SomeToSome(value_vecvec);
	cout << "after" << endl;
	for (int j = 0; j < value_vecvec_new.size(); j++)
	{
		for (int i = 0; i < value_vecvec_new[j].size(); i++)
			cout << " " << value_vecvec_new[j][i];
		cout << endl;
	}
	cout << endl;

	//not contained to bigger cluster
	vector<bool> b_frames_isInBiggerCluster_vec;
	b_frames_isInBiggerCluster_vec.resize(M_cloud_vec.size());
	fill(b_frames_isInBiggerCluster_vec.begin(), b_frames_isInBiggerCluster_vec.end(), false);
	for (int j = 0; j < value_vecvec_new[0].size(); j++)
		b_frames_isInBiggerCluster_vec[value_vecvec_new[0][j]] = true;

	for (int j = 0; j < b_frames_isInBiggerCluster_vec.size(); j++)
		if (!b_frames_isInBiggerCluster_vec[j]) cout << "frame:" << j << "is not contained." << endl;

	int index_new = 0;
	for (int j = 0; j < M_cloud_vec.size(); j++)
	{
		if (b_frames_isInBiggerCluster_vec[j])
		{
			frames_all.push_back(index_new);
			index_new++;
		}
		else frames_all.push_back(-1);
	}

	//cout << "show deleted frames(-1)" << endl;
	//for (int i = 0; i < frames_all.size(); i++)
	//	cout << "i:" << i << " " << frames_all[i] << endl;

	//M_initPos_vec
	for (int j = M_initPos_vec.size() - 1; j >= 0; j--)
	{
		if (b_frames_isInBiggerCluster_vec[M_initPos_vec[j].i_tgt]
			&& b_frames_isInBiggerCluster_vec[M_initPos_vec[j].i_src]) continue;
		M_initPos_vec.erase(M_initPos_vec.begin() + j);
	}

}

void CGlobalFeatureRegistration_test::fillParameterToTXT_ICP(vector<float> parameter_vec)
{
	//M_s_output_vecvec <- parameter_vec
	{
		vector<string> s_vec;
		s_vec.push_back("Parameter_ICP");
		M_s_output_vecvec.push_back(s_vec);
	}

	for (int j = 0; j < M_name_parameter_vec.size(); j++)
	{
		vector<string> s_vec;
		s_vec.push_back(M_name_parameter_vec[j]);
		s_vec.push_back("");
		s_vec.push_back("");
		s_vec.push_back("");
		if (j == 0) s_vec.push_back(to_string((int)parameter_vec[j]));
		else s_vec.push_back(to_string(parameter_vec[j]));
		M_s_output_vecvec.push_back(s_vec);

	}

	//M_name_parameter_vec.clear();
	//M_name_parameter_vec.push_back("MaximumIterations");							//0
	//M_name_parameter_vec.push_back("MaxCorrespondenceDistance");					//1
	//M_name_parameter_vec.push_back("EuclideanFitnessEpsilon");					//2
	//M_name_parameter_vec.push_back("TransformationEpsilon");						//3
	//M_name_parameter_vec.push_back("penalty_chara");								//4
	//M_name_parameter_vec.push_back("weight_dist_chara");							//5
	//M_name_parameter_vec.push_back("th_successOfGlobalRegistration_distance");	//6
}

vector<vector<string>> CGlobalFeatureRegistration_test::DoEvaluation_ICP(string dir_save, float th_successOfICP_distance, bool b_useProposed, bool b_cout)
{
	cout << "DoEvaluation_ICP" << endl;
	vector<vector<string>> s_output_vecvec;
	{
		vector<string> s_output_vec;
		s_output_vec.push_back("i_tgt");
		s_output_vec.push_back("i_src");
		s_output_vec.push_back("isProposed");
		//s_output_vec.push_back("b_usedNIR");
		//s_output_vec.push_back("b_usedVelodyne");
		//s_output_vec.push_back("b_usedFPFH");
		s_output_vec.push_back("X");
		s_output_vec.push_back("Y");
		s_output_vec.push_back("Z");
		s_output_vec.push_back("Roll");
		s_output_vec.push_back("Pitch");
		s_output_vec.push_back("Yaw");
		s_output_vec.push_back("isConverged");
		//s_output_vec.push_back("corr_nir_size");
		//s_output_vec.push_back("corr_velodyne_size");
		//s_output_vec.push_back("corr_fpfh_size");
		//s_output_vec.push_back("corr_output_size");
		s_output_vec.push_back("e_euqulid");
		s_output_vec.push_back("e_error_PointCloudDistance");
		s_output_vec.push_back("e_error_PointCloudDistance_median");
		s_output_vec.push_back("e_error_beta");
		s_output_vec.push_back("e_error_angle_normal");
		s_output_vec.push_back("mean_nearest");
		s_output_vec.push_back("median_nearest");
		s_output_vec.push_back("b_estimatedSuccess");
		s_output_vecvec.push_back(s_output_vec);
	}

	for (int j = 0; j < M_initPos_vec.size(); j++)
	{
		vector<string> s_output_vec;
		int i_tgt = M_initPos_vec[j].i_tgt;
		int i_src = M_initPos_vec[j].i_src;

		Eigen::Matrix4d transformation_init = Eigen::Matrix4d::Identity();
		transformation_init = calcHomogeneousMatrixFromVector6d(M_initPos_vec[j].Init_Vector);
		Eigen::Matrix4d transformation_ = Eigen::Matrix4d::Identity();
		transformation_ = M_transformation_vec[j] * transformation_init;

		Eigen::Matrix4d transformation_true = Eigen::Matrix4d::Identity();
		transformation_true =
			calcHomogeneousMatrixFromVector6d(M_trajectory_true_vec[i_tgt]).inverse() *
			calcHomogeneousMatrixFromVector6d(M_trajectory_true_vec[i_src]);

		float error_euqulid;
		error_euqulid = sqrt(
			pow(transformation_(0, 3) - transformation_true(0, 3), 2.)
			+ pow(transformation_(1, 3) - transformation_true(1, 3), 2.)
			+ pow(transformation_(2, 3) - transformation_true(2, 3), 2.)
		);

		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src_true(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src_init(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());

		//cloud_src_init
		pcl::copyPointCloud(*M_cloud_vec[i_src], *cloud_src_init);
		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(transformation_init);
			pcl::transformPointCloud(*cloud_src_init, *cloud_src_init, trans_);
		}
		//cloud_src
		pcl::copyPointCloud(*M_cloud_vec[i_src], *cloud_src);
		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(transformation_);
			pcl::transformPointCloud(*cloud_src, *cloud_src, trans_);
		}
		//cloud_src_true
		pcl::copyPointCloud(*M_cloud_vec[i_src], *cloud_src_true);
		{
			Eigen::Affine3f trans_ = calcAffine3fFromHomogeneousMatrix(transformation_true);
			pcl::transformPointCloud(*cloud_src_true, *cloud_src_true, trans_);
		}
		//cloud_tgt
		pcl::copyPointCloud(*M_cloud_vec[i_tgt], *cloud_tgt);

		double mean_nearest = getMeanDistanceVectorOfNearestPointCloud(cloud_src, cloud_tgt);
		double median_nearest = getMedianDistanceVectorOfNearestPointCloud(cloud_src, cloud_tgt);

		vector<float> error_PointCloudDistance_vec;
		for (int i = 0; i < cloud_src->size(); i++)
		{
			error_PointCloudDistance_vec.push_back(
				sqrt(
					pow(cloud_src->points[i].x - cloud_src_true->points[i].x, 2.)
					+ pow(cloud_src->points[i].y - cloud_src_true->points[i].y, 2.)
					+ pow(cloud_src->points[i].z - cloud_src_true->points[i].z, 2.))
			);
		}

		float error_PointCloudDistance = 0.;
		for (int i = 0; i < error_PointCloudDistance_vec.size(); i++)
			error_PointCloudDistance += error_PointCloudDistance_vec[i];
		if (error_PointCloudDistance_vec.size() != 0) error_PointCloudDistance
			/= (float)error_PointCloudDistance_vec.size();

		float error_PointCloudDistance_median = 0.;
		error_PointCloudDistance_median = CTimeString::getMedian(error_PointCloudDistance_vec);

		float error_beta;
		float error_angle_normal;
		vector<float> error_angle_vec;
		//error_angle_vec = getAngleError(transformation_true, transformation_);
		error_angle_vec = getAngleError(transformation_, transformation_true);
		error_beta = error_angle_vec[0];
		error_angle_normal = error_angle_vec[1];

		Eigen::Vector6d pos_vec = Eigen::Vector6d::Zero();
		pos_vec = calcVector6dFromHomogeneousMatrix(transformation_);

		bool b_isConverged = M_b_isConverged_vec[j];

		//estimation
		bool b_estimatedSuccess = false;
		if (error_PointCloudDistance < th_successOfICP_distance) b_estimatedSuccess = true;
		M_b_estimatedSuccess_vec.push_back(b_estimatedSuccess);

		if (b_cout)
		{
			cout << endl;
			cout << "i_tgt:" << i_tgt;
			cout << ", i_src:" << i_src << endl;

			cout << "transformation_:" << endl;
			cout << transformation_ << endl;

			cout << "transformation_true:" << endl;
			cout << transformation_true << endl;
		}

		s_output_vec.push_back(to_string(i_tgt));
		s_output_vec.push_back(to_string(i_src));
		s_output_vec.push_back(to_string((int)b_useProposed));
		//s_output_vec.push_back(to_string((int)b_usedNIR));
		//s_output_vec.push_back(to_string((int)b_usedVelodyne));
		//s_output_vec.push_back(to_string((int)b_usedFPFH));
		s_output_vec.push_back(to_string(pos_vec(0, 0)));
		s_output_vec.push_back(to_string(pos_vec(1, 0)));
		s_output_vec.push_back(to_string(pos_vec(2, 0)));
		s_output_vec.push_back(to_string(pos_vec(3, 0)));
		s_output_vec.push_back(to_string(pos_vec(4, 0)));
		s_output_vec.push_back(to_string(pos_vec(5, 0)));
		s_output_vec.push_back(to_string((int)b_isConverged));
		//s_output_vec.push_back(to_string(corr_nir_size));
		//s_output_vec.push_back(to_string(corr_velodyne_size));
		//s_output_vec.push_back(to_string(corr_fpfh_size));
		//s_output_vec.push_back(to_string(corr_output_size));
		s_output_vec.push_back(to_string(error_euqulid));
		s_output_vec.push_back(to_string(error_PointCloudDistance));
		s_output_vec.push_back(to_string(error_PointCloudDistance_median));
		s_output_vec.push_back(to_string(error_beta));
		s_output_vec.push_back(to_string(error_angle_normal));
		s_output_vec.push_back(to_string(mean_nearest));			//mean of distance of nearest neighbor(after registration)
		s_output_vec.push_back(to_string(median_nearest));			//median of distance of nearest neighbor(after registration)
		s_output_vec.push_back(to_string((int)b_estimatedSuccess));

		pcl::PointCloud<T_PointType>::Ptr cloud_output(new pcl::PointCloud<T_PointType>());
		cloud_output->clear();
		pcl::copyPointCloud(*cloud_tgt, *cloud_output);
		//tgt -> red
		for (int i = 0; i < cloud_output->size(); i++)
		{
			cloud_output->points[i].r = 255;
			cloud_output->points[i].g = 0;
			cloud_output->points[i].b = 0;
		}
		//cloud_src -> green
		for (int i = 0; i < cloud_src->size(); i++)
		{
			cloud_src->points[i].r = 0;
			cloud_src->points[i].g = 255;
			cloud_src->points[i].b = 0;
			cloud_output->push_back(cloud_src->points[i]);
		}

		//cloud_src_init -> green + white
		for (int i = 0; i < cloud_src_init->size(); i++)
		{
			cloud_src_init->points[i].r = 100;
			cloud_src_init->points[i].g = 255;
			cloud_src_init->points[i].b = 100;
			cloud_output->push_back(cloud_src_init->points[i]);
		}

		string filename_pcd;
		{
			string s_tgt = to_string(i_tgt);
			if (s_tgt.size() < 2) s_tgt = "0" + s_tgt;
			s_tgt = "tgt" + s_tgt;
			string s_src = to_string(i_src);
			if (s_src.size() < 2) s_src = "0" + s_src;
			s_src = "src" + s_src;
			filename_pcd = s_tgt + s_src + "_result";

			if (!b_useProposed) filename_pcd += "_00conventional_ICP";
			else  filename_pcd += "_01proposed_ICP";
			filename_pcd += ".pcd";
		}

		if (b_isConverged) pcl::io::savePCDFile<T_PointType>(dir_save + "/" + filename_pcd, *cloud_output);
		s_output_vecvec.push_back(s_output_vec);
	}

	{
		vector<string> s_vec;
		s_output_vecvec.push_back(s_vec);
	}
	{
		vector<string> s_vec;
		s_vec.push_back("time_elapsed:");
		s_vec.push_back("");
		s_vec.push_back(M_t_elapsed);
		s_output_vecvec.push_back(s_vec);
	}

	return s_output_vecvec;
}

vector<vector<string>> CGlobalFeatureRegistration_test::calcBiggestFrameCluster_ICP()
{
	vector<vector<string>> s_output_vec;
	{
		vector<string> s_vec;
		s_output_vec.push_back(s_vec);
	}

	//estimation cluster
	vector<vector<int>> framePair_success_vecvec;
	for (int j = 0; j < M_initPos_vec.size(); j++)
	{
		if (!M_b_estimatedSuccess_vec[j]) continue;
		vector<int> framePair_success_vec;
		framePair_success_vec.push_back(M_initPos_vec[j].i_tgt);
		framePair_success_vec.push_back(M_initPos_vec[j].i_src);
		framePair_success_vecvec.push_back(framePair_success_vec);
	}
	//calc cluster size
	vector<vector<int>> cluster_vecvec;
	cluster_vecvec = CTimeString::getIntCluster_SomeToSome(framePair_success_vecvec);

	//num_allFramePairs
	int num_allFramePairs = M_initPos_vec.size();
	{
		vector<string> s_vec;
		s_vec.push_back("num_allFramePairs:");
		s_vec.push_back(to_string(num_allFramePairs));
		s_output_vec.push_back(s_vec);
	}

	//num_succeededFramePairs
	int num_succeededFramePairs = framePair_success_vecvec.size();
	{
		vector<string> s_vec;
		s_vec.push_back("num_succeededFramePairs:");
		s_vec.push_back(to_string(num_succeededFramePairs));
		s_output_vec.push_back(s_vec);
	}

	//succeededFramePairs
	{
		string s_output = "";
		for (int j = 0; j < framePair_success_vecvec.size(); j++)
		{
			s_output +=
				to_string(framePair_success_vecvec[j][0]) + "-"
				+ to_string(framePair_success_vecvec[j][1]) + " ";
		}
		vector<string> s_vec;
		s_vec.push_back("succeededFramePairs:");
		s_vec.push_back(s_output);
		s_output_vec.push_back(s_vec);
	}

	//biggestCluster
	{
		string s_output = "";
		vector<string> s_vec;
		s_vec.push_back("biggestCluster:");
		if (cluster_vecvec.size() > 0)
		{
			for (int j = 0; j < cluster_vecvec[0].size(); j++)
				s_output += to_string(cluster_vecvec[0][j]) + " ";
		}
		s_vec.push_back(s_output);
		s_output_vec.push_back(s_vec);
	}

	//size_biggestCluster
	{
		vector<string> s_vec;
		s_vec.push_back("size_biggestCluster:");
		if (cluster_vecvec.size() > 0)
			s_vec.push_back(to_string(cluster_vecvec[0].size()));
		s_output_vec.push_back(s_vec);

	}

	//second_biggestCluster
	{
		string s_output = "";
		vector<string> s_vec;
		s_vec.push_back("second_biggestCluster:");
		if (cluster_vecvec.size() > 1)
		{
			for (int j = 0; j < cluster_vecvec[1].size(); j++)
				s_output += to_string(cluster_vecvec[1][j]) + " ";
		}
		s_vec.push_back(s_output);
		s_output_vec.push_back(s_vec);
	}

	//frames_notContainded
	//need debug
	{
		int frame_max = M_cloud_vec.size() - 1;
		vector<int> frames_notContainded_vec;
		frames_notContainded_vec.clear();
		for (int i = 0; i <= frame_max; i++)
			frames_notContainded_vec.push_back(i);
		string s_output = "";
		if (cluster_vecvec.size() > 0)
		{
			//cout << "frame_max:" << frame_max << endl;
			for (int j = cluster_vecvec[0].size() - 1; j >= 0; j--)
				frames_notContainded_vec.erase(frames_notContainded_vec.begin() + cluster_vecvec[0][j]);
		}
		for (int j = 0; j < frames_notContainded_vec.size(); j++)
			s_output += to_string(frames_notContainded_vec[j]) + " ";
		vector<string> s_vec;
		s_vec.push_back("frames_notContainded:");
		s_vec.push_back(s_output);
		s_output_vec.push_back(s_vec);
	}

	return s_output_vec;
}

void CGlobalFeatureRegistration_test::align_ICP_AllFrames(string dir_, string s_folder_arg, vector<float> parameter_vec)
{
	typedef pcl::PointXYZRGB T_PointType;

	string s_folder = "Result_02_ICP_varyParameters/_Input";
	string s_folder_output = "Result_02_ICP_varyParameters";

	M_s_output_vecvec.clear();

	string t_start = CTimeString::getTimeString();

	inputData_ICP(dir_);

	bool b_isProposed_arg;
	inputData_ICP_initPos(dir_ + "/" + s_folder + "/" + s_folder_arg, 
		true, parameter_vec[6], b_isProposed_arg);
	//void inputData_ICP_initPos(string dir_, bool b_calcOnlyBiggestCluster, float th_successOfGlobalRegistration_distance, bool &b_isProposed)

	fillParameterToTXT_ICP(parameter_vec);

	//s_output_vecvec <- filename_GlobalRegistration:
	{
		vector<string> s_vec;
		s_vec.push_back("filename_GlobalRegistration:");
		s_vec.push_back("");
		s_vec.push_back("");
		s_vec.push_back("");
		s_vec.push_back(s_folder_arg);
		M_s_output_vecvec.push_back(s_vec);
	}

	cout << "M_s_output_vecvec:" << endl;
	for (int j = 0; j < M_s_output_vecvec.size(); j++)
	{
		for (int i = 0; i < M_s_output_vecvec[j].size(); i++)
			cout << M_s_output_vecvec[j][i] << "  ";
		cout << endl;
	}
	cout << endl;

	//loop by folder
	for (int j = 0; j < M_initPos_vec.size(); j++)
	{
		pcl::PointCloud<T_PointType>::Ptr cloud_temp(new pcl::PointCloud<T_PointType>());

		int i_tgt = M_initPos_vec[j].i_tgt;
		int i_src = M_initPos_vec[j].i_src;

		pcl::PointCloud<T_PointType>::Ptr cloud_tgt(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src(new pcl::PointCloud<T_PointType>());
		pcl::PointCloud<T_PointType>::Ptr cloud_src_transformed(new pcl::PointCloud<T_PointType>());
		pcl::copyPointCloud(*M_cloud_vec[i_tgt], *cloud_tgt);
		pcl::copyPointCloud(*M_cloud_vec[i_src], *cloud_src);

		//transform src by InitPos
		{
			Eigen::Affine3f Trans_temp = Eigen::Affine3f::Identity();
			Trans_temp = calcAffine3fFromHomogeneousMatrix(
				calcHomogeneousMatrixFromVector6d(M_initPos_vec[j].Init_Vector));
			pcl::transformPointCloud(*cloud_src, *cloud_src_transformed, Trans_temp);
		}

		string time_start_frame = CTimeString::getTimeString();

		//vector<int> inlier_;
		float fitnessscore;
		//int frame_failed = 0;
		//Eigen::Matrix4d transform_ = Eigen::Matrix4d::Identity();
		//bool b_cout_RANSAC = false;
		Eigen::Vector6d Registration_Vector = Eigen::Vector6d::Zero();

		cout << "i_tgt:" << i_tgt << " i_src:" << i_src << endl;

		//parameter
		int MaximumIterations;
		float MaxCorrespondenceDistance, EuclideanFitnessEpsilon, TransformationEpsilon;
		MaximumIterations = (int)parameter_vec[0];
		MaxCorrespondenceDistance = parameter_vec[1];
		EuclideanFitnessEpsilon = parameter_vec[2];
		TransformationEpsilon = parameter_vec[3];
		//property
		float penalty_chara, weight_dist_chara;
		penalty_chara = parameter_vec[4];
		weight_dist_chara = parameter_vec[5];

		if (!b_isProposed_arg)
		{
			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> align_ICP;
			//parameter
			align_ICP.setMaximumIterations(MaximumIterations);
			align_ICP.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
			align_ICP.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
			align_ICP.setTransformationEpsilon(TransformationEpsilon);
			//data
			align_ICP.setInputTarget(cloud_tgt);
			align_ICP.setInputSource(cloud_src_transformed);
			//align
			align_ICP.align(*cloud_temp);
			M_b_isConverged_vec.push_back(align_ICP.hasConverged());
			//Registration_Vector = calcVector6dFromHomogeneousMatrix(
			//	align_ICP.getFinalTransformation().cast<double>());
			M_transformation_vec.push_back(align_ICP.getFinalTransformation().cast<double>());
		}
		else
		{
			CExtendableICP align_ICP_proposed;
			//parameter
			align_ICP_proposed.setMothodInt(1);
			align_ICP_proposed.setMaximumIterations(MaximumIterations);
			align_ICP_proposed.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
			align_ICP_proposed.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
			align_ICP_proposed.setTransformationEpsilon(TransformationEpsilon);
			align_ICP_proposed.setCharaParameter(penalty_chara, weight_dist_chara);
			//data
			align_ICP_proposed.setInputTarget(cloud_tgt);
			align_ICP_proposed.setInputSource(cloud_src_transformed);
			align_ICP_proposed.setCharaVector_tgt(M_chara_vecvec[i_tgt]);
			align_ICP_proposed.setCharaVector_src(M_chara_vecvec[i_src]);
			//align
			align_ICP_proposed.align();
			M_b_isConverged_vec.push_back(align_ICP_proposed.hasConverged());
			//Registration_Vector = align_ICP_proposed.getFinalTransformation_Vec();
			M_transformation_vec.push_back(align_ICP_proposed.getFinalTransformation().cast<double>());
		}

	}

	string t_end = CTimeString::getTimeString();
	string t_elapsed = CTimeString::getTimeElapsefrom2Strings(t_start, t_end);
	cout << "t_elapsed(all frames):" << t_elapsed << endl;
	M_t_elapsed = t_elapsed;

	//M_s_output_vecvec <- Result
	{
		vector<string> s_vec;
		M_s_output_vecvec.push_back(s_vec);
	}
	{
		vector<string> s_vec;
		s_vec.push_back("Result_ICP");
		M_s_output_vecvec.push_back(s_vec);
	}

	string s_newfoldername = CTimeString::getTimeString();
	if (!b_isProposed_arg)
		s_newfoldername += "_conventional";
	else
		s_newfoldername += "_proposed";

	s_newfoldername += "_ICP";

	//makenewfolder
	CTimeString::makenewfolder(dir_ + "/" + s_folder_output, s_newfoldername);
	//evaluation
	float th_successOfICP_distance = 1.;
	vector<vector<string>> s_value_vecvec;
	s_value_vecvec = DoEvaluation_ICP(dir_ + "/" + s_folder_output + "/" + s_newfoldername, th_successOfICP_distance, b_isProposed_arg, false);
	M_s_output_vecvec.insert(M_s_output_vecvec.end(), s_value_vecvec.begin(), s_value_vecvec.end());

	//th_successOfICP_distance
	{
		vector<string> s_vec;
		M_s_output_vecvec.push_back(s_vec);
	}
	{
		vector<string> s_vec;
		s_vec.push_back("th_successOfICP_distance:");
		s_vec.push_back(to_string(th_successOfICP_distance));
		M_s_output_vecvec.push_back(s_vec);
	}

	vector<vector<string>> s_cluster_vecvec = calcBiggestFrameCluster_ICP();
	M_s_output_vecvec.insert(M_s_output_vecvec.end(), s_cluster_vecvec.begin(), s_cluster_vecvec.end());

	//M_s_output_vecvec <-(front) inputed file
	vector<vector<string>> s_input_vecvec;
	{
		vector<string> s_filename_vec;
		CTimeString::getFileNames_extension(dir_ + "/" + s_folder + "/" + s_folder_arg, s_filename_vec, ".csv");
		if (s_filename_vec.size() == 0)
		{
			cout << "ERROR: No .csv found and continuing." << endl;
			return;
		}
		s_input_vecvec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder + "/" + s_folder_arg + "/" + s_filename_vec[0]);
	}
	{
		vector<string> s_vec;
		s_input_vecvec.push_back(s_vec);
	}
	M_s_output_vecvec.insert(M_s_output_vecvec.begin(), s_input_vecvec.begin(), s_input_vecvec.end());

	CTimeString::getCSVFromVecVec(M_s_output_vecvec, dir_ + "/" + s_folder_output + "/" + s_newfoldername + "/" + s_newfoldername + "_output.csv");
}

void CGlobalFeatureRegistration_test::align_ICP_fromGlobalRegistration_variParamaters(string dir_)
{
	bool b_create_new_pattern_file = false;
	cout << "do you create new pattern?  Yes:1  No:0" << endl;
	cout << "->";
	cin >> b_create_new_pattern_file;

	string s_folder = "Result_02_ICP_varyParameters/_Input";

	//M_name_parameter_vec
	M_name_parameter_vec.clear();
	M_name_parameter_vec.push_back("MaximumIterations");						//0
	M_name_parameter_vec.push_back("MaxCorrespondenceDistance");				//1
	M_name_parameter_vec.push_back("EuclideanFitnessEpsilon");					//2
	M_name_parameter_vec.push_back("TransformationEpsilon");					//3
	M_name_parameter_vec.push_back("penalty_chara");							//4
	M_name_parameter_vec.push_back("weight_dist_chara");						//5
	M_name_parameter_vec.push_back("th_successOfGlobalRegistration_distance");	//6

	if (b_create_new_pattern_file)
	{
		vector<vector<float>> pattern_vecvec_new;
		//input parameter
		vector<vector<float>> parameter_vecvec;
		CTimeString::changeParameter_2dimension(parameter_vecvec, M_name_parameter_vec,
			dir_ + "/" + s_folder + "/parameter_vecvec.csv", 1, 4, -1, -1);
		pattern_vecvec_new = CTimeString::calcVectorPairPattern(parameter_vecvec);
		//write new parameter_vec_vec
		{
			vector<vector<string>> s_vec_vec;
			//header
			{
				vector<string> s_header_vec;
				//s_header_vec.push_back("Parameter");
				for (int i = 0; i < M_name_parameter_vec.size(); i++)
					s_header_vec.push_back(M_name_parameter_vec[i]);
				s_vec_vec.push_back(s_header_vec);
			}
			for (int j = 0; j < pattern_vecvec_new.size(); j++)
			{
				vector<string> s_vec;
				for (int i = 0; i < pattern_vecvec_new[j].size(); i++)
					s_vec.push_back(to_string(pattern_vecvec_new[j][i]));
				s_vec_vec.push_back(s_vec);
			}
			CTimeString::getCSVFromVecVec(s_vec_vec, dir_ + "/" + s_folder + "/pattern_vecvec.csv");
		}
	}

	cout << "press 1 and Enter if you have closed file" << endl;
	{
		int aa;
		cin >> aa;
	}

	//read pattern_vec_vec.csv
	vector<vector<float>> pattern_vec_vec;
	{
		vector<vector<string>> s_vec_vec;
		s_vec_vec = CTimeString::getVecVecFromCSV_string(dir_ + "/" + s_folder + "/pattern_vecvec.csv");
		for (int j = 1; j < s_vec_vec.size(); j++)
		{
			vector<float> pattern_vec;
			for (int i = 0; i < s_vec_vec[j].size(); i++)
				pattern_vec.push_back(stof(s_vec_vec[j][i]));
			pattern_vec_vec.push_back(pattern_vec);
		}
	}
	cout << "show pattern" << endl;
	for (int j = 0; j < pattern_vec_vec.size(); j++)
	{
		cout << j << ":";
		for (int i = 0; i < pattern_vec_vec[j].size(); i++)
		{
			string s_value;
			s_value = to_string(pattern_vec_vec[j][i]);
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			if (s_value.size() < 4) s_value = " " + s_value;
			cout << "  " << s_value;
		}
		cout << endl;
	}

	//read folder name
	vector<string> s_folder_vec;
	CTimeString::getFileNames_folder(dir_ + "/" + s_folder, s_folder_vec);
	for (int j = s_folder_vec.size() - 1; j >= 0; j--)
		if (s_folder_vec[j] == "_Ignore") s_folder_vec.erase(s_folder_vec.begin() + j);

	for (int j = 0; j < s_folder_vec.size(); j++)
	{
		for (int i = 0; i < pattern_vec_vec.size(); i++)
		{
			vector<float> parameter_vec = pattern_vec_vec[i];
			cout << "start pattern:" << j << endl;
			align_ICP_AllFrames(dir_, s_folder_vec[j], pattern_vec_vec[i]);

		}
	}

	cout << endl;
}
