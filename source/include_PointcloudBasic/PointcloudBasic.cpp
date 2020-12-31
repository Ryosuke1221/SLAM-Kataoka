#include "PointcloudBasic.h"

void CPointcloudBasic::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

double CPointcloudBasic::getDistanceOf2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud1_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud2_arg)
{
	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	double dist_1to2, ave_dist_1to2;
	dist_1to2 = 0.;
	ave_dist_1to2 = 0.;

	pcl::CorrespondencesPtr model_scene_corrs_1to2(new pcl::Correspondences());
	match_search.setInputCloud(p_cloud2_arg);

	for (size_t i = 0; i < p_cloud1_arg->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);

		int found_neighs = match_search.nearestKSearch(p_cloud1_arg->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1)
		{
			//pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			pcl::Correspondence corr(static_cast<int> (i), neigh_indices[0], neigh_sqr_dists[0]);

			model_scene_corrs_1to2->push_back(corr);
		}
	}

	//std::cout << "Correspondences found (1to2): " << model_scene_corrs_1To2->size() << std::endl;
	for (auto itr = model_scene_corrs_1to2->begin(); itr != model_scene_corrs_1to2->end(); itr++) {

		int idx_src, idx_tgt;
		idx_src = itr->index_query;
		idx_tgt = itr->index_match;

		double dist_;
		dist_ = sqrt(itr->distance);
		dist_1to2 += dist_;

	}

	//p_cloud1_arg
	//p_cloud2_arg

	if (model_scene_corrs_1to2->size() == 0) {
		cout << "error: no point" << endl;
		int aa;
		cin >> aa;
		return -1.;
	}

	{
		vector<float> distance_vec;
		float median_corr;
		int size = model_scene_corrs_1to2->size();
		for (int i = 0; i < size; i++)
		{
			distance_vec.push_back(sqrt(model_scene_corrs_1to2->at(i).distance));
		}
		sort(distance_vec.begin(), distance_vec.end());

		if (size % 2 == 1) {
			median_corr = distance_vec[(size - 1) / 2];
		}
		else {
			median_corr = (distance_vec[(size / 2) - 1] + distance_vec[size / 2]) / 2.;
		}
		cout << "median_corr(map) = " << median_corr << endl;

	}


	ave_dist_1to2 = dist_1to2 / model_scene_corrs_1to2->size();

	return ave_dist_1to2;

}

float CPointcloudBasic::getCorrMedianDistance(pcl::Correspondences correspondences)
{
	vector<float> distance_vec;
	int size = correspondences.size();
	for (int i = 0; i < size; i++)
	{
		distance_vec.push_back(sqrt(correspondences.at(i).distance));
	}
	sort(distance_vec.begin(), distance_vec.end());

	if (size % 2 == 1)
		return distance_vec[(size - 1) / 2];
	else
		return (distance_vec[(size / 2) - 1] + distance_vec[size / 2]) / 2.;

}

void CPointcloudBasic::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &input_arg,
	pcl::PointCloud<pcl::PointXYZRGB> &output_arg, Eigen::Matrix4f transformation_arg)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_copy(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(input_arg, *input_copy);
	output_arg.clear();
	Eigen::Affine3f trans_ = Eigen::Affine3f::Identity();
	trans_ = calcAffine3fFromHomogeneousMatrix(transformation_arg.cast<double>());
	pcl::transformPointCloud(*input_copy, output_arg, trans_);

}

Eigen::Matrix4d CPointcloudBasic::calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
	double Roll_, double Pitch_, double Yaw_)
{
	Eigen::Matrix4d	transformation_Position = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Roll_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Pitch_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Yaw_mat = Eigen::Matrix4d::Identity();
	T_mat(0, 3) = X_;
	T_mat(1, 3) = Y_;
	T_mat(2, 3) = Z_;
	Roll_mat(1, 1) = cos(Roll_);
	Roll_mat(1, 2) = -sin(Roll_);
	Roll_mat(2, 1) = sin(Roll_);
	Roll_mat(2, 2) = cos(Roll_);
	Pitch_mat(0, 0) = cos(Pitch_);
	Pitch_mat(2, 0) = -sin(Pitch_);
	Pitch_mat(0, 2) = sin(Pitch_);
	Pitch_mat(2, 2) = cos(Pitch_);
	Yaw_mat(0, 0) = cos(Yaw_);
	Yaw_mat(0, 1) = -sin(Yaw_);
	Yaw_mat(1, 0) = sin(Yaw_);
	Yaw_mat(1, 1) = cos(Yaw_);
	transformation_Position = T_mat * Yaw_mat * Pitch_mat * Roll_mat;
	return transformation_Position;
}

Eigen::Matrix4d CPointcloudBasic::calcHomogeneousMatrixFromVector6d(Eigen::Vector6d XYZRPY_arg)
{
	Eigen::Matrix4d	transformation_Position = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d T_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Roll_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Pitch_mat = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d Yaw_mat = Eigen::Matrix4d::Identity();
	T_mat(0, 3) = XYZRPY_arg(0, 0);
	T_mat(1, 3) = XYZRPY_arg(1, 0);
	T_mat(2, 3) = XYZRPY_arg(2, 0);
	Roll_mat(1, 1) = cos(XYZRPY_arg(3, 0));
	Roll_mat(1, 2) = -sin(XYZRPY_arg(3, 0));
	Roll_mat(2, 1) = sin(XYZRPY_arg(3, 0));
	Roll_mat(2, 2) = cos(XYZRPY_arg(3, 0));
	Pitch_mat(0, 0) = cos(XYZRPY_arg(4, 0));
	Pitch_mat(2, 0) = -sin(XYZRPY_arg(4, 0));
	Pitch_mat(0, 2) = sin(XYZRPY_arg(4, 0));
	Pitch_mat(2, 2) = cos(XYZRPY_arg(4, 0));
	Yaw_mat(0, 0) = cos(XYZRPY_arg(5, 0));
	Yaw_mat(0, 1) = -sin(XYZRPY_arg(5, 0));
	Yaw_mat(1, 0) = sin(XYZRPY_arg(5, 0));
	Yaw_mat(1, 1) = cos(XYZRPY_arg(5, 0));
	transformation_Position = T_mat * Yaw_mat * Pitch_mat * Roll_mat;
	return transformation_Position;
}

//Not very confident
Eigen::Vector6d CPointcloudBasic::calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat)
{
	Eigen::Vector6d XYZRPY = Eigen::Vector6d::Zero();
	double X_, Y_, Z_, Roll_, Pitch_, Yaw_;
	X_ = input_Mat(0, 3);
	Y_ = input_Mat(1, 3);
	Z_ = input_Mat(2, 3);
	if (input_Mat(2, 0) == -1.) {
		Pitch_ = M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(input_Mat(1, 2), input_Mat(1, 1));
	}
	else if (input_Mat(2, 0) == 1.) {
		Pitch_ = -M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(-input_Mat(1, 2), input_Mat(1, 1));
	}
	else {
		Yaw_ = atan2(input_Mat(1, 0), input_Mat(0, 0));
		Roll_ = atan2(input_Mat(2, 1), input_Mat(2, 2));
		double cos_Pitch;
		if (cos(Yaw_) == 0.) {
			cos_Pitch = input_Mat(0, 0) / sin(Yaw_);
		}
		else 	cos_Pitch = input_Mat(0, 0) / cos(Yaw_);

		Pitch_ = atan2(-input_Mat(2, 0), cos_Pitch);
	}
	if (!(-M_PI < Roll_)) Roll_ += M_PI;
	else if (!(Roll_ < M_PI)) Roll_ -= M_PI;
	if (!(-M_PI < Pitch_)) Pitch_ += M_PI;
	else if (!(Pitch_ < M_PI)) Pitch_ -= M_PI;
	if (!(-M_PI < Yaw_)) Yaw_ += M_PI;
	else if (!(Yaw_ < M_PI)) Yaw_ -= M_PI;

	XYZRPY << X_, Y_, Z_,
		Roll_, Pitch_, Yaw_;
	//cout << "Roll_ = " << Roll_ << endl;
	//cout << "Pitch_ = " << Pitch_ << endl;
	//cout << "Yaw_ = " << Yaw_ << endl;

	return XYZRPY;
}

Eigen::Affine3f CPointcloudBasic::calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat)
{
	Eigen::Affine3f Trans_Affine = Eigen::Affine3f::Identity();
	Eigen::Vector6d Trans_Vec = Eigen::Vector6d::Identity();
	Trans_Vec = calcVector6dFromHomogeneousMatrix(input_Mat);
	Trans_Affine.translation() << Trans_Vec(0, 0), Trans_Vec(1, 0), Trans_Vec(2, 0);
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(5, 0), Eigen::Vector3f::UnitZ()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(4, 0), Eigen::Vector3f::UnitY()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(3, 0), Eigen::Vector3f::UnitX()));
	return Trans_Affine;
}

Eigen::Vector6d CPointcloudBasic::calcRobotPosition_3DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
	Eigen::Vector6d pose_sensor, Eigen::Vector6d disp_registration)
{
	//T(i-1)T(odo,i)T(s)T(icp,i)=T(i)T(s)
	//<-> T(i)=T(i-1)T(odo,i)T(s)T(icp,i)T(s)^(-1)

	//robot pos after odometry
	Eigen::Vector6d pos_odometry = Eigen::Vector6d::Zero();
	pos_odometry(0, 0) = cos(pos_before(5, 0))*disp_odometry(0, 0) - sin(pos_before(5, 0))*disp_odometry(1, 0)
		+ pos_before(0, 0);
	pos_odometry(1, 0) = sin(pos_before(5, 0))*disp_odometry(0, 0) + cos(pos_before(5, 0))*disp_odometry(1, 0)
		+ pos_before(1, 0);
	pos_odometry(5, 0) = pos_before(5, 0) + disp_odometry(5, 0);

	//sensor pos after odometry
	Eigen::Vector6d pos_sensor_odometry = Eigen::Vector6d::Zero();
	pos_sensor_odometry(0, 0) = cos(pos_odometry(5, 0))*pose_sensor(0, 0) + pos_odometry(0, 0);
	pos_sensor_odometry(1, 0) = sin(pos_odometry(5, 0))*pose_sensor(0, 0) + pos_odometry(1, 0);
	pos_sensor_odometry(5, 0) = pos_odometry(5, 0);
	pos_sensor_odometry(2, 0) = pose_sensor(2, 0) + pos_odometry(2, 0);

	//sensor pos after registration
	Eigen::Vector6d pos_sensor_reg = Eigen::Vector6d::Zero();
	pos_sensor_reg(0, 0) = cos(disp_registration(5, 0))*pos_sensor_odometry(0, 0) - sin(disp_registration(5, 0))*pos_sensor_odometry(1, 0)
		+ disp_registration(0, 0);
	pos_sensor_reg(1, 0) = sin(disp_registration(5, 0))*pos_sensor_odometry(0, 0) + cos(disp_registration(5, 0))*pos_sensor_odometry(1, 0)
		+ disp_registration(1, 0);
	pos_sensor_reg(5, 0) = pos_sensor_odometry(5, 0) + disp_registration(5, 0);
	pos_sensor_reg(2, 0) = pos_sensor_odometry(2, 0);

	//robot pos after registration
	Eigen::Vector6d pos_reg = Eigen::Vector6d::Zero();
	pos_reg(5, 0) = pos_sensor_reg(5, 0);
	pos_reg(0, 0) = pos_sensor_reg(0, 0) - cos(pos_reg(5, 0))*pose_sensor(0, 0);
	pos_reg(1, 0) = pos_sensor_reg(1, 0) - sin(pos_reg(5, 0))*pose_sensor(0, 0);
	pos_reg(2, 0) = pos_sensor_reg(2, 0) - pose_sensor(2, 0);
	return pos_reg;
}

Eigen::Vector6d CPointcloudBasic::calcRobotPosition_6DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
	Eigen::Vector6d pose_sensor, Eigen::Vector6d disp_registration)
{
	//T(i-1)T(odo,i)T(s)T(icp,i)=T(i)T(s)
	//<-> T(i)=T(i-1)T(odo,i)T(s)T(icp,i)T(s)^(-1)
	Eigen::Vector6d pos_reg = Eigen::Vector6d::Zero();
	pos_reg = calcVector6dFromHomogeneousMatrix(
		calcHomogeneousMatrixFromVector6d(pos_before)
		* calcHomogeneousMatrixFromVector6d(disp_odometry)
		* calcHomogeneousMatrixFromVector6d(pose_sensor)
		* calcHomogeneousMatrixFromVector6d(disp_registration)
		* calcHomogeneousMatrixFromVector6d(pose_sensor).inverse());
	return pos_reg;
}

pcl::Correspondences CPointcloudBasic::getCorrespondences_eachPairHaving(const pcl::Correspondences &corr_src_tgt,
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
