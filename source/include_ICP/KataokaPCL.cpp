#include "KataokaPCL.h"

CKataokaPCL::CKataokaPCL()
	:
M_convergence_criteria_()
//, M_correspondences_(new pcl::Correspondences())
, M_correspondences_original(new Correspondences_Kataoka())
, M_correspondences_spring1(new Correspondences_Spring1())
, M_correspondences_spring2(new Correspondences_Spring2())
//, M_correspondences_spring1()
//	CorrespondencesPtr_Spring_1 M_correspondences_spring1;

{
	M_transformation_ = Eigen::Matrix4f::Identity();
	M_previous_transformation_ = Eigen::Matrix4f::Identity();
	M_final_transformation_ = Eigen::Matrix4f::Identity();
	M_transformation_arg = Eigen::Matrix4f::Identity();

	M_max_iterations_ = 50000;
	M_euclidean_fitness_epsilon_ = 1.;
	M_transformation_epsilon_ = 1e-6;
	M_corr_dist_threshold_ = 0.05;

	//convergence_criteria_.reset(new pcl::registration::
	//	DefaultConvergenceCriteria<double>(nr_iterations_, transformation_, correspondences_));

	M_pPointCloud_source = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_target = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
	M_pPointCloud_src_transformed = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

	M_converged_ = false;

	M_min_number_correspondences_ = 3;	//same as PCL

	//M_convergence_criteria_original.reset(new CKataokaConvergence_(M_nr_iterations_, M_transformation_, *M_correspondences_));
	M_convergence_criteria_original.reset(new CKataokaConvergence_(M_nr_iterations_, M_transformation_, *M_correspondences_original));
	M_convergence_criteria_Spring1.reset(new CKataokaConvergence_(M_nr_iterations_, M_transformation_, *M_correspondences_spring1));
	M_convergence_criteria_Spring2.reset(new CKataokaConvergence_(M_nr_iterations_, M_transformation_, *M_correspondences_spring2));

	M_chara_tgt_vec.push_back(1.);
	M_chara_src_vec.push_back(1.);

}

CKataokaPCL::~CKataokaPCL()
{

}

void CKataokaPCL::setCharaVector_src(vector<int> chara_vec)
{
	M_chara_src_vec = chara_vec;
}

void CKataokaPCL::setCharaVector_tgt(vector<int> chara_vec)
{
	M_chara_tgt_vec = chara_vec;
}

Eigen::Matrix4d CKataokaPCL::calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
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

Eigen::Matrix4d CKataokaPCL::calcHomogeneousMatrixFromVector6d(Eigen::Vector6d XYZRPY_arg)
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
Eigen::Vector6d CKataokaPCL::calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat)
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

Eigen::Affine3f CKataokaPCL::calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat)
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

void CKataokaPCL::align()
{

	M_converged_ = false;

	if (M_pPointCloud_source->size() == 0 || M_pPointCloud_target->size() == 0) 
	{
		cout << "error: pPointCloud_src or pPointCloud_tgt has no point" << endl;
		M_final_transformation_ = Eigen::Matrix4f::Identity();
		M_final_transformation_Vec = Eigen::Vector6d::Zero();
		return;
	}

	if (M_i_method == 1 && (M_chara_src_vec.size() == 0 || M_chara_tgt_vec.size() == 0))
	{
		cout << "error: chara_src_vec or chara_tgt_vec has no value" << endl;
		M_final_transformation_ = Eigen::Matrix4f::Identity();
		M_final_transformation_Vec = Eigen::Vector6d::Zero();
		return;
	}

	if (M_i_method == 2 && (M_spring_src_vecvec.size() == 0 || M_spring_tgt_vecvec.size() == 0))
	{
		cout << "error: M_spring1_src_vecvec or M_spring1_tgt_vecvec has no value" << endl;
		M_final_transformation_ = Eigen::Matrix4f::Identity();
		M_final_transformation_Vec = Eigen::Vector6d::Zero();
		return;
	}

	M_final_transformation_ = M_transformation_arg;

	computeTransformation();
}

void CKataokaPCL::align(Eigen::Matrix4d init)
{

	//メンバ変数に渡す．
	M_transformation_arg = init.cast<float>();

	align();

	M_transformation_arg = Eigen::Matrix4f::Identity();

}

void CKataokaPCL::computeTransformation()
{
	static bool b_convergence_original = false;
	b_convergence_original = true;

	double FittnessScore = 0.;

	//cout << "computeTransformation" << endl;

	M_pPointCloud_src_transformed->clear();
	pcl::copyPointCloud(*M_pPointCloud_source, *M_pPointCloud_src_transformed);

	M_transformation_ = Eigen::Matrix4f::Identity();


	if (M_final_transformation_ != Eigen::Matrix4f::Identity()) {
		transformPointCloud(*M_pPointCloud_src_transformed, *M_pPointCloud_src_transformed, M_final_transformation_);
	}

	pcl::registration::TransformationEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::
		Ptr transformation_estimation_(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB, pcl::PointXYZRGB>);


	//insert to M_i_method
	//convergence
	cout << "M_i_method = " << M_i_method << endl;
	switch (M_i_method) {
	case 0:
		M_convergence_criteria_original->setMaximumIterations(M_max_iterations_);
		M_convergence_criteria_original->setRelativeMSE(M_euclidean_fitness_epsilon_);
		M_convergence_criteria_original->setTranslationThreshold(M_transformation_epsilon_);
		M_convergence_criteria_original->setRotationThreshold(1.0 - M_transformation_epsilon_);
		M_convergence_criteria_original->setMaximumIterationsSimilarTransforms(10);
		M_convergence_criteria_original->setAbsoluteMSE(0.0001);
		M_convergence_criteria_original->setMethodInt(M_i_method);
		break;
	case 1:
		M_convergence_criteria_original->setMaximumIterations(M_max_iterations_);
		M_convergence_criteria_original->setRelativeMSE(M_euclidean_fitness_epsilon_);
		M_convergence_criteria_original->setTranslationThreshold(M_transformation_epsilon_);
		M_convergence_criteria_original->setRotationThreshold(1.0 - M_transformation_epsilon_);
		M_convergence_criteria_original->setMaximumIterationsSimilarTransforms(10);
		M_convergence_criteria_original->setAbsoluteMSE(0.0001);
		M_convergence_criteria_original->setMethodInt(M_i_method);
		break;
	case 2:
		M_convergence_criteria_Spring1->setMaximumIterations(M_max_iterations_);
		M_convergence_criteria_Spring1->setRelativeMSE(M_euclidean_fitness_epsilon_);
		M_convergence_criteria_Spring1->setTranslationThreshold(M_transformation_epsilon_);
		M_convergence_criteria_Spring1->setRotationThreshold(1.0 - M_transformation_epsilon_);
		M_convergence_criteria_Spring1->setMaximumIterationsSimilarTransforms(10);
		M_convergence_criteria_Spring1->setAbsoluteMSE(0.0001);
		M_convergence_criteria_Spring1->setMethodInt(M_i_method);
		break;
	case 3:
		M_convergence_criteria_Spring2->setMaximumIterations(M_max_iterations_);
		M_convergence_criteria_Spring2->setRelativeMSE(M_euclidean_fitness_epsilon_);
		M_convergence_criteria_Spring2->setTranslationThreshold(M_transformation_epsilon_);
		M_convergence_criteria_Spring2->setRotationThreshold(1.0 - M_transformation_epsilon_);
		M_convergence_criteria_Spring2->setMaximumIterationsSimilarTransforms(10);
		M_convergence_criteria_Spring2->setAbsoluteMSE(0.0001);
		M_convergence_criteria_Spring2->setMethodInt(M_i_method);
		break;
	default:
		cout << "error: no method" << endl;
		break;
	}


	M_nr_iterations_ = 0;

	M_converged_ = false;

	size_t cnt;

	do
	{
		cout << "nr_iterations_ : " << M_nr_iterations_ << endl;
		switch (M_i_method)
		{
		case 0:
			determineCorrespondences(*M_correspondences_original, M_corr_dist_threshold_);
			cnt = M_correspondences_original->size();
			break;
		case 1:
			determineCorrespondences_chara(*M_correspondences_original,
				M_proposed_penalty_chara, M_proposed_dist_search, M_proposed_weight_dist_chara);
			cnt = M_correspondences_original->size();
			break;
		case 2:
			determineCorrespondences_Spring1(*M_correspondences_spring1, M_corr_dist_threshold_, M_WeightConstant_Spring1_vec);
			cnt = M_correspondences_spring1->size();
			break;
		case 3:
			determineCorrespondences_Spring2(*M_correspondences_spring2, M_corr_dist_threshold_, M_WeightConstant_Spring1_vec);
			cnt = M_correspondences_spring2->size();

			//need to set
			break;
		default:
			break;

		}

		//if (M_i_method == 3)
		//{
		//	for (int i = 0; i < M_correspondences_spring2->size(); i++)
		//	{
		//		for (int j = 0; j < M_correspondences_spring2->at(i).index_match_vec.size(); j++)
		//		{
		//			if (i % 200 != 0) continue;
		//			if (j % 20 != 0) continue;
		//			cout << "i:" << i;
		//			cout << " j:" << j << endl;
		//			cout << "weight_spring = " << M_correspondences_spring2->at(i).weight_spring_vec[j] << endl;
		//		}
		//	}
		//	{
		//		int aa;
		//		cin >> aa;
		//	}
		//}

		//Check whether we have enough correspondences
		if (static_cast<int> (cnt) < M_min_number_correspondences_)
		{
			cout << "error: Not enough correspondences found. Relax your threshold parameters." << endl;
			M_final_transformation_ = Eigen::Matrix4f::Identity();

			break;
		}


		//Estimate the transform
		//transformation_estimation_->estimateRigidTransformation(*M_pPointCloud_src_transformed, *M_pPointCloud_target, *M_correspondences_, M_transformation_);
		//estimateRigidTransformation(*M_pPointCloud_src_transformed, *M_pPointCloud_target, *M_correspondences_, M_transformation_);
		//estimateRigidTransformation(*M_pPointCloud_src_transformed, *M_pPointCloud_target, *M_correspondences_original, M_transformation_);

		//transformation
		switch (M_i_method) {
		case 0:
			estimateRigidTransformation(*M_pPointCloud_src_transformed, *M_pPointCloud_target, *M_correspondences_original, M_transformation_);
			break;
		case 1:
			estimateRigidTransformation(*M_pPointCloud_src_transformed, *M_pPointCloud_target, *M_correspondences_original, M_transformation_);
			break;
		case 2:
			estimateRigidTransformation(*M_pPointCloud_src_transformed, *M_pPointCloud_target, *M_correspondences_spring1, M_transformation_);
			break;
		case 3:
			estimateRigidTransformation(*M_pPointCloud_src_transformed, *M_pPointCloud_target, *M_correspondences_spring2, M_transformation_);
			break;
		default:
			break;
		}

		transformPointCloud(*M_pPointCloud_src_transformed, *M_pPointCloud_src_transformed, M_transformation_);

		//// Obtain the final transformation    
		M_final_transformation_ = M_transformation_ * M_final_transformation_;

		++M_nr_iterations_;

		double FittnessScore_Chara;
		switch (M_i_method) {
		case 0:
			M_converged_ = M_convergence_criteria_original->hasConverged();
			FittnessScore = M_convergence_criteria_original->getFittnessScore();
			cout << "FittnessScore = " << FittnessScore << endl;
			break;

		case 1:
			M_converged_ = M_convergence_criteria_original->hasConverged();
			FittnessScore = M_convergence_criteria_original->getFittnessScore();
			cout << "FittnessScore = " << FittnessScore << endl;
			FittnessScore_Chara = getFitnessScore_chara();
			cout << "FittnessScore_Chara = " << FittnessScore_Chara << endl;
			break;

		case 2:
			M_converged_ = M_convergence_criteria_Spring1->hasConverged();
			FittnessScore = M_convergence_criteria_Spring1->getFittnessScore();
			cout << "FittnessScore = " << FittnessScore << endl;
			break;
		case 3:
			M_converged_ = M_convergence_criteria_Spring2->hasConverged();
			FittnessScore = M_convergence_criteria_Spring2->getFittnessScore();
			cout << "FittnessScore = " << FittnessScore << endl;
			break;

		default:
			break;
		}

		//median
		float median_corr = 0.;
		switch (M_i_method) {
		case 0:
			median_corr = getCorrMedianDistance(M_correspondences_original);
			break;
		case 1:
			median_corr = getCorrMedianDistance(M_correspondences_original);
			break;
		case 2:
			median_corr = getCorrMedianDistance(M_correspondences_spring1);
			break;
		case 3:
			median_corr = getCorrMedianDistance(M_correspondences_spring2);
			break;
		default:
			break;
		}
		cout << "median = " << median_corr << endl;



	} while (!M_converged_);

	// Repeat until convergence

	M_final_transformation_Vec = Eigen::Vector6d::Zero();
	if (M_converged_)
	{
		cout << "converged!!" << endl;
		M_final_transformation_Vec(0, 0) = (double)M_final_transformation_(0, 3);
		M_final_transformation_Vec(1, 0) = (double)M_final_transformation_(1, 3);
		M_final_transformation_Vec(2, 0) = (double)M_final_transformation_(2, 3);
		M_final_transformation_Vec(4, 0) = (double)(asin(-M_final_transformation_(2, 0)));
		M_final_transformation_Vec(3, 0) = (double)(asin(M_final_transformation_(2, 1)) / cos(M_final_transformation_Vec(4, 0)));
		M_final_transformation_Vec(5, 0) = (double)(asin(M_final_transformation_(1, 0)) / cos(M_final_transformation_Vec(4, 0)));
	}
	else cout << "not converged!!" << endl;


	cout << "X_ICP = " << M_final_transformation_Vec(0, 0) << endl;
	cout << "Y_ICP = " << M_final_transformation_Vec(1, 0) << endl;
	cout << "Z_ICP = " << M_final_transformation_Vec(2, 0) << endl;
	cout << "Roll_ICP = " << M_final_transformation_Vec(3, 0) * R2D << "[deg]" << endl;
	cout << "Pitch_ICP = " << M_final_transformation_Vec(4, 0) * R2D << "[deg]" << endl;
	cout << "Yaw_ICP = " << M_final_transformation_Vec(5, 0) * R2D << "[deg]" << endl;

}

void CKataokaPCL::determineCorrespondences(pcl::Correspondences &correspondences, double max_distance)
{

	//cout << "determineCorrespondences" << endl;

	////why sqr ?
	//double max_dist_sqr;
	//max_dist_sqr = max_distance * max_distance;

	//cout<<"correspondences"

	correspondences.resize(M_pPointCloud_src_transformed->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;

	std::vector<int> index(1);
	std::vector<float> distance(1);
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	match_search.setInputCloud(M_pPointCloud_target);

	for (size_t i = 0; i < M_pPointCloud_src_transformed->size(); ++i)
	{

		//if ((int)(i) % 5000 == 0)	cout << "i = " << i << endl;

		int found_neighs = match_search.nearestKSearch(M_pPointCloud_src_transformed->at(i), 1, index, distance);
		//int found_neighs = match_search.nearestKSearch(M_pPointCloud_source->points[i], 1, index, distance);

		if (distance[0] > max_distance * max_distance)
			continue;
		pcl::Correspondence corr;
		corr.index_query = i;
		corr.index_match = index[0];
		corr.distance = distance[0];
		correspondences[nr_valid_correspondences++] = corr;
	}

	correspondences.resize(nr_valid_correspondences);

	//cout << "correspondences size = " << correspondences.size() << endl;
	//cout << "correspondences size = " << (int)(correspondences.size()) << endl;
	cout << "correspondences size = " << nr_valid_correspondences << endl;
}

void CKataokaPCL::determineCorrespondences(Correspondences_Kataoka &correspondences, double max_distance)
{

	//cout << "determineCorrespondences" << endl;

	//why sqr ?
	//double max_dist_sqr;
	//max_dist_sqr = max_distance * max_distance;

	//cout<<"correspondences"

	correspondences.resize(M_pPointCloud_src_transformed->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;

	std::vector<int> index(1);
	std::vector<float> distance(1);
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	match_search.setInputCloud(M_pPointCloud_target);

	for (size_t i = 0; i < M_pPointCloud_src_transformed->size(); ++i)
	{

		//if ((int)(i) % 5000 == 0)	cout << "i = " << i << endl;

		int found_neighs = match_search.nearestKSearch(M_pPointCloud_src_transformed->at(i), 1, index, distance);
		//int found_neighs = match_search.nearestKSearch(M_pPointCloud_source->points[i], 1, index, distance);

		if (distance[0] > max_distance * max_distance)
			continue;

		Correspondence_Kataoka corr;
		//pcl::Correspondence corr;
		corr.index_query = i;
		corr.index_match = index[0];
		corr.distance = distance[0];
		correspondences[nr_valid_correspondences++] = corr;
	}

	correspondences.resize(nr_valid_correspondences);

	//cout << "correspondences size = " << correspondences.size() << endl;
	//cout << "correspondences size = " << (int)(correspondences.size()) << endl;
	cout << "correspondences size = " << nr_valid_correspondences << endl;
}


void CKataokaPCL::determineCorrespondences_chara(pcl::Correspondences &correspondences,
	double penalty_chara, double dist_search_arg, double weight_dist_chara)
{

	//cout << "determineCorrespondences" << endl;

	correspondences.resize(M_pPointCloud_src_transformed->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;

	std::vector<int> index_vec;
	std::vector<float> distance_vec;
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	////size check
	//cout << "size check" << endl;
	//cout << "M_pPointCloud_transformed size = " << M_pPointCloud_src_transformed->size() << endl;
	//cout << "M_pPointCloud_target size = " << M_pPointCloud_target->size() << endl;
	//cout << "M_chara_src_vec size = " << M_chara_src_vec.size() << endl;
	//cout << "M_chara_tgt_vec size = " << M_chara_tgt_vec.size() << endl;


	match_search.setInputCloud(M_pPointCloud_target);

	//cout << "calc correspondences" << endl;
	for (size_t i = 0; i < M_pPointCloud_src_transformed->size(); ++i)
	{

		//if(static_cast<int>(i) % 5000 == 0)	cout << "calc correspondences  i : " << i << endl;

		//int found_neighs = match_search.nearestKSearch(M_pPointCloud_transformed->at(i), 1, index, distance);
		//int found_neighs = match_search.nearestKSearch(M_pPointCloud_source->points[i], 1, index, distance);

		double dist_search = dist_search_arg;	//should be not squared distance

		if (!(M_chara_src_vec[i] == 0)) dist_search *= weight_dist_chara;

		int found_neighs = match_search.radiusSearch(M_pPointCloud_src_transformed->at(i), dist_search, index_vec, distance_vec);


		int index_min = 0;
		double value_min = 100.;
		double distance_tgt = 10000.;

		if (found_neighs == 0) continue;

		for (int j = 0; j < index_vec.size(); j++) {

			double error_chara = 0;

			if (M_chara_src_vec[i] == M_chara_tgt_vec[index_vec[j]])
				error_chara = sqrt(distance_vec[j]);
			else
				error_chara = sqrt(distance_vec[j]) + penalty_chara;


			if (error_chara < value_min) {
				value_min = error_chara;
				index_min = index_vec[j];
				distance_tgt = distance_vec[j];
			}
		}

		pcl::Correspondence corr;
		corr.index_query = i;
		corr.index_match = index_min;
		corr.distance = distance_tgt;
		correspondences[nr_valid_correspondences++] = corr;

		//if (static_cast<int>(i) % 5000 == 0) {

		//	cout << "corr.index_query = " << corr.index_query << endl;
		//	cout << "corr.index_match = " << corr.index_match << endl;
		//	cout << "corr.distance =" << corr.distance << endl;

		//}	

	}

	correspondences.resize(nr_valid_correspondences);

	cout << "correspondences size = " << nr_valid_correspondences << endl;
}

void CKataokaPCL::determineCorrespondences_chara(Correspondences_Kataoka &correspondences,
	double penalty_chara, double dist_search_arg, double weight_dist_chara)
{

	//cout << "determineCorrespondences" << endl;

	correspondences.resize(M_pPointCloud_src_transformed->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;

	std::vector<int> index_vec;
	std::vector<float> distance_vec;
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	////size check
	//cout << "size check" << endl;
	//cout << "M_pPointCloud_transformed size = " << M_pPointCloud_src_transformed->size() << endl;
	//cout << "M_pPointCloud_target size = " << M_pPointCloud_target->size() << endl;
	//cout << "M_chara_src_vec size = " << M_chara_src_vec.size() << endl;
	//cout << "M_chara_tgt_vec size = " << M_chara_tgt_vec.size() << endl;


	match_search.setInputCloud(M_pPointCloud_target);

	//cout << "calc correspondences" << endl;
	for (size_t i = 0; i < M_pPointCloud_src_transformed->size(); ++i)
	{

		//if(static_cast<int>(i) % 5000 == 0)	cout << "calc correspondences  i : " << i << endl;

		//int found_neighs = match_search.nearestKSearch(M_pPointCloud_transformed->at(i), 1, index, distance);
		//int found_neighs = match_search.nearestKSearch(M_pPointCloud_source->points[i], 1, index, distance);

		double dist_search = dist_search_arg;	//should be not squared distance

		if (!(M_chara_src_vec[i] == 0)) dist_search *= weight_dist_chara;

		int found_neighs = match_search.radiusSearch(M_pPointCloud_src_transformed->at(i), dist_search, index_vec, distance_vec);


		int index_min = 0;
		double value_min = 100.;
		double distance_tgt = 10000.;

		if (found_neighs == 0) continue;

		for (int j = 0; j < index_vec.size(); j++) {

			double error_chara = 0;

			if (M_chara_src_vec[i] == M_chara_tgt_vec[index_vec[j]])
				error_chara = sqrt(distance_vec[j]);
			else
				error_chara = sqrt(distance_vec[j]) + penalty_chara;


			if (error_chara < value_min) {
				value_min = error_chara;
				index_min = index_vec[j];
				distance_tgt = distance_vec[j];
			}
		}

		Correspondence_Kataoka corr;
		//pcl::Correspondence corr;
		corr.index_query = i;
		corr.index_match = index_min;
		corr.distance = distance_tgt;
		correspondences[nr_valid_correspondences++] = corr;

		//if (static_cast<int>(i) % 5000 == 0) {

		//	cout << "corr.index_query = " << corr.index_query << endl;
		//	cout << "corr.index_match = " << corr.index_match << endl;
		//	cout << "corr.distance =" << corr.distance << endl;

		//}	

	}

	correspondences.resize(nr_valid_correspondences);

	cout << "correspondences size = " << nr_valid_correspondences << endl;
}


void CKataokaPCL::determineCorrespondences_argPC(pcl::Correspondences &correspondences, double max_distance,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_tgt)
{

	//cout << "determineCorrespondences" << endl;

	double max_dist_sqr;
	max_dist_sqr = max_distance * max_distance;

	//cout<<"correspondences"

	correspondences.resize(p_cloud_src->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;

	std::vector<int> index(1);
	std::vector<float> distance(1);
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	match_search.setInputCloud(p_cloud_tgt);

	for (size_t i = 0; i < p_cloud_src->size(); ++i)
	{

		int found_neighs = match_search.nearestKSearch(p_cloud_src->at(i), 1, index, distance);

		if (distance[0] > max_dist_sqr)		//squared
			continue;
		pcl::Correspondence corr;
		corr.index_query = i;
		corr.index_match = index[0];
		corr.distance = distance[0];
		correspondences[nr_valid_correspondences++] = corr;
	}

	correspondences.resize(nr_valid_correspondences);

	//cout << "correspondences size = " << correspondences.size() << endl;
	//cout << "correspondences size = " << (int)(correspondences.size()) << endl;
	cout << "correspondences size = " << nr_valid_correspondences << endl;
}

void CKataokaPCL::determineCorrespondences_argPC_chara(pcl::Correspondences &correspondences, double max_distance,
	double penalty_chara, double dist_search_arg, double weight_dist_chara,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_src_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud_tgt_arg,
	vector<int> Chara_start_vec, vector<int> Chara_end_vec)
{

	//cout << "determineCorrespondences" << endl;

	//why sqr ?
	double max_dist_sqr;
	max_dist_sqr = max_distance * max_distance;

	//cout<<"correspondences"

	correspondences.resize(pPointCloud_src_arg->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;

	std::vector<int> index_vec;
	std::vector<float> distance_vec;
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	//cout << "debug: pPointCloud_src_arg size = " << pPointCloud_src_arg->size() << endl;
	//cout << "debug: pPointCloud_tgt_arg size = " << pPointCloud_tgt_arg->size() << endl;
	//cout << "debug: Chara_start_vec size = " << Chara_start_vec.size() << endl;
	//cout << "debug: Chara_end_vec size = " << Chara_end_vec.size() << endl;

	match_search.setInputCloud(pPointCloud_tgt_arg);

	for (size_t i = 0; i < pPointCloud_src_arg->size(); ++i)
	{

		//if(static_cast<int>(i)%100 == 0) cout << "debug: A" << endl;

		double dist_search = dist_search_arg;	//should be not squared distance
		if (!(Chara_end_vec[static_cast<int>(i)] == 0)) dist_search *= weight_dist_chara;

		int found_neighs = match_search.radiusSearch(pPointCloud_src_arg->at(i), dist_search, index_vec, distance_vec);
		//if (static_cast<int>(i) % 100 == 0) cout << "debug: B" << endl;

		int index_min = 0;
		double value_min = 100.;
		double distance_tgt = 10000.;

		if (found_neighs == 0) continue;

		for (int j = 0; j < index_vec.size(); j++) {

			double error_chara = 0;

			if (Chara_end_vec[static_cast<int>(i)] == Chara_start_vec[index_vec[j]])
				error_chara = sqrt(distance_vec[j]);
			else
				error_chara = sqrt(distance_vec[j]) + penalty_chara;

			if (error_chara < value_min) {
				value_min = error_chara;
				index_min = index_vec[j];
				distance_tgt = distance_vec[j];
			}
		}
		//if (static_cast<int>(i) % 100 == 0) cout << "debug: C" << endl;

		pcl::Correspondence corr;
		corr.index_query = i;
		corr.index_match = index_min;
		corr.distance = distance_tgt;
		correspondences[nr_valid_correspondences++] = corr;
	}

	correspondences.resize(nr_valid_correspondences);

	cout << "correspondences size = " << nr_valid_correspondences << endl;

}
//	CorrespondencesPtr_Spring_1 M_correspondences_spring1;

void CKataokaPCL::determineCorrespondences_Spring1(Correspondences_Spring1 &correspondences,
	double max_distance, vector<double> WeightConstant_spring_vec)
{
	correspondences.resize(M_pPointCloud_src_transformed->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;
	//cout << "M_spring1_src_vecvec.size() = " << M_spring1_src_vecvec.size() << endl;
	//cout << "M_spring1_tgt_vecvec.size() = " << M_spring1_tgt_vecvec.size() << endl;

	std::vector<int> index(1);
	std::vector<float> distance(1);
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	match_search.setInputCloud(M_pPointCloud_target);

	for (size_t i = 0; i < M_pPointCloud_src_transformed->size(); ++i)
	{
		//cout << "i:" << i << endl;
		//cout << endl;
		//if ((int)(i) % 5000 == 0)	cout << "i = " << i << endl;

		int found_neighs = match_search.nearestKSearch(M_pPointCloud_src_transformed->at(i), 1, index, distance);

		if (distance[0] > max_distance * max_distance)
			continue;

		Correspondence_Spring1 corr;
		corr.index_query = static_cast<int>(i);
		corr.index_match = index[0];
		corr.distance = distance[0];
		double chara_error = 0.;

		//for (int j = 0; j < WeightConstant_spring_vec.size(); j++) 
		for (int dim_chara = 0; dim_chara < M_spring_tgt_vecvec[corr.index_match].size(); dim_chara++)
		{

			chara_error += pow(WeightConstant_spring_vec[dim_chara]
				* (M_spring_tgt_vecvec[corr.index_match][dim_chara] - M_spring_src_vecvec[corr.index_query][dim_chara]), 2.);
			//chara_error += pow(1. * (M_spring1_tgt_vecvec[corr.index_match][j] - M_spring1_src_vecvec[corr.index_query][j]), 2.);
		}
		double weight_spring1 = 0.;
		weight_spring1 = exp(-chara_error);
		corr.weight_spring = weight_spring1;
		correspondences[nr_valid_correspondences++] = corr;
	}

	correspondences.resize(nr_valid_correspondences);
	cout << "correspondences size = " << nr_valid_correspondences << endl;
}

void CKataokaPCL::determineCorrespondences_Spring2(Correspondences_Spring2 &correspondences,
	double max_distance, vector<double> WeightConstant_spring_vec)
{
	correspondences.resize(M_pPointCloud_src_transformed->size());

	//cout << "M_pPointCloud_source->size() = " << M_pPointCloud_source->size() << endl;
	//cout << "M_pPointCloud_target->size() = " << M_pPointCloud_target->size() << endl;
	//cout << "M_spring1_src_vecvec.size() = " << M_spring1_src_vecvec.size() << endl;
	//cout << "M_spring1_tgt_vecvec.size() = " << M_spring1_tgt_vecvec.size() << endl;

	std::vector<int> index(1);
	std::vector<float> distance(1);
	unsigned int nr_valid_correspondences = 0;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	match_search.setInputCloud(M_pPointCloud_target);

	for (size_t i = 0; i < M_pPointCloud_src_transformed->size(); ++i)
	{
		//cout << "i:" << i << endl;
		//cout << endl;
		//if ((int)(i) % 5000 == 0)	cout << "i = " << i << endl;

		int found_neighs_nearestK = match_search.nearestKSearch(M_pPointCloud_src_transformed->at(i), 1, index, distance);

		if (distance[0] > max_distance * max_distance)
			continue;

		index.clear();
		distance.clear();

		int found_neighs_radius = match_search.radiusSearch(M_pPointCloud_src_transformed->at(i), sqrt(max_distance), index, distance);

		Correspondence_Spring2 corr;
		//corr.index_query
		//corr.index_match_vec
		//corr.distance_vec
		//corr.weight_spring_vec

		corr.index_query = static_cast<int>(i);
		corr.index_match_vec = index;
		corr.distance_vec = distance;

		//if (i % 100 == 0)
		//{
		//	cout << "i:" << i;
		//	cout << " corr:" << corr.index_match_vec.size() << endl;
		//}

		vector<float> weight_spring_vec;
		weight_spring_vec.clear();
		for (int j = 0; j < index.size(); j++)
		{
			double chara_error = 0.;
			//for (int dim_chara = 0; dim_chara < WeightConstant_spring_vec.size(); dim_chara++)
			for (int dim_chara = 0; dim_chara < M_spring_tgt_vecvec[corr.index_match_vec[j]].size(); dim_chara++)
				{

				chara_error += pow(WeightConstant_spring_vec[dim_chara]
					* (M_spring_tgt_vecvec[corr.index_match_vec[j]][dim_chara] - M_spring_src_vecvec[corr.index_query][dim_chara]), 2.);
				//chara_error += pow(1. * (M_spring1_tgt_vecvec[corr.index_match][j] - M_spring1_src_vecvec[corr.index_query][j]), 2.);
			}
			float weight_spring = (float)exp(-chara_error);
			weight_spring_vec.push_back(weight_spring);

			//if (i % 20 != 0) continue;
			//if (j % 20 != 0) continue;
			//cout << "chara_error = " << chara_error << endl;
		}
		corr.weight_spring_vec = weight_spring_vec;

		correspondences[nr_valid_correspondences++] = corr;
	}

	correspondences.resize(nr_valid_correspondences);
	cout << "correspondences size = " << nr_valid_correspondences << endl;
}

void CKataokaPCL::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &input_arg,
	pcl::PointCloud<pcl::PointXYZRGB> &output_arg, Eigen::Matrix4f transformation_arg)
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_copy(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(input_arg, *input_copy);
	output_arg.clear();
	Eigen::Affine3f trans_ = Eigen::Affine3f::Identity();
	trans_ = calcAffine3fFromHomogeneousMatrix(transformation_arg.cast<double>());
	pcl::transformPointCloud(*input_copy, output_arg, trans_);

}

double CKataokaPCL::getFitnessScore()
{

	double fitness_score = 0.0;

	// Transform the input dataset using the final transformation
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
	
	transformPointCloud(*M_pPointCloud_source, *input_transformed, M_final_transformation_);


	//std::vector<int> nn_indices(1);
	//std::vector<float> nn_dists(1);
	// For each point in the source dataset
	int nr = 0;

	std::vector<int> index(1);
	std::vector<float> distance(1);
	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	match_search.setInputCloud(M_pPointCloud_target);

	for (size_t i = 0; i < input_transformed->size(); ++i)
	{

		int found_neighs = match_search.nearestKSearch(input_transformed->at(i), 1, index, distance);

		// Add to the fitness score
		fitness_score += sqrt(distance[0]);
		nr++;
	}

	if (nr > 0)	return (fitness_score / (double)(nr));
	else		return (-1.);

}

double CKataokaPCL::getFitnessScore_chara() 
{

	double fitness_score_chara = 0.0;

	// Transform the input dataset using the final transformation
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());

	transformPointCloud(*M_pPointCloud_source, *input_transformed, M_final_transformation_);


	//std::vector<int> nn_indices(1);
	//std::vector<float> nn_dists(1);
	// For each point in the source dataset
	int nr = 0;

	std::vector<int> index(1);
	std::vector<float> distance(1);
	pcl::KdTreeFLANN<pcl::PointXYZRGB> match_search;

	match_search.setInputCloud(M_pPointCloud_target);

	for (size_t i = 0; i < input_transformed->size(); ++i)
	{

		int found_neighs = match_search.nearestKSearch(input_transformed->at(i), 1, index, distance);

		if (distance[0] > M_proposed_dist_search * M_proposed_dist_search) continue;

		// Add to the fitness score
		if (M_chara_src_vec[i] == M_chara_tgt_vec[index[0]])
			fitness_score_chara += sqrt(distance[0]);
		else fitness_score_chara += sqrt(distance[0]) + M_proposed_penalty_chara;
		nr++;
	}

	if (nr > 0)	return (fitness_score_chara / (double)(nr));
	else		return (-1.);

}


void CKataokaPCL::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

double CKataokaPCL::getDistanceOf2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud1_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud2_arg)
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

int CKataokaPCL::do_exp_getCharaOfPoint_Todai(double x_, double y_, double z_)
{

	int chara_value = 0;

	double x1, x2, y1, y2, z2;
	z2 = 1.;

	//0821
	{
		double width_heat = 0.25 + 1.;
		double width_pubble = 1. + 1.;
		double length = 1.;

		int value_pubble = 1;
		int value_heat = 2;

		//up
		x1 = 23.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 27.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 32.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}


		x1 = 36.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 37.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 42.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 44.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_pubble;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_pubble;
		}

		x1 = 46.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 47.;
		y2 = 14.;
		x2 = x1 + length;
		y1 = y2 - width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		//down
		x1 = 21.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 22.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 24.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_pubble;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_pubble;
		}

		x1 = 30.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_pubble;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_pubble;
		}

		x1 = 31.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_pubble;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_pubble;
		}

		x1 = 33.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 38.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_pubble;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_pubble;
		}

		x1 = 40.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 41.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_pubble;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_pubble;
		}

		x1 = 44.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_heat;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_heat;
		}

		x1 = 48.;
		y1 = 10.;
		x2 = x1 + length;
		y2 = y1 + width_pubble;
		if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z_ <= z2) {
			chara_value = value_pubble;
		}

	}

	return chara_value;
}

bool CKataokaPCL::do_exp_getIsRemovedPoint_Todai(double x_, double y_, double z_)
{

	bool b_remove = false;

	//cubic
	{
		double x1, y1, x2, y2, z1, z2;
		z1 = 0.;
		z2 = 100.;

		{
			x1 = 22.;
			y1 = 13.;
			x2 = x1 + 1.;
			y2 = y1 + 1.;

			y1 += 0.15;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;

		}

		{
			x1 = 24.;
			y1 = 13.;
			x2 = x1 + 1.;
			y2 = y1 + 1.;

			y1 += 0.15;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;

		}

		{
			x1 = 24.;
			y1 = 11.;
			x2 = x1 + 5.;
			y2 = y1 + 1.;

			z1 = 0.;
			z2 = 1.;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;
			z1 = 0.;
			z2 = 100.;

		}

		{
			x1 = 29.;
			y1 = 13.;
			x2 = x1 + 2.;
			y2 = y1 + 1.;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;

		}

		{
			x1 = 34.;
			y1 = 10.;
			x2 = x1 + 3.;
			y2 = y1 + 1.;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;

		}


		{
			x1 = 37.;
			y1 = 13.;
			x2 = x1 + 2.;
			y2 = y1 + 2.;

			y1 += 0.1;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;

		}

		{
			x1 = 40.;
			y1 = 13.;
			x2 = x1 + 2.;
			y2 = y1 + 1.;

			y1 += 0.1;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;

		}

		{
			x1 = 40.;
			y1 = 10.;
			x2 = x1 + 1.;
			y2 = y1 + 1.;

			if (x1 <= x_ && x_ <= x2 && y1 <= y_ && y_ <= y2 && z1 <= z_ && z_ <= z2) b_remove = true;

		}

	}

	////radius
	//{
	//	double radius_delete;
	//	radius_delete = 1.;

	//	{
	//		double x_center;
	//		double y_center;
	//		double distance_;
	//		x_center = 30.;
	//		y_center = 13.;
	//		distance_ = sqrt(pow(x_ - x_center, 2.) + pow(y_ - y_center, 2.));
	//		if (distance_ <= radius_delete) b_remove = true;
	//	}

	//	{
	//		double x_center;
	//		double y_center;
	//		double distance_;
	//		x_center = 38.;
	//		y_center = 14.;
	//		distance_ = sqrt(pow(x_ - x_center, 2.) + pow(y_ - y_center, 2.));
	//		if (distance_ <= radius_delete) b_remove = true;
	//	}

	//	{
	//		double x_center;
	//		double y_center;
	//		double distance_;
	//		x_center = 41.;
	//		y_center = 13.;
	//		distance_ = sqrt(pow(x_ - x_center, 2.) + pow(y_ - y_center, 2.));
	//		if (distance_ <= radius_delete) b_remove = true;
	//	}

	//	{
	//		double x_center;
	//		double y_center;
	//		double distance_;
	//		x_center = 28.;
	//		y_center = 11.;
	//		distance_ = sqrt(pow(x_ - x_center, 2.) + pow(y_ - y_center, 2.));
	//		if (distance_ <= radius_delete) b_remove = true;
	//	}

	//	{
	//		double x_center;
	//		double y_center;
	//		double distance_;
	//		x_center = 35.;
	//		y_center = 11.;
	//		distance_ = sqrt(pow(x_ - x_center, 2.) + pow(y_ - y_center, 2.));
	//		if (distance_ <= radius_delete) b_remove = true;
	//	}

	//	{
	//		double x_center;
	//		double y_center;
	//		double distance_;
	//		x_center = 46.;
	//		y_center = 11.;
	//		distance_ = sqrt(pow(x_ - x_center, 2.) + pow(y_ - y_center, 2.));
	//		if (distance_ <= radius_delete) b_remove = true;
	//	}

	//}
	return b_remove;
}

//	vector<vector<double>> M_chara_src_vecvec;

void CKataokaPCL::setSpring1VecVec_src(vector<vector<double>> spring1_src_vecvec)
{
	M_spring_src_vecvec = spring1_src_vecvec;
}

void CKataokaPCL::setSpring1VecVec_tgt(vector<vector<double>> spring1_tgt_vecvec)
{
	M_spring_tgt_vecvec = spring1_tgt_vecvec;
}

template <typename PointSource, typename PointTarget> void
CKataokaPCL::estimateRigidTransformation(
	const pcl::PointCloud<PointSource> &cloud_src_arg,
	const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
	const pcl::Correspondences &correspondences,
	Eigen::Matrix4f &transformation_matrix) const
{
	//ここらへんを自分で書く?
	//ConstCloudIterator<PointSource> source_it(cloud_src_arg, correspondences, true);
	//ConstCloudIterator<PointTarget> target_it(cloud_tgt_arg, correspondences, false);

	pcl::PointCloud<PointSource>::Ptr cloud_src_corr(new pcl::PointCloud<PointSource>());
	pcl::PointCloud<PointTarget>::Ptr cloud_tgt_corr(new pcl::PointCloud<PointTarget>());

	for (int i = 0; i < correspondences.size(); i++) {

		//イテレータの方がよさそう．
		cloud_src_corr->push_back(cloud_src_arg.points[correspondences[i].index_query]);
		cloud_tgt_corr->push_back(cloud_tgt_arg.points[correspondences[i].index_match]);

	}


	const int npts = static_cast <int> (correspondences.size());
	//cout << "npts = " << npts << endl;
	typedef float Scalar;
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts);
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, npts);
	{
		auto itr_src = cloud_src_corr->begin();
		auto itr_tgt = cloud_tgt_corr->begin();

		for (int i = 0; i < npts; ++i)
		{
			if (itr_src == cloud_src_corr->end()) break;
			if (itr_tgt == cloud_tgt_corr->end()) break;

			//cout << "i:" << i << endl;
			//cloud_src(0, i) = source_it->x;
			//cloud_src(1, i) = source_it->y;
			//cloud_src(2, i) = source_it->z;
			//++source_it;
			//cloud_tgt(0, i) = target_it->x;
			//cloud_tgt(1, i) = target_it->y;
			//cloud_tgt(2, i) = target_it->z;
			//++target_it;
			cloud_src(0, i) = itr_src->x;
			cloud_src(1, i) = itr_src->y;
			cloud_src(2, i) = itr_src->z;
			++itr_src;
			cloud_tgt(0, i) = itr_tgt->x;
			cloud_tgt(1, i) = itr_tgt->y;
			cloud_tgt(2, i) = itr_tgt->z;
			//cout << "cloud_tgt(0, i) = " << cloud_tgt(0, i) << endl;
			++itr_tgt;
		}

	}

	// Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
	{
		//typedef typename internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type TransformationMatrixType;
		typedef typename Eigen::Matrix4f TransformationMatrixType ;
		//typedef typename internal::traits<TransformationMatrixType>::Scalar Scalar;
		typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;

		//EIGEN_STATIC_ASSERT(!NumTraits<Scalar>::IsComplex, NUMERIC_TYPE_MUST_BE_REAL)
		//	EIGEN_STATIC_ASSERT((internal::is_same<Scalar, typename internal::traits<OtherDerived>::Scalar>::value),
		//		YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

			//enum { Dimension = EIGEN_SIZE_MIN_PREFER_DYNAMIC(Derived::RowsAtCompileTime, OtherDerived::RowsAtCompileTime) };
		enum { Dimension = 3 };

		typedef Eigen::Matrix<Scalar, Dimension, 1> VectorType;
		typedef Eigen::Matrix<Scalar, Dimension, Dimension> MatrixType;
		//typedef typename internal::plain_matrix_type_row_major<Derived>::type RowMajorMatrixType;

		typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;

		const Index m = cloud_src.rows(); // dimension
		const Index n = cloud_src.cols(); // number of measurements

		// required for demeaning ...
		const RealScalar one_over_n = RealScalar(1) / static_cast<RealScalar>(n);

		// computation of mean
		//列方向に足し算している
		const VectorType src_mean = cloud_src.rowwise().sum() * one_over_n;
		const VectorType tgt_mean = cloud_tgt.rowwise().sum() * one_over_n;

		// demeaning of src and dst points
		//列方向に共通の列ベクトルを引き算している．
		//const RowMajorMatrixType src_demean = src.colwise() - src_mean;
		//const RowMajorMatrixType dst_demean = dst.colwise() - dst_mean;
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> src_demean(3, npts);
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> tgt_demean(3, npts);
		src_demean = cloud_src.colwise() - src_mean;
		tgt_demean = cloud_tgt.colwise() - tgt_mean;

		// Eq. (36)-(37)
		const Scalar src_var = src_demean.rowwise().squaredNorm().sum() * one_over_n;

		// Eq. (38)
		const MatrixType sigma = one_over_n * tgt_demean * src_demean.transpose();

		Eigen::JacobiSVD<MatrixType> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);

		// Initialize the resulting transformation with an identity matrix...
		Eigen::Matrix4f Rt = TransformationMatrixType::Identity(m + 1, m + 1);

		// Eq. (39)
		VectorType S = VectorType::Ones(m);

		if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
			S(m - 1) = -1;

		// Eq. (40) and (43)	
		//回転成分のみに代入
		Rt.block(0, 0, m, m).noalias() = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

		//並進成分のみに代入
		Rt.col(m).head(m) = tgt_mean;
		//並進成分のみに代入
		//回転成分のみ呼び出し
		Rt.col(m).head(m).noalias() -= Rt.topLeftCorner(m, m)*src_mean;

		transformation_matrix = Rt;
	}

}

template <typename PointSource, typename PointTarget> void
CKataokaPCL::estimateRigidTransformation(
	const pcl::PointCloud<PointSource> &cloud_src_arg,
	const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
	const Correspondences_Kataoka &correspondences,
	Eigen::Matrix4f &transformation_matrix) const
{

	pcl::PointCloud<PointSource>::Ptr cloud_src_corr(new pcl::PointCloud<PointSource>());
	pcl::PointCloud<PointTarget>::Ptr cloud_tgt_corr(new pcl::PointCloud<PointTarget>());

	for (int i = 0; i < correspondences.size(); i++) {

		//イテレータの方がよさそう．
		cloud_src_corr->push_back(cloud_src_arg.points[correspondences[i].index_query]);
		cloud_tgt_corr->push_back(cloud_tgt_arg.points[correspondences[i].index_match]);

	}

	const int npts = static_cast <int> (correspondences.size());
	//cout << "npts = " << npts << endl;
	typedef float Scalar;
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts);
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, npts);
	{
		auto itr_src = cloud_src_corr->begin();
		auto itr_tgt = cloud_tgt_corr->begin();

		for (int i = 0; i < npts; ++i)
		{
			if (itr_src == cloud_src_corr->end()) break;
			if (itr_tgt == cloud_tgt_corr->end()) break;

			//cout << "i:" << i << endl;
			//cloud_src(0, i) = source_it->x;
			//cloud_src(1, i) = source_it->y;
			//cloud_src(2, i) = source_it->z;
			//++source_it;
			//cloud_tgt(0, i) = target_it->x;
			//cloud_tgt(1, i) = target_it->y;
			//cloud_tgt(2, i) = target_it->z;
			//++target_it;
			cloud_src(0, i) = itr_src->x;
			cloud_src(1, i) = itr_src->y;
			cloud_src(2, i) = itr_src->z;
			++itr_src;
			cloud_tgt(0, i) = itr_tgt->x;
			cloud_tgt(1, i) = itr_tgt->y;
			cloud_tgt(2, i) = itr_tgt->z;
			//cout << "cloud_tgt(0, i) = " << cloud_tgt(0, i) << endl;
			++itr_tgt;
		}

	}

	// Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
	{
		//typedef typename internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type TransformationMatrixType;
		typedef typename Eigen::Matrix4f TransformationMatrixType;
		//typedef typename internal::traits<TransformationMatrixType>::Scalar Scalar;
		typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;

		//EIGEN_STATIC_ASSERT(!NumTraits<Scalar>::IsComplex, NUMERIC_TYPE_MUST_BE_REAL)
		//	EIGEN_STATIC_ASSERT((internal::is_same<Scalar, typename internal::traits<OtherDerived>::Scalar>::value),
		//		YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

			//enum { Dimension = EIGEN_SIZE_MIN_PREFER_DYNAMIC(Derived::RowsAtCompileTime, OtherDerived::RowsAtCompileTime) };
		enum { Dimension = 3 };

		typedef Eigen::Matrix<Scalar, Dimension, 1> VectorType;
		typedef Eigen::Matrix<Scalar, Dimension, Dimension> MatrixType;
		//typedef typename internal::plain_matrix_type_row_major<Derived>::type RowMajorMatrixType;

		typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;

		const Index m = cloud_src.rows(); // dimension
		const Index n = cloud_src.cols(); // number of measurements

		// required for demeaning ...
		const RealScalar one_over_n = RealScalar(1) / static_cast<RealScalar>(n);

		// computation of mean
		//列方向に足し算している
		const VectorType src_mean = cloud_src.rowwise().sum() * one_over_n;
		const VectorType tgt_mean = cloud_tgt.rowwise().sum() * one_over_n;

		// demeaning of src and dst points
		//列方向に共通の列ベクトルを引き算している．
		//const RowMajorMatrixType src_demean = src.colwise() - src_mean;
		//const RowMajorMatrixType dst_demean = dst.colwise() - dst_mean;
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> src_demean(3, npts);
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> tgt_demean(3, npts);
		src_demean = cloud_src.colwise() - src_mean;
		tgt_demean = cloud_tgt.colwise() - tgt_mean;

		// Eq. (36)-(37)
		const Scalar src_var = src_demean.rowwise().squaredNorm().sum() * one_over_n;

		// Eq. (38)
		//const MatrixType sigma = one_over_n * tgt_demean * src_demean.transpose();
		const MatrixType sigma_ = one_over_n * src_demean * tgt_demean.transpose();

		//Eigen::JacobiSVD<MatrixType> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::JacobiSVD<MatrixType> svd_(sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);

		// Initialize the resulting transformation with an identity matrix...
		Eigen::Matrix4f Rt = TransformationMatrixType::Identity(m + 1, m + 1);

		// Eq. (39)
		VectorType S = VectorType::Ones(m);

		//if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
		//	S(m - 1) = -1;
		if (svd_.matrixV().determinant() * svd_.matrixU().determinant() < 0)
			S(m - 1) = -1;

		// Eq. (40) and (43)	
		//回転成分のみに代入
		//Rt.block(0, 0, m, m).noalias() = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
		Rt.block(0, 0, m, m).noalias() = svd_.matrixV() * S.asDiagonal() * svd_.matrixU().transpose();

		//並進成分のみに代入
		Rt.col(m).head(m) = tgt_mean;
		//並進成分のみに代入
		//回転成分のみ呼び出し
		Rt.col(m).head(m).noalias() -= Rt.topLeftCorner(m, m)*src_mean;

		transformation_matrix = Rt;
	}

}

template <typename PointSource, typename PointTarget> void
CKataokaPCL::estimateRigidTransformation(
	const pcl::PointCloud<PointSource> &cloud_src_arg,
	const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
	const Correspondences_Spring1 &correspondences,
	Eigen::Matrix4f &transformation_matrix) const 
{
	typedef float Scalar;
	enum { Dimension = 3 };
	const int npts = static_cast <int> (correspondences.size());

	pcl::PointCloud<PointSource>::Ptr cloud_src_corr(new pcl::PointCloud<PointSource>());
	pcl::PointCloud<PointTarget>::Ptr cloud_tgt_corr(new pcl::PointCloud<PointTarget>());
	Eigen::Matrix<Scalar, Eigen::Dynamic,1 > weight_Vector(npts, 1);
	//		typedef Eigen::Matrix<Scalar, Dimension, 1> VectorType;
	//		VectorType S = VectorType::Ones(m);

	for (int i = 0; i < correspondences.size(); i++) {

		//イテレータの方がよさそう．
		cloud_src_corr->push_back(cloud_src_arg.points[correspondences[i].index_query]);
		cloud_tgt_corr->push_back(cloud_tgt_arg.points[correspondences[i].index_match]);
		weight_Vector(i, 0) = correspondences[i].weight_spring;
	}

	//cout << "npts = " << npts << endl;
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts);
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, npts);
	{
		auto itr_src = cloud_src_corr->begin();
		auto itr_tgt = cloud_tgt_corr->begin();

		for (int i = 0; i < npts; ++i)
		{
			if (itr_src == cloud_src_corr->end()) break;
			if (itr_tgt == cloud_tgt_corr->end()) break;

			//cout << "i:" << i << endl;
			//cloud_src(0, i) = source_it->x;
			//cloud_src(1, i) = source_it->y;
			//cloud_src(2, i) = source_it->z;
			//++source_it;
			//cloud_tgt(0, i) = target_it->x;
			//cloud_tgt(1, i) = target_it->y;
			//cloud_tgt(2, i) = target_it->z;
			//++target_it;
			cloud_src(0, i) = itr_src->x;
			cloud_src(1, i) = itr_src->y;
			cloud_src(2, i) = itr_src->z;
			++itr_src;
			cloud_tgt(0, i) = itr_tgt->x;
			cloud_tgt(1, i) = itr_tgt->y;
			cloud_tgt(2, i) = itr_tgt->z;
			//cout << "cloud_tgt(0, i) = " << cloud_tgt(0, i) << endl;
			++itr_tgt;
		}

	}

	// Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
	{
		//typedef typename internal::umeyama_transform_matrix_type<Derived, OtherDerived>::type TransformationMatrixType;
		typedef typename Eigen::Matrix4f TransformationMatrixType;
		//typedef typename internal::traits<TransformationMatrixType>::Scalar Scalar;
		typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;

		//EIGEN_STATIC_ASSERT(!NumTraits<Scalar>::IsComplex, NUMERIC_TYPE_MUST_BE_REAL)
		//	EIGEN_STATIC_ASSERT((internal::is_same<Scalar, typename internal::traits<OtherDerived>::Scalar>::value),
		//		YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)


		typedef Eigen::Matrix<Scalar, Dimension, 1> VectorType;
		typedef Eigen::Matrix<Scalar, Dimension, Dimension> MatrixType;
		//typedef typename internal::plain_matrix_type_row_major<Derived>::type RowMajorMatrixType;

		typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;

		const Index m = cloud_src.rows(); // dimension
		const Index n = cloud_src.cols(); // number of measurements

		// required for demeaning ...
		const RealScalar one_over_n = RealScalar(1) / static_cast<RealScalar>(n);

		// computation of mean
		//列方向に足し算している
		const VectorType src_mean = cloud_src.rowwise().sum() * one_over_n;
		const VectorType tgt_mean = cloud_tgt.rowwise().sum() * one_over_n;

		// demeaning of src and dst points
		//列方向に共通の列ベクトルを引き算している．
		//const RowMajorMatrixType src_demean = src.colwise() - src_mean;
		//const RowMajorMatrixType dst_demean = dst.colwise() - dst_mean;
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> src_demean(3, npts);
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> tgt_demean(3, npts);
		src_demean = cloud_src.colwise() - src_mean;
		tgt_demean = cloud_tgt.colwise() - tgt_mean;

		//// Eq. (36)-(37)
		//const Scalar src_var = src_demean.rowwise().squaredNorm().sum() * one_over_n;

		// Eq. (38)
		//const MatrixType sigma = one_over_n * tgt_demean * src_demean.transpose();
		const MatrixType sigma_ = one_over_n * src_demean * weight_Vector.asDiagonal() * tgt_demean.transpose();

		//Eigen::JacobiSVD<MatrixType> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::JacobiSVD<MatrixType> svd_(sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);

		// Initialize the resulting transformation with an identity matrix...
		Eigen::Matrix4f Rt = TransformationMatrixType::Identity(m + 1, m + 1);

		// Eq. (39)
		VectorType S = VectorType::Ones(m);

		//if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
		//	S(m - 1) = -1;
		if (svd_.matrixV().determinant() * svd_.matrixU().determinant() < 0)
			S(m - 1) = -1;

		// Eq. (40) and (43)	
		//回転成分のみに代入
		//Rt.block(0, 0, m, m).noalias() = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
		Rt.block(0, 0, m, m).noalias() = svd_.matrixV() * S.asDiagonal() * svd_.matrixU().transpose();

		//並進成分のみに代入
		Rt.col(m).head(m) = tgt_mean;
		//並進成分のみに代入
		//回転成分のみ呼び出し
		Rt.col(m).head(m).noalias() -= Rt.topLeftCorner(m, m)*src_mean;

		transformation_matrix = Rt;
	}

}

template <typename PointSource, typename PointTarget> void
CKataokaPCL::estimateRigidTransformation(
	const pcl::PointCloud<PointSource> &cloud_src_arg,
	const pcl::PointCloud<PointTarget> &cloud_tgt_arg,
	const Correspondences_Spring2 &correspondences,
	Eigen::Matrix4f &transformation_matrix) const
{
	typedef float Scalar;
	enum { Dimension = 3 };
	const int npts = static_cast <int> (correspondences.size());

	//set pointcloud
	pcl::PointCloud<PointSource>::Ptr cloud_src_corr(new pcl::PointCloud<PointSource>());
	vector<pcl::PointCloud<PointTarget>::Ptr> cloud_tgt_corr_vec;
	cloud_tgt_corr_vec.clear();

	for (int i = 0; i < correspondences.size(); i++)
	{
		pcl::PointCloud<PointTarget>::Ptr cloud_tgt_corr(new pcl::PointCloud<PointTarget>());
		cloud_tgt_corr->clear();
		cloud_src_corr->push_back(cloud_src_arg.points[correspondences[i].index_query]);
		for (int index_match = 0; index_match < correspondences[i].index_match_vec.size(); index_match++)
		{
			int index_ = correspondences[i].index_match_vec[index_match];
			cloud_tgt_corr->push_back(cloud_tgt_arg.points[index_]);
		}
		cloud_tgt_corr_vec.push_back(cloud_tgt_corr);
	}

	//matrix from pointcloud and chara(spring)
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts);
	vector<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>> cloud_tgt_vec;
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt_forCoG_mat(3, npts);
	vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1 >> weight_Vector_vec;
	{
		auto itr_src = cloud_src_corr->begin();
		for (int i = 0; i < npts; ++i)
		{
			if (itr_src == cloud_src_corr->end()) break;
			cloud_src(0, i) = itr_src->x;
			cloud_src(1, i) = itr_src->y;
			cloud_src(2, i) = itr_src->z;
			++itr_src;
			Eigen::Matrix<Scalar, Eigen::Dynamic, 1 > weight_Vector(correspondences[i].index_match_vec.size(), 1);
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, cloud_tgt_corr_vec[i]->size());
			auto itr_tgt = cloud_tgt_corr_vec[i]->begin();
			for (int j = 0; j < correspondences[i].index_match_vec.size(); j++)
			{
				if (itr_tgt == cloud_tgt_corr_vec[i]->end()) break;
				cloud_tgt(0, j) = itr_tgt->x;
				cloud_tgt(1, j) = itr_tgt->y;
				cloud_tgt(2, j) = itr_tgt->z;
				++itr_tgt;
				weight_Vector(j, 0) = correspondences[i].weight_spring_vec[j];
			}
			weight_Vector_vec.push_back(weight_Vector);
			cloud_tgt_forCoG_mat(0, i) = cloud_tgt(0, 0);
			cloud_tgt_forCoG_mat(1, i) = cloud_tgt(1, 0);
			cloud_tgt_forCoG_mat(2, i) = cloud_tgt(2, 0);
			cloud_tgt_vec.push_back(cloud_tgt);
		}
	}

	// Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
	{
		typedef typename Eigen::Matrix4f TransformationMatrixType;
		typedef typename Eigen::NumTraits<Scalar>::Real RealScalar;
		typedef Eigen::Matrix<Scalar, Dimension, 1> VectorType;
		typedef Eigen::Matrix<Scalar, Dimension, Dimension> MatrixType;
		typedef EIGEN_DEFAULT_DENSE_INDEX_TYPE Index;

		const Index m = cloud_src.rows(); // dimension
		const Index n = cloud_src.cols(); // number of measurements

		// required for demeaning ...
		const RealScalar one_over_n = RealScalar(1) / static_cast<RealScalar>(n);

		// computation of mean
		const VectorType src_mean = cloud_src.rowwise().sum() * one_over_n;
		const VectorType tgt_mean = cloud_tgt_forCoG_mat.rowwise().sum() * one_over_n;

		// demeaning of src and dst points
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> src_demean(3, npts);
		vector<Eigen::Matrix<Scalar, 3, Eigen::Dynamic>> tgt_demean_vec;
		src_demean = cloud_src.colwise() - src_mean;
		for (int i = 0; i < npts; ++i)
		{
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> tgt_demean(3, cloud_tgt_vec[i].cols());
			tgt_demean = cloud_tgt_vec[i].colwise() - tgt_mean;
			tgt_demean_vec.push_back(tgt_demean);
		}

		// Eq. (38)
		MatrixType sigma_ = MatrixType::Zero();
		for (int i = 0; i < npts; ++i)
		{
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> src_mat(3, tgt_demean_vec[i].size());
			for (int j = 0; j < tgt_demean_vec[i].size(); j++)
			{
				src_mat(0, j) = src_demean(0, i);
				src_mat(1, j) = src_demean(1, i);
				src_mat(2, j) = src_demean(2, i);
			}
			MatrixType sigma_1frame = MatrixType::Zero();
			sigma_1frame = one_over_n * src_mat * weight_Vector_vec[i].asDiagonal() * tgt_demean_vec[i].transpose();
			sigma_ += sigma_1frame;
		}

		Eigen::JacobiSVD<MatrixType> svd_(sigma_, Eigen::ComputeFullU | Eigen::ComputeFullV);

		// Initialize the resulting transformation with an identity matrix...
		Eigen::Matrix4f Rt = TransformationMatrixType::Identity(m + 1, m + 1);

		// Eq. (39)
		VectorType S = VectorType::Ones(m);
		if (svd_.matrixV().determinant() * svd_.matrixU().determinant() < 0)
			S(m - 1) = -1;

		// Eq. (40) and (43)	
		Rt.block(0, 0, m, m).noalias() = svd_.matrixV() * S.asDiagonal() * svd_.matrixU().transpose();

		Rt.col(m).head(m) = tgt_mean;
		Rt.col(m).head(m).noalias() -= Rt.topLeftCorner(m, m)*src_mean;

		transformation_matrix = Rt;
	}

}

float CKataokaPCL::getCorrMedianDistance(pcl::Correspondences correspondences)
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

float CKataokaPCL::getCorrMedianDistance(CorrespondencesPtr_Kataoka correspondences)
{
	vector<float> distance_vec;
	int size = correspondences->size();
	for (int i = 0; i < size; i++)
	{
		distance_vec.push_back(sqrt(correspondences->at(i).distance));
	}
	sort(distance_vec.begin(), distance_vec.end());

	if (size % 2 == 1) {
		return distance_vec[(size - 1) / 2];
	}
	else {
		return (distance_vec[(size / 2) - 1] + distance_vec[size / 2]) / 2;
	}

}

float CKataokaPCL::getCorrMedianDistance(CorrespondencesPtr_Spring1 correspondences)
{
	vector<float> distance_vec;
	int size = correspondences->size();
	for (int i = 0; i < size; i++)
	{
		distance_vec.push_back(sqrt(correspondences->at(i).distance));
	}
	sort(distance_vec.begin(), distance_vec.end());

	if (size % 2 == 1) {
		return distance_vec[(size - 1) / 2];
	}
	else {
		return (distance_vec[(size / 2) - 1] + distance_vec[size / 2]) / 2;
	}

}

float CKataokaPCL::getCorrMedianDistance(CorrespondencesPtr_Spring2 correspondences)
{
	vector<float> distance_vec;
	int size = correspondences->size();
	for (int i = 0; i < size; i++)
	{
		distance_vec.push_back(sqrt(correspondences->at(i).distance_vec[0]));
	}
	sort(distance_vec.begin(), distance_vec.end());

	if (size % 2 == 1) {
		return distance_vec[(size - 1) / 2];
	}
	else {
		return (distance_vec[(size / 2) - 1] + distance_vec[size / 2]) / 2;
	}

}

Eigen::Vector6d CKataokaPCL::calcRobotPosition_3DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
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

Eigen::Vector6d CKataokaPCL::calcRobotPosition_6DoF(Eigen::Vector6d pos_before, Eigen::Vector6d disp_odometry,
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

vector<float> CKataokaPCL::getErrorOfFPFHSource_corr(float &median_arg, pcl::Correspondences correspondences,
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt)
{
	vector<float> error_vec;
	error_vec.resize(fpfh_src->size());
	fill(error_vec.begin(), error_vec.end(), 100000.);

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
				cout << "correspondences.size():" << correspondences.size() << endl;
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
	if (num_nan != 0) cout << "num_nan:" << num_nan << endl;

	//show inlier rate
	cout << "inlier rate:" << (float)correspondences.size() / (float)fpfh_src->size() << endl;

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
	cout << "fpfh error median_arg:" << median_arg << endl;

	return error_vec;
}

vector<float> CKataokaPCL::getFPFHVariance(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_)
{
	vector<float> fpfh_hist_mean;
	fpfh_hist_mean.resize(sizeof(fpfh_->points[0].histogram));
	fill(fpfh_hist_mean.begin(), fpfh_hist_mean.end(), 0.);
	int num_nan = 0;
	vector<bool> b_isnan_vec;

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
				cout << "nan occored in j;" << j << " i:" << i << endl;
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
	cout << "num_nan:" << num_nan << endl;

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
