//https://qiita.com/n_chiba_/items/fc9605cde5c19a8c7dad
#include<iostream>
#include <string>
#include <tuple>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "TimeString.h"

typedef pcl::PointXYZ T_PointType;
//typedef pcl::PointXYZRGB T_PointType;

// 法線推定と特徴量計算
auto preprocess_point_cloud(const pcl::PointCloud<T_PointType>::Ptr& pointcloud, const float voxel_size)
{
	//Keypoint を Voxel Down Sample で生成
	const pcl::PointCloud<T_PointType>::Ptr keypoints(new pcl::PointCloud<T_PointType>);
	const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>); // なぜかPtrがprotectedなので
	sor->setInputCloud(pointcloud);
	sor->setLeafSize(voxel_size, voxel_size, voxel_size);
	sor->filter(*keypoints);
	cout << "pointcloud->size():" << pointcloud->size() << endl;
	cout << "keypoints->size():" << keypoints->size() << endl;

	// 法線推定
	const float radius_normal = voxel_size * 2.0;
	const auto view_point = T_PointType(0.0, 10.0, 10.0);

	const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
	const pcl::search::KdTree<T_PointType>::Ptr kdtree(new pcl::search::KdTree<T_PointType>);
	//ne->setInputCloud(pointcloud);
	ne->setInputCloud(keypoints);
	ne->setRadiusSearch(radius_normal);
	ne->setSearchMethod(kdtree);
	ne->setViewPoint(view_point.x, view_point.y, view_point.z);
	ne->compute(*normals);

	// FPFH特徴量計算
	const float radius_feature = voxel_size * 5.0;

	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	const pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>);
	fpfhe->setInputCloud(keypoints);
	//fpfhe->setSearchSurface(pointcloud); // Open3Dだとこれができない（？）
	fpfhe->setSearchSurface(keypoints); // Open3Dだとこれができない（？）
	fpfhe->setInputNormals(normals);
	fpfhe->setSearchMethod(kdtree);
	fpfhe->setRadiusSearch(radius_feature);
	fpfhe->compute(*fpfh);

	cout << "fpfh->size:" << fpfh->size() << endl;
	vector<vector<double>> hist_vecvec;
	for (int j = 0; j < fpfh->size(); j++)
	{
		//if (j % 1000 != 0) continue;
		//cout << "j:" << j << " fpfh->points[j].descriptorSize:" << (int)fpfh->points[j].descriptorSize << endl;
		//cout << "j:" << j << " fpfh->points[j].descriptorSize:" << fpfh->points[j].descriptorSize << endl;

		int size_array = sizeof(fpfh->points[j].histogram);
		vector<double> hist_vec;
		for (int i = 0; i < size_array; i++)
		{
			hist_vec.push_back(fpfh->points[j].histogram[i]);
		}
		hist_vecvec.push_back(hist_vec);
	}
	cout << "hist_vecvec[0].size()" << hist_vecvec[0].size() << endl;
	string filename ="../../data/" + CTimeString::getTimeString() + "hist.csv";
	//CTimeString::getCSVFromVecVec(hist_vecvec, filename);

	return std::make_pair(keypoints, fpfh);
}

auto draw_registration_result(
	const std::string& window_name,
	const pcl::PointCloud<T_PointType>::Ptr& scene1,
	const pcl::PointCloud<T_PointType>::Ptr& scene2,
	const Eigen::Matrix4f& trans)
{
	const pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
	viewer->setBackgroundColor(255.0, 255.0, 255.0);

	const auto scene1_temp = scene1->makeShared();
	const auto scene2_temp = scene2->makeShared();

	pcl::transformPointCloud(*scene1_temp, *scene1_temp, trans);

	const pcl::visualization::PointCloudColorHandlerCustom<T_PointType>::Ptr color1(new pcl::visualization::PointCloudColorHandlerCustom<T_PointType>(scene1_temp, 1.0 * 255, 0.706 * 255, 0.0 * 255));
	const pcl::visualization::PointCloudColorHandlerCustom<T_PointType>::Ptr color2(new pcl::visualization::PointCloudColorHandlerCustom<T_PointType>(scene2_temp, 0.0 * 255, 0.651 * 255, 0.929 * 255));
	viewer->addPointCloud<T_PointType>(scene1_temp, *color1, "scene1");
	viewer->addPointCloud<T_PointType>(scene2_temp, *color2, "scene2");

	return viewer;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
	pcl::PointCloud<T_PointType>::Ptr cloud_, float radius_normal, float radius_FPFH)
{
	//const float radius_normal = voxel_size * 2.0;
	const auto view_point = T_PointType(0.0, 10.0, 10.0);

	const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
	const pcl::search::KdTree<T_PointType>::Ptr kdtree(new pcl::search::KdTree<T_PointType>);
	ne->setInputCloud(cloud_);
	ne->setRadiusSearch(radius_normal);
	ne->setSearchMethod(kdtree);
	ne->setViewPoint(view_point.x, view_point.y, view_point.z);
	ne->compute(*normals);

	//const float radius_FPFH = voxel_size * 5.0;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	const pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>);
	fpfhe->setInputCloud(cloud_);
	fpfhe->setSearchSurface(cloud_);
	fpfhe->setInputNormals(normals);
	fpfhe->setSearchMethod(kdtree);
	fpfhe->setRadiusSearch(radius_FPFH);
	fpfhe->compute(*fpfh);

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

Eigen::Matrix4f align_FPFH_SAC_AI(pcl::PointCloud<T_PointType>::Ptr cloud_src, pcl::PointCloud<T_PointType>::Ptr cloud_tgt)
{
	cout << "preprocess" << endl;

	pcl::PointCloud<T_PointType>::Ptr src_(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr tgt_(new pcl::PointCloud<T_PointType>());
	pcl::copyPointCloud(*cloud_src, *src_);
	pcl::copyPointCloud(*cloud_tgt, *tgt_);

	float voxel_size;
	//voxel_size = 0.01;
	//voxel_size = 0.05;
	voxel_size = 0.1;

	float radius_normal_FPFH, radius_FPFH;
	radius_normal_FPFH = voxel_size * 2.0;
	radius_FPFH = voxel_size * 5.0;

	float distance_th_evaluation_SAC, SimilarityThreshold_SAC, InlierFraction_SAC;
	distance_th_evaluation_SAC = voxel_size * 2.5;
	int MaximumIterations_SAC, NumberOfSamples_SAC, CorrespondenceRandomness_SAC;
	//MaximumIterations_SAC = 500000;
	//MaximumIterations_SAC = 50;	//8 & 8
	MaximumIterations_SAC = 200;
	NumberOfSamples_SAC = 4;//8 & 8
	//NumberOfSamples_SAC = 10;
	CorrespondenceRandomness_SAC = 2;
	SimilarityThreshold_SAC = 0.9f;
	InlierFraction_SAC = 0.25f;


	const pcl::PointCloud<T_PointType>::Ptr keypoints(new pcl::PointCloud<T_PointType>);
	const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>); // なぜかPtrがprotectedなので
	sor->setLeafSize(voxel_size, voxel_size, voxel_size);
	sor->setInputCloud(src_);
	sor->filter(*src_);
	cout << "cloud_src->size():" << cloud_src->size() << endl;
	cout << "src_->size():" << src_->size() << endl;
	sor->setInputCloud(tgt_);
	sor->filter(*tgt_);
	cout << "cloud_tgt->size():" << cloud_tgt->size() << endl;
	cout << "tgt_->size():" << tgt_->size() << endl;

	//compute fpfh
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_src(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
	fpfh_src = computeFPFH(src_, radius_normal_FPFH, radius_FPFH);
	fpfh_tgt = computeFPFH(tgt_, radius_normal_FPFH, radius_FPFH);

	const pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33>::Ptr align(new pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33>);
	const pcl::PointCloud<T_PointType>::Ptr temp_(new pcl::PointCloud<T_PointType>);
	cout << "RANSAC" << endl;
	align->setInputSource(src_);
	align->setSourceFeatures(fpfh_src);
	align->setInputTarget(tgt_);
	align->setTargetFeatures(fpfh_tgt);
	align->setMaximumIterations(MaximumIterations_SAC);
	align->setNumberOfSamples(NumberOfSamples_SAC);
	align->setCorrespondenceRandomness(CorrespondenceRandomness_SAC);
	align->setSimilarityThreshold(SimilarityThreshold_SAC);				//th of corr rejecter
	align->setMaxCorrespondenceDistance(distance_th_evaluation_SAC);	//related to th of computing fitness score
	align->setInlierFraction(InlierFraction_SAC);						//th of inlier number
	//align->setMinSampleDistance(min_sample_distance_);	//function not found
	align->align(*temp_);
	cout << "align->getFitnessScore():" << align->getFitnessScore() << endl;
	cout << "align->hasConverged():" << align->hasConverged() << endl;
	return align->getFinalTransformation();
}

int main()
{
	pcl::PointCloud<T_PointType>::Ptr scene1(new pcl::PointCloud<T_PointType>());
	pcl::PointCloud<T_PointType>::Ptr scene2(new pcl::PointCloud<T_PointType>());
	pcl::io::loadPCDFile("../../data/008XYZRGB_naraha.pcd", *scene1);
	pcl::io::loadPCDFile("../../data/008XYZRGB_naraha.pcd", *scene2);
	//pcl::io::loadPCDFile("../../data/009XYZRGB_naraha.pcd", *scene2);

	// scene2 を適当に回転・並進
	Eigen::Matrix4f transform_matrix;
	transform_matrix <<
		1.0, 0.0, 0.0, -0.1,
		0.0, 0.0, -1.0, 0.1,
		0.0, 1.0, 0.0, -0.1,
		0.0, 0.0, 0.0, 1;
	pcl::transformPointCloud(*scene2, *scene2, transform_matrix);

	const auto viewer1 = draw_registration_result("Initial", scene1, scene2, Eigen::Matrix4f::Identity());

	auto transform = align_FPFH_SAC_AI(scene1, scene2);

	const auto viewer2 = draw_registration_result("Global", scene1, scene2, transform);

	while (!viewer1->wasStopped() && !viewer2->wasStopped())
	{
		viewer1->spinOnce();
		viewer2->spinOnce();
	}
}