//https://qiita.com/n_chiba_/items/fc9605cde5c19a8c7dad#icp-%E3%81%AB%E3%82%88%E3%82%8Bregistration

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

// 法線推定と特徴量計算
auto preprocess_point_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud, const float voxel_size)
{
	//Keypoint を Voxel Down Sample で生成
	const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	const boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> sor(new pcl::VoxelGrid<pcl::PointXYZ>); // なぜかPtrがprotectedなので
	sor->setInputCloud(pointcloud);
	sor->setLeafSize(voxel_size, voxel_size, voxel_size);
	sor->filter(*keypoints);

	// 法線推定
	const float radius_normal = voxel_size * 2.0;
	const auto view_point = pcl::PointXYZ(0.0, 10.0, 10.0);

	const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	const pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>);
	const pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne->setInputCloud(pointcloud);
	ne->setRadiusSearch(radius_normal);
	ne->setSearchMethod(kdtree);
	ne->setViewPoint(view_point.x, view_point.y, view_point.z);
	ne->compute(*normals);

	// FPFH特徴量計算
	const float radius_feature = voxel_size * 5.0;

	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
	const pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
	fpfhe->setInputCloud(keypoints);
	fpfhe->setSearchSurface(pointcloud); // Open3Dだとこれができない（？）
	fpfhe->setInputNormals(normals);
	fpfhe->setSearchMethod(kdtree);
	fpfhe->setRadiusSearch(radius_feature);
	fpfhe->compute(*fpfh);

	return std::make_pair(keypoints, fpfh);
}

auto execute_global_registration(
	const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr>& scene1,
	const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr>& scene2,
	const float voxel_size)
{
	const auto& kp1 = scene1.first;
	const auto& kp2 = scene2.first;
	const auto& fpfh1 = scene1.second;
	const auto& fpfh2 = scene2.second;

	const float distance_threshold = voxel_size * 2.5;

	const pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	const pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>::Ptr align(new pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>);
	align->setInputSource(kp1);
	align->setSourceFeatures(fpfh1);
	align->setInputTarget(kp2);
	align->setTargetFeatures(fpfh2);
	align->setMaximumIterations(500000);
	align->setNumberOfSamples(4);
	align->setCorrespondenceRandomness(2);
	align->setSimilarityThreshold(0.9f);
	align->setMaxCorrespondenceDistance(distance_threshold);
	align->setInlierFraction(0.25f);
	align->align(*output);

	return align->getFinalTransformation();
}

auto refine_registration(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene1,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene2,
	const Eigen::Matrix4f& trans,
	const float voxel_size)
{
	const auto scene1_temp = scene1->makeShared();
	pcl::transformPointCloud(*scene1_temp, *scene1_temp, trans);

	const float distance_threshold = voxel_size * 0.4;

	const pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	const pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
	icp->setInputSource(scene1_temp);
	icp->setInputTarget(scene2);
	icp->setMaxCorrespondenceDistance(distance_threshold);
	icp->align(*output);

	return icp->getFinalTransformation();
}

auto read_point_cloud(const std::string& filename)
{
	const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
	//const int retval = pcl::io::loadPLYFile(filename, *pointcloud);
	const int retval = pcl::io::loadPCDFile(filename, *pointcloud);
	if (retval == -1 || pointcloud->size() <= 0)
	{
		PCL_ERROR("File load error.");
		exit(-1);
	}

	return pointcloud;
}

auto draw_registration_result(
	const std::string& window_name,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene1,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene2,
	const Eigen::Matrix4f& trans)
{
	const pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
	viewer->setBackgroundColor(255.0, 255.0, 255.0);

	const auto scene1_temp = scene1->makeShared();
	const auto scene2_temp = scene2->makeShared();

	pcl::transformPointCloud(*scene1_temp, *scene1_temp, trans);

	const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color1(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene1_temp, 1.0 * 255, 0.706 * 255, 0.0 * 255));
	const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color2(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene2_temp, 0.0 * 255, 0.651 * 255, 0.929 * 255));
	viewer->addPointCloud<pcl::PointXYZ>(scene1_temp, *color1, "scene1");
	viewer->addPointCloud<pcl::PointXYZ>(scene2_temp, *color2, "scene2");

	return viewer;
}

int main()
{
	// 読み込み
	//const auto scene1 = read_point_cloud("scene1.ply");
	//const auto scene2 = read_point_cloud("scene2.ply");
	const auto scene1 = read_point_cloud("\../../data/_Naraha_output_and_heavy/rawPointCloud/008.pcd");
	const auto scene2 = read_point_cloud("\../../data/_Naraha_output_and_heavy/rawPointCloud/009.pcd");

	// scene2 を適当に回転・並進
	Eigen::Matrix4f transform_matrix;
	//transform_matrix <<
	//	1.0, 0.0, 0.0, -0.1,
	//	0.0, 0.0, -1.0, 0.1,
	//	0.0, 1.0, 0.0, -0.1,
	//	0.0, 0.0, 0.0, 1;
	//pcl::transformPointCloud(*scene2, *scene2, transform_matrix);

	// 位置合わせ前の点群の表示
	const auto viewer1 = draw_registration_result("Initial", scene1, scene2, Eigen::Matrix4f::Identity());

	const float voxel_size = 0.01;

	cout << "preprocess" << endl;
	//RANSAC による Global Registration
	const auto scene1_kp_fpfh = preprocess_point_cloud(scene1, voxel_size);
	const auto scene_kp_fpfh = preprocess_point_cloud(scene2, voxel_size);
	cout << "RANSAC" << endl;
	const auto result_ransac = execute_global_registration(scene1_kp_fpfh, scene_kp_fpfh, voxel_size);
	const auto viewer2 = draw_registration_result("Global", scene1, scene2, result_ransac);

	// ICP による refine
	const auto result = refine_registration(scene1, scene2, result_ransac, voxel_size);
	const auto viewer3 = draw_registration_result("Refined", scene1, scene2, result * result_ransac);

	while (!viewer1->wasStopped() && !viewer2->wasStopped() && !viewer3->wasStopped())
	{
		viewer1->spinOnce();
		viewer2->spinOnce();
		viewer3->spinOnce();
	}
}