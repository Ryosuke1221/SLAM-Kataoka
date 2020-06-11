//https://qiita.com/n_chiba_/items/fc9605cde5c19a8c7dad#icp-%E3%81%AB%E3%82%88%E3%82%8Bregistration

//#include<iostream>
//#include <string>
//#include <tuple>
//#include <Eigen/Core>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/transforms.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/registration/sample_consensus_prerejective.h>
//#include <pcl/registration/icp.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>
//
//typedef pcl::PointXYZ T_PointType;
////typedef pcl::PointXYZRGB T_PointType;
//
//// 法線推定と特徴量計算
//auto preprocess_point_cloud(const pcl::PointCloud<T_PointType>::Ptr& pointcloud, const float voxel_size)
//{
//	//Keypoint を Voxel Down Sample で生成
//	const pcl::PointCloud<T_PointType>::Ptr keypoints(new pcl::PointCloud<T_PointType>);
//	const boost::shared_ptr<pcl::VoxelGrid<T_PointType>> sor(new pcl::VoxelGrid<T_PointType>); // なぜかPtrがprotectedなので
//	sor->setInputCloud(pointcloud);
//	sor->setLeafSize(voxel_size, voxel_size, voxel_size);
//	sor->filter(*keypoints);
//
//	// 法線推定
//	const float radius_normal = voxel_size * 2.0;
//	const auto view_point = T_PointType(0.0, 10.0, 10.0);
//
//	const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	const pcl::NormalEstimation<T_PointType, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<T_PointType, pcl::Normal>);
//	const pcl::search::KdTree<T_PointType>::Ptr kdtree(new pcl::search::KdTree<T_PointType>);
//	ne->setInputCloud(pointcloud);
//	ne->setRadiusSearch(radius_normal);
//	ne->setSearchMethod(kdtree);
//	ne->setViewPoint(view_point.x, view_point.y, view_point.z);
//	ne->compute(*normals);
//
//	// FPFH特徴量計算
//	const float radius_feature = voxel_size * 5.0;
//
//	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
//	const pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<T_PointType, pcl::Normal, pcl::FPFHSignature33>);
//	fpfhe->setInputCloud(keypoints);
//	fpfhe->setSearchSurface(pointcloud); // Open3Dだとこれができない（？）
//	fpfhe->setInputNormals(normals);
//	fpfhe->setSearchMethod(kdtree);
//	fpfhe->setRadiusSearch(radius_feature);
//	fpfhe->compute(*fpfh);
//
//	return std::make_pair(keypoints, fpfh);
//}
//
//auto execute_global_registration(
//	const std::pair<const pcl::PointCloud<T_PointType>::Ptr, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr>& scene1,
//	const std::pair<const pcl::PointCloud<T_PointType>::Ptr, const pcl::PointCloud<pcl::FPFHSignature33>::Ptr>& scene2,
//	const float voxel_size)
//{
//	const auto& kp1 = scene1.first;
//	const auto& kp2 = scene2.first;
//	const auto& fpfh1 = scene1.second;
//	const auto& fpfh2 = scene2.second;
//
//	const float distance_threshold = voxel_size * 2.5;
//
//	const pcl::PointCloud<T_PointType>::Ptr output(new pcl::PointCloud<T_PointType>);
//	const pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33>::Ptr align(new pcl::SampleConsensusPrerejective<T_PointType, T_PointType, pcl::FPFHSignature33>);
//	align->setInputSource(kp1);
//	align->setSourceFeatures(fpfh1);
//	align->setInputTarget(kp2);
//	align->setTargetFeatures(fpfh2);
//	//align->setMaximumIterations(500000);
//	align->setMaximumIterations(50);	//8 & 8
//	//align->setMaximumIterations(200);
//	align->setNumberOfSamples(4);
//	align->setCorrespondenceRandomness(2);
//	align->setSimilarityThreshold(0.9f);
//	align->setMaxCorrespondenceDistance(distance_threshold);
//	align->setInlierFraction(0.25f);
//	align->align(*output);
//	cout << "align->getFitnessScore():" << align->getFitnessScore() << endl;
//	cout << "align->hasConverged():" << align->hasConverged() << endl;
//	return align->getFinalTransformation();
//}
//
//auto draw_registration_result(
//	const std::string& window_name,
//	const pcl::PointCloud<T_PointType>::Ptr& scene1,
//	const pcl::PointCloud<T_PointType>::Ptr& scene2,
//	const Eigen::Matrix4f& trans)
//{
//	const pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
//	viewer->setBackgroundColor(255.0, 255.0, 255.0);
//
//	const auto scene1_temp = scene1->makeShared();
//	const auto scene2_temp = scene2->makeShared();
//
//	pcl::transformPointCloud(*scene1_temp, *scene1_temp, trans);
//
//	const pcl::visualization::PointCloudColorHandlerCustom<T_PointType>::Ptr color1(new pcl::visualization::PointCloudColorHandlerCustom<T_PointType>(scene1_temp, 1.0 * 255, 0.706 * 255, 0.0 * 255));
//	const pcl::visualization::PointCloudColorHandlerCustom<T_PointType>::Ptr color2(new pcl::visualization::PointCloudColorHandlerCustom<T_PointType>(scene2_temp, 0.0 * 255, 0.651 * 255, 0.929 * 255));
//	viewer->addPointCloud<T_PointType>(scene1_temp, *color1, "scene1");
//	viewer->addPointCloud<T_PointType>(scene2_temp, *color2, "scene2");
//
//	return viewer;
//}
//
//int maint()
//{
//	// 読み込み
//	//const auto scene1 = read_point_cloud("scene1.ply");
//	//const auto scene2 = read_point_cloud("scene2.ply");
//	//const auto scene1 = read_point_cloud();
//	//const auto scene2 = read_point_cloud("\../../data/_Naraha_output_and_heavy/rawPointCloud/009.pcd");
//	pcl::PointCloud<T_PointType>::Ptr scene1(new pcl::PointCloud<T_PointType>());
//	pcl::PointCloud<T_PointType>::Ptr scene2(new pcl::PointCloud<T_PointType>());
//	pcl::io::loadPCDFile("../../data/008XYZRGB_naraha.pcd", *scene1);
//	//pcl::io::loadPCDFile("../../data/009XYZRGB_naraha.pcd", *scene2);
//	pcl::io::loadPCDFile("../../data/008XYZRGB_naraha.pcd", *scene2);
//	// scene2 を適当に回転・並進
//	Eigen::Matrix4f transform_matrix;
//	transform_matrix <<
//		1.0, 0.0, 0.0, -0.1,
//		0.0, 0.0, -1.0, 0.1,
//		0.0, 1.0, 0.0, -0.1,
//		0.0, 0.0, 0.0, 1;
//	pcl::transformPointCloud(*scene2, *scene2, transform_matrix);
//
//	// 位置合わせ前の点群の表示
//	const auto viewer1 = draw_registration_result("Initial", scene1, scene2, Eigen::Matrix4f::Identity());
//
//	//const float voxel_size = 0.01;
//	//const float voxel_size = 0.05;
//	const float voxel_size = 0.1;
//
//	cout << "preprocess" << endl;
//	//RANSAC による Global Registration
//	const auto scene1_kp_fpfh = preprocess_point_cloud(scene1, voxel_size);
//	const auto scene_kp_fpfh = preprocess_point_cloud(scene2, voxel_size);
//	cout << "RANSAC" << endl;
//	const auto result_ransac = execute_global_registration(scene1_kp_fpfh, scene_kp_fpfh, voxel_size);
//	const auto viewer2 = draw_registration_result("Global", scene1, scene2, result_ransac);
//
//	//// ICP による refine
//	//const auto result = refine_registration(scene1, scene2, result_ransac, voxel_size);
//	//const auto viewer3 = draw_registration_result("Refined", scene1, scene2, result * result_ransac);
//	//const auto viewer3 = draw_registration_result("Refined", scene1, scene2, result * result_ransac);
//
//	//while (!viewer1->wasStopped() && !viewer2->wasStopped() && !viewer3->wasStopped())
//	while (!viewer1->wasStopped() && !viewer2->wasStopped())
//	{
//		viewer1->spinOnce();
//		viewer2->spinOnce();
//	}
//}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

//typedef pcl::PointXYZ T_PointType;
typedef pcl::PointXYZRGB T_PointType;


pcl::PointCloud<pcl::Normal>::Ptr surface_normals(pcl::PointCloud<T_PointType>::Ptr cloud)
{

	pcl::NormalEstimation<T_PointType, pcl::Normal> ne;
	ne.setInputCloud(cloud);//法線の計算を行いたい点群を指定する

	pcl::search::KdTree<T_PointType>::Ptr tree(new pcl::search::KdTree<T_PointType>());//KDTREEを作る
	ne.setSearchMethod(tree);//検索方法にKDTREEを指定する

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);//法線情報を入れる変数

	//ne.setRadiusSearch(0.005);//検索する半径を指定する
	ne.setRadiusSearch(0.3);//検索する半径を指定する

	ne.compute(*cloud_normals);//法線情報の出力先を指定する

	return cloud_normals;
}

int main()
{
	pcl::PointCloud<T_PointType>::Ptr cloud(new pcl::PointCloud<T_PointType>);
	pcl::io::loadPCDFile("../../data/008XYZRGB_naraha.pcd", *cloud);

	//surface_normals(cloud);

	pcl::visualization::PCLVisualizer viewer("3D Viewer");
	//viewer.setBackgroundColor(1.0, 0.5, 1.0);
	viewer.setBackgroundColor(0., 0., 0.);
	viewer.addPointCloud<T_PointType>(cloud, "Input cloud");
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Input cloud");
	//viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, surface_normals(cloud), 10, 0.05, "normals");
	//viewer.addPointCloudNormals<T_PointType, pcl::Normal>(cloud, surface_normals(cloud), 10, 0.5, "normals");
	viewer.addPointCloudNormals<T_PointType, pcl::Normal>(cloud, surface_normals(cloud), 10, 0.1, "normals");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return (0);
}