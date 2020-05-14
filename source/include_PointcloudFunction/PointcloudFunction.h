#pragma once

#include<iostream>
#include <vector>
#include<windows.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/cloud_viewer.h>


#define M_PI 3.14159265359

using namespace std;
typedef pcl::PointXYZI PointType;
//typedef pcl::PointXYZRGB PointType;

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

class CPointcloudFuction
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;				//ëÂè‰ïvÅH

public:

	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);

	static Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

	static Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d transformation_Node);

	void initVisualizer(const std::type_info& type);

};
