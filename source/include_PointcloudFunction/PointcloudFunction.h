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
//typedef pcl::PointXYZI PointType;
//typedef pcl::PointXYZRGB PointType;

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

template < typename T_PointType >	class CPointVisualization
{

	pcl::visualization::PointCloudColorHandler<T_PointType>::Ptr m_handler;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;				//ëÂè‰ïvÅH
	string name_window;

public:
	//template < typename T_PointType >
	//CPointVisualization();

	CPointVisualization();

	void setViewer(const string name_window_arg);


	//m_viewer->close();
};


class CPointcloudFuction
{

public:
	CPointcloudFuction() 
	{

	}

	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);

	static Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

	static Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d transformation_Node);

	//pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>::Ptr m_handler;
	//pcl::visualization::PointCloudColorHandler<>::Ptr m_handler;


	//CPointVisualization<pcl::PointXYZ> pointv;
	//CPointVisualization<pcl::PointXYZ> pointv2("test");
	//CPointVisualization<pcl::PointXYZ> pointv3(const_cast<string>("test"));
	//CPointcloudFuction::CPointVisualization<pcl::PointXYZ> pointv4("test");

};
