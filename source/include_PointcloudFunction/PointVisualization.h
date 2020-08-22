#pragma once

#include<iostream>
#include <vector>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

//should be under pcl includes
#include<windows.h>
#include "TimeString.h"

#define M_PI 3.14159265359
#define D2R 0.017453288888889
#define R2D 57.29579143313326

template <class T_PointType>
class CPointVisualization
{
	//https://stackoverflow.com/questions/40544174/class-templating-to-process-pclpointcloud-objects-with-different-point-types
	typedef typename pcl::visualization::PointCloudColorHandler<T_PointType> T_ColorHandler;
	typedef typename pcl::PointCloud<T_PointType>::Ptr T_PointCloudPtr;
	typedef typename pcl::visualization::PointCloudColorHandlerCustom<T_PointType> T_ColorHandlerCustom;				//XYZ
	typedef typename pcl::visualization::PointCloudColorHandlerGenericField<T_PointType> T_ColorHandlerGenericField;	//XYZI
	typedef typename pcl::visualization::PointCloudColorHandlerRGBField<T_PointType> T_ColorHandlerRGBField;			//XYZRGB

//public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> M_viewer;
//private:
	boost::shared_ptr<T_ColorHandler> M_handler;
	string M_name_window;
	boost::mutex M_mutex;
	static boost::mutex M_mutex_viewer;
	T_PointCloudPtr M_cloud_;

	pcl::PointCloud<pcl::Normal>::Ptr M_cloud_normal;
	bool M_b_useNormal;
	double M_radius_nor;
	int M_level_nor_vis;
	float M_scale_nor_vis;

	boost::thread M_thread;

	bool M_b_showWindow;

	void startThread(T_PointCloudPtr cloud_arg);
	void doThread();
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawNumber_OnetoNine_pointcloud(
		pcl::PointXYZRGB point_center, int num_arg, double length_horizontal_arg);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr expandDiagram(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, float times_);

	T_PointCloudPtr M_cloud_second;
	int M_num_PointCloudShowing;
	int M_i_viewer1, M_i_viewer2;

	int M_num_text;

	vector<string> M_s_arrow_vec, M_s_line_vec, M_s_cylinder_vec, M_s_number_vec, M_s_trajectory_vec;
	void drawNumber_OnetoNine(pcl::PointXYZRGB point_center, int num_arg, double length_horizontal_arg, string s_name);

	pcl::PointXYZRGB addPointXYZRGB(pcl::PointXYZRGB point1, pcl::PointXYZRGB point2)
	{
		pcl::PointXYZRGB point;
		point.x = point1.x + point2.x; point.y = point1.y + point2.y; point.z = point1.z + point2.z;
		point.r = point1.r + point2.r; point.g = point1.g + point2.g; point.b = point1.b + point2.b;
		return point;
	}

	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(Eigen::Vector6d XYZRPY_arg);

	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);
	static Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);
	static Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

public:

	CPointVisualization();
	~CPointVisualization();

	void setWindowName(const string name_window_arg);

	void setPointCloud(T_PointCloudPtr cloud_arg);
	void setPointCloud(T_PointCloudPtr cloud_arg, string s_name);
	void setPointCloud(T_PointCloudPtr cloud_arg1, T_PointCloudPtr cloud_arg2);
	void updateViewer();
	void closeViewer();
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawLine_pointcloud(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr proliferation_rand(pcl::PointXYZRGB point_arg, int times);
	static  bool isPointOutOnLineSegmant(
		pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_, pcl::PointXYZRGB point3_);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawArrow_pointcloud(
		pcl::PointXYZRGB point_arg, double Roll_, double Pitch_, double Yaw);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawNumber_pointcloud(
		pcl::PointXYZRGB point_center, int num_arg);

	static vector<std::uint8_t> getRGBwithValuebyHSV(double value_, double value_max, double value_min);
	void useNormal(double radius_arg, int level_arg, float scale_arg);
	void addProcess_InsetPointCloud();

	void drawArrow(pcl::PointXYZRGB point_arg, double Roll_, double Pitch_, double Yaw_, string s_name = "");
	void drawLine(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_, string s_name = "");
	void drawLine_cylinder(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_, double radius_, string s_name = "");
	void drawNumber(pcl::PointXYZRGB point_center, int num_arg, string s_name = "");
	void drawTrajectory(vector<Eigen::Vector6d> trajectory_vec_vec, int i_frame, bool b_showNumber, bool b_showArrow, string s_name = "");
	void deleteTrajectory(string s_name = "");

	void addGrid(float x_max_arg, float y_max_arg, float x_min_arg, float y_min_arg)
	{
		typedef pcl::PointXYZRGB T_PointType;
		T_PointType point_color;
		//point_color.r = 255;
		//point_color.g = 255;
		point_color.r = 180;
		point_color.g = 180;
		point_color.b = 0;

		int x_max, y_max, x_min, y_min;
		x_max = (int)(x_max_arg + 1);
		y_max = (int)(y_max_arg + 1);
		x_min = (int)(x_min_arg);
		y_min = (int)(y_min_arg);

		//point pair
		pcl::PointCloud<T_PointType>::Ptr cloud_min(new pcl::PointCloud<T_PointType>);
		pcl::PointCloud<T_PointType>::Ptr cloud_max(new pcl::PointCloud<T_PointType>);

		//horizontal
		for (int j = y_min; j < y_max + 1; j++)
		{
			T_PointType point_min;
			point_min = point_color;
			point_min.x = x_min;
			point_min.y = (float)j;
			point_min.z = 0.;
			cloud_min->push_back(point_min);
			T_PointType point_max;
			point_max = point_color;
			point_max.x = x_max;
			point_max.y = (float)j;
			point_max.z = 0.;
			cloud_max->push_back(point_max);
		}
		//vertical
		for (int i = x_min; i < x_max + 1; i++)
		{
			T_PointType point_min;
			point_min = point_color;
			point_min.x = (float)i;
			point_min.y = y_min;
			point_min.z = 0.;
			cloud_min->push_back(point_min);
			T_PointType point_max;
			point_max = point_color;
			point_max.x = (float)i;
			point_max.y = y_max;
			point_max.z = 0.;
			cloud_max->push_back(point_max);
		}

		//draw line
		for (int j = 0; j < cloud_min->size(); j++)
			drawLine(cloud_min->points[j], cloud_max->points[j], "grid_line" + to_string(j));
	}
};

//https://qiita.com/i153/items/38f9688a9c80b2cb7da7
template < typename T_PointType >
CPointVisualization<T_PointType>::CPointVisualization()
	:
	M_num_PointCloudShowing(1)
	, M_i_viewer1(0)
	, M_i_viewer2(0)
	, M_cloud_(new pcl::PointCloud<T_PointType>)
	, M_cloud_second(new pcl::PointCloud<T_PointType>)
	, M_cloud_normal(new pcl::PointCloud<pcl::Normal>)
	, M_b_useNormal(false)
	, M_num_text(0)
{
	setWindowName("init");

	//m_viewer->removeCoordinateSystem("coordinate");		//remove axis in viewer

	// Point Cloud Color Handler
	const std::type_info& type = typeid(T_PointType);
	if (type == typeid(pcl::PointXYZ))
	{
		std::vector<double> color = { 255.0, 255.0, 255.0 };
		boost::shared_ptr<T_ColorHandlerCustom> color_handler(new T_ColorHandlerCustom(color[0], color[1], color[2]));
		M_handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZI))
	{
		boost::shared_ptr<T_ColorHandlerGenericField> color_handler(new T_ColorHandlerGenericField("intensity"));
		M_handler = color_handler;
	}
	else if (type == typeid(pcl::PointXYZRGB)) {
		boost::shared_ptr<T_ColorHandlerRGBField> color_handler(new T_ColorHandlerRGBField());
		M_handler = color_handler;
	}
	//else if (type == typeid(pcl::PointXYZRGB)) {
	//	boost::shared_ptr<T_ColorHandlerCustom> color_handler(new T_ColorHandlerCustom());
	//	M_handler = color_handler;
	//}
	//else if (type == typeid(pcl::PointXYZRGBA)) {
	//	boost::shared_ptr<T_ColorHandlerCustom> color_handler(new T_ColorHandlerCustom());
	//	M_handler = color_handler;
	//}
	else {
		throw std::runtime_error("This PointType is unsupported.");
	}

}

template < typename T_PointType >
Eigen::Matrix4d CPointVisualization<T_PointType>::calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
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

template < typename T_PointType >
Eigen::Matrix4d CPointVisualization<T_PointType>::calcHomogeneousMatrixFromVector6d(Eigen::Vector6d XYZRPY_arg)
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


template < typename T_PointType >
Eigen::Affine3f CPointVisualization<T_PointType>::calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat)
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

template < typename T_PointType >
Eigen::Vector6d CPointVisualization<T_PointType>::calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat)
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

template < typename T_PointType >
void CPointVisualization<T_PointType>::setWindowName(const string name_window_arg)
{
	M_name_window = name_window_arg;
	if (M_viewer)
	{
		cout << "reset viewer" << endl;
		M_viewer->close();
	}
	M_viewer.reset(new pcl::visualization::PCLVisualizer(name_window_arg));
	M_viewer->initCameraParameters();


	//cout << "M_num_PointCloudShowing:" << M_num_PointCloudShowing << endl;

	if (M_num_PointCloudShowing == 1)
	{
		M_viewer->addCoordinateSystem(3.0, "coordinate");
		M_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
		M_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
		//m_viewer->removeCoordinateSystem("coordinate");		//remove axis in viewer
	}
	else if (M_num_PointCloudShowing == 2)
	{
		M_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, M_i_viewer1);
		M_viewer->addCoordinateSystem(3.0, "coordinate", M_i_viewer1);
		M_viewer->setBackgroundColor(0.0, 0.0, 0.0, M_i_viewer1);
		M_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, M_i_viewer1);

		M_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, M_i_viewer2);
		M_viewer->addCoordinateSystem(3.0, "coordinate", M_i_viewer2);
		M_viewer->setBackgroundColor(0.03, 0.03, 0.03, M_i_viewer2);
		M_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, M_i_viewer2);
	}
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::setPointCloud(T_PointCloudPtr cloud_arg)
{
	//mutex with scoped_lock
	boost::mutex::scoped_lock lock(M_mutex);
	//update pointcloud
	M_cloud_->clear();
	pcl::copyPointCloud(*cloud_arg, *M_cloud_);

	if (M_num_PointCloudShowing == 2)
	{
		M_num_PointCloudShowing = 1;
		setWindowName(M_name_window);
	}

	if (!M_b_useNormal || M_cloud_->size() == 0) return;
	addProcess_InsetPointCloud();
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::setPointCloud(T_PointCloudPtr cloud_arg, string s_name)
{
	if (M_num_text == 0)
		M_viewer->addText(s_name, 0., 0., 20, 1., 1., 1., "text");
	else
		M_viewer->updateText(s_name, 0., 0., "text");

	M_num_text++;
	setPointCloud(cloud_arg);
}


template < typename T_PointType >
void CPointVisualization<T_PointType>::setPointCloud(T_PointCloudPtr cloud_arg1, T_PointCloudPtr cloud_arg2)
{
	//mutex with scoped_lock
	boost::mutex::scoped_lock lock(M_mutex);
	//update pointcloud
	M_cloud_->clear();
	pcl::copyPointCloud(*cloud_arg1, *M_cloud_);
	M_cloud_second->clear();
	pcl::copyPointCloud(*cloud_arg2, *M_cloud_second);

	if (M_num_PointCloudShowing == 1)
	{
		M_num_PointCloudShowing = 2;
		setWindowName(M_name_window);
	}

	if (M_b_useNormal && M_num_PointCloudShowing == 2)
	{
		cout << "ERROR: normal cannot be used with 2 pointcloud showing." << endl;
		throw std::runtime_error("ERROR: normal cannot be used with 2 pointcloud showing");
	}

	return;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::addProcess_InsetPointCloud()
{
	{
		//http://virtuemarket-lab.blogspot.com/2015/02/blog-post_35.html
		pcl::NormalEstimation<T_PointType, pcl::Normal> ne;
		ne.setInputCloud(M_cloud_);
		pcl::search::KdTree<T_PointType>::Ptr tree(new pcl::search::KdTree<T_PointType>());
		ne.setSearchMethod(tree);
		ne.setRadiusSearch(M_radius_nor);
		ne.compute(*M_cloud_normal);
		cout << "M_cloud_normal->size():" << M_cloud_normal->size() << endl;
	}

	static int index_normal = 0;
	if (M_b_useNormal && M_cloud_->size() != 0 && M_cloud_normal->size() != 0)
	{
		if (index_normal != 0)
			M_viewer->removePointCloud("normals " + to_string(index_normal - 1));
		M_viewer->addPointCloudNormals<T_PointType, pcl::Normal>(
			M_cloud_, M_cloud_normal, M_level_nor_vis, M_scale_nor_vis, "normals " + to_string(index_normal));
		index_normal++;
	}
}


template < typename T_PointType >
void CPointVisualization<T_PointType>::updateViewer()
{
	if (!M_viewer->wasStopped())
	{
		//spinOnce() error in thread
		//https://stackoverflow.com/questions/21155658/using-pclvisualization-in-different-threads-from-different-instance-of-a-cla
		//http://www.pcl-users.org/PCL-viewer-hangs-with-busy-cursor-td4021568.html
		//http://www.pcl-users.org/Updating-PCLVisualizer-from-another-thread-td4021570.html
		M_viewer->spinOnce();

		//mutex with no stop
		boost::mutex::scoped_try_lock lock(M_mutex);

		if (M_num_PointCloudShowing == 1)
		{
			if (lock.owns_lock() && M_cloud_)
			{
				//cout << "locked";
				//cout << "(WindowName:" << M_name_window << ")" << endl;
				M_handler->setInputCloud(M_cloud_);
				//update viewer
				if (!M_viewer->updatePointCloud(M_cloud_, *M_handler, "cloud1"))
					M_viewer->addPointCloud(M_cloud_, *M_handler, "cloud1");
			}
		}
		else if (M_num_PointCloudShowing == 2)
		{
			if (lock.owns_lock() && M_cloud_ && M_cloud_second)
			{
				//cout << "locked";
				//cout << "(WindowName:" << M_name_window << ")" << endl;
				M_handler->setInputCloud(M_cloud_);
				//update viewer
				if (!M_viewer->updatePointCloud(M_cloud_, *M_handler, "cloud1"))
					M_viewer->addPointCloud(M_cloud_, *M_handler, "cloud1", M_i_viewer1);
				if (!M_viewer->updatePointCloud(M_cloud_second, *M_handler, "cloud2"))
					M_viewer->addPointCloud(M_cloud_second, *M_handler, "cloud2", M_i_viewer2);
			}
		}
	}
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::closeViewer()
{
	M_viewer->close();
}

//don't work
template < typename T_PointType >
void CPointVisualization<T_PointType>::startThread(T_PointCloudPtr cloud_arg)
{
	M_cloud_->clear();
	pcl::copyPointCloud(*cloud_arg, *M_cloud_);
	cout << "size = " << M_cloud_->size() << endl;
	M_thread = boost::thread(&CPointVisualization<T_PointType>::doThread, this);
}

//don't work
template < typename T_PointType >
void CPointVisualization<T_PointType>::doThread()
{
	//while (!M_viewer->wasStopped())
	//{
	//	cout << "not stoped" << endl;

	//	M_viewer->spinOnce(100);
	//	cout << "not stoped" << endl;

	//	//mutex with no stop
	//	boost::mutex::scoped_try_lock lock(M_mutex);
	//	//cout << "bool:" << lock.owns_lock() << endl;
	//	if (lock.owns_lock() && M_cloud_)
	//	{
	//		M_handler->setInputCloud(M_cloud_);
	//		//update viewer
	//		//if (!M_viewer->updatePointCloud(M_cloud_, *M_handler, "cloud"))
	//		//	M_viewer->addPointCloud(M_cloud_, *M_handler, "cloud");
	//		if (!M_viewer->updatePointCloud(M_cloud_, *M_handler, "cloud"))
	//			M_viewer->addPointCloud(M_cloud_, *M_handler, "cloud");
	//		cout << "thread" << endl;
	//	}
	//	break;
	//}

	while (1)
	{
		cout << "while start" << endl;
		CTimeString time_;
		string t1 = time_.getTimeString();
		updateViewer();
		string t2 = time_.getTimeString();
		cout << "elapsed:" << time_.getTimeElapsefrom2Strings_millisec(t1, t2) << endl;
	}
}

template < typename T_PointType >
CPointVisualization<T_PointType>::~CPointVisualization()
{
	if (M_thread.joinable()) M_thread.join();
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawLine_pointcloud(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_)
{
	double distance_ = 0.01;//:distance point to point [m]
	Eigen::Vector3d direction_vec = Eigen::Vector3d::Zero();
	direction_vec <<
		point2_.x - point1_.x,
		point2_.y - point1_.y,
		point2_.z - point1_.z;

	direction_vec <<
		direction_vec(0, 0) / direction_vec.norm(),
		direction_vec(1, 0) / direction_vec.norm(),
		direction_vec(2, 0) / direction_vec.norm();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
	int index_ = 0;

	while (1)
	{
		pcl::PointXYZRGB point_;
		point_ = point1_;
		point_.x = point1_.x + index_ * distance_ * direction_vec(0, 0);
		point_.y = point1_.y + index_ * distance_ * direction_vec(1, 0);
		point_.z = point1_.z + index_ * distance_ * direction_vec(2, 0);
		if (isPointOutOnLineSegmant(point1_, point2_, point_)) break;
		cloud_temp->push_back(point_);
		index_++;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (int i = 0; i < cloud_temp->size(); i++)
		*cloud_ += *proliferation_rand(cloud_temp->points[i], 2);

	return cloud_;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::drawLine(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_, string s_name)
{
	string name_id;
	if (s_name.size() == 0)
		name_id = "line" + to_string(M_s_line_vec.size());
	else
		name_id = s_name;

	M_viewer->addLine(point1_, point2_, (float)point1_.r / 255., (float)point1_.g / 255.,
		(float)point1_.b / 255., name_id);

	M_s_line_vec.push_back(name_id);

	return;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::drawLine_cylinder(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_, double radius_, string s_name)
{
	string name_id;
	if (s_name.size() == 0)
		name_id = "cylinder" + to_string(M_s_cylinder_vec.size());
	else
		name_id = s_name;

	Eigen::Vector3d E_ = Eigen::Vector3d::Zero();
	E_ << point1_.x - point2_.x, point1_.y - point2_.y, point1_.z - point2_.z;
	E_ / E_.norm();

	pcl::ModelCoefficients coeff_cylinder;
	coeff_cylinder.values.resize(7);
	coeff_cylinder.values[0] = point2_.x;
	coeff_cylinder.values[1] = point2_.y;
	coeff_cylinder.values[2] = point2_.z;
	coeff_cylinder.values[3] = E_(0, 0);
	coeff_cylinder.values[4] = E_(1, 0);
	coeff_cylinder.values[5] = E_(2, 0);
	coeff_cylinder.values[6] = radius_;

	M_viewer->addCylinder(coeff_cylinder, name_id);
	//add color reference:http://www.pcl-users.org/Shading-for-PCLVisualizer-addCylinder-td4038645.html
	M_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
		(float)point1_.r / 255., (float)point1_.g / 255., (float)point1_.b / 255., name_id);

	M_s_cylinder_vec.push_back(name_id);

	return;
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::proliferation_rand(pcl::PointXYZRGB point_arg, int times)
{
	//îCà”ÇÃóêêîê∂ê¨äÌÅAï™ïzÇê›íËâ¬î\
	boost::mt19937 gen(static_cast<unsigned long>(time(0)));
	//boost::uniform_real<> dst(-0.1, 0.1);
	boost::uniform_real<> dst(-0.01, 0.01);
	boost::variate_generator< boost::mt19937&, boost::uniform_real<> > rand(gen, dst);// óêêîê∂ê¨
	//rand();// 47

	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());
	double sigma_x;
	sigma_x = 0.01;
	normal_distribution<> dist(0., sigma_x);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	int index_ = 0;
	while (1)
	{
		pcl::PointXYZRGB point_;
		point_ = point_arg;
		//point_.x += rand();
		//point_.y += rand();
		//point_.z += rand();
		point_.x += dist(engine);
		point_.y += dist(engine);
		point_.z += dist(engine);
		cloud_->push_back(point_);
		index_++;
		if (index_ == times) break;
	}

	return cloud_;
}

template < typename T_PointType >
bool CPointVisualization<T_PointType>::isPointOutOnLineSegmant(
	pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_, pcl::PointXYZRGB point3_)
{
	if (point1_.x < point2_.x)
	{
		if (point3_.x > point2_.x) return true;
	}
	else if (point1_.x > point2_.x)
	{
		if (point3_.x < point2_.x) return true;
	}
	else
	{
		if (point1_.y < point2_.y)
		{
			if (point3_.y > point2_.y) return true;
		}
		else if (point1_.y > point2_.y)
		{
			if (point3_.y < point2_.y) return true;
		}
		else
		{
			if (point1_.z < point2_.z)
			{
				if (point3_.z > point2_.z) return true;
			}
			else if (point1_.z > point2_.z)
			{
				if (point3_.z < point2_.z) return true;
			}
			else
			{
				throw std::runtime_error("ERROR: invalid positon of the points which making line.");
			}
		}
	}
	return false;
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawArrow_pointcloud(
	pcl::PointXYZRGB point_arg, double Roll_, double Pitch_, double Yaw_)
{
	//drawing 4 lines. a line and a triangle.
	//First, root to center of bottom of triangle.
	//Second, left point of bottom to right.
	//Third, right point of bottom to top.
	//Fourth, left point of bottom to top.

	double length_arraw = 0.3 * 5.;
	double lenght_half_arraw;
	lenght_half_arraw = length_arraw / 2.;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PointXYZRGB point_Root;
	point_Root = point_arg;
	point_Root.x = 0.;
	point_Root.y = 0.;
	point_Root.z = 0.;

	pcl::PointXYZRGB point_BottomCenter;
	point_BottomCenter = point_Root;
	point_BottomCenter.x = lenght_half_arraw;

	pcl::PointXYZRGB point_BottomLeft;
	point_BottomLeft = point_BottomCenter;
	point_BottomLeft.y = lenght_half_arraw / 3.;

	pcl::PointXYZRGB point_BottomRight;
	point_BottomRight = point_BottomCenter;
	point_BottomRight.y = -lenght_half_arraw / 3.;

	pcl::PointXYZRGB point_Top;
	point_Top = point_BottomCenter;
	point_Top.x = lenght_half_arraw * 2.;

	*cloud_ += *drawLine_pointcloud(point_Root, point_BottomCenter);
	*cloud_ += *drawLine_pointcloud(point_BottomLeft, point_BottomRight);
	*cloud_ += *drawLine_pointcloud(point_BottomRight, point_Top);
	*cloud_ += *drawLine_pointcloud(point_BottomLeft, point_Top);

	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::copyPointCloud(*cloud_, *cloud_temp);
		cloud_->clear();
		for (int i = 0; i < cloud_temp->size(); i++)
			*cloud_ += *proliferation_rand(cloud_temp->points[i], 2);
	}
	//cloud_ = expandDiagram(cloud_, 5.);

	Eigen::Affine3f Trans_ = Eigen::Affine3f::Identity();
	Trans_ = CPointcloudFunction::calcAffine3fFromHomogeneousMatrix(
		CPointcloudFunction::calcHomogeneousMatrixFromVector6d(
			point_arg.x, point_arg.y, point_arg.z, Roll_, Pitch_, Yaw_));
	pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

	return cloud_;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::drawArrow(
	pcl::PointXYZRGB point_arg, double Roll_, double Pitch_, double Yaw_, string s_name)
{
	string name_id;
	if (s_name.size() == 0)
		name_id = "arrow" + to_string(M_s_arrow_vec.size());
	else
		name_id = s_name;

	double length_arraw = 0.3 * 5.;
	pcl::PointXYZRGB point_start;
	pcl::PointXYZRGB point_end;
	point_end.x = length_arraw;
	Eigen::Matrix4d point_end_homogeneous = Eigen::Matrix4d::Identity();
	point_end_homogeneous =
		calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., Yaw_)
		*calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., Pitch_, 0.)
		*calcHomogeneousMatrixFromVector6d(0., 0., 0., Roll_, 0., 0.)
		*calcHomogeneousMatrixFromVector6d(
			point_end.x, point_end.y, point_end.z, 0., 0., 0.);
	point_end.x = point_end_homogeneous(0, 3);
	point_end.y = point_end_homogeneous(1, 3);
	point_end.z = point_end_homogeneous(2, 3);
	point_start = point_arg;
	point_end.x += point_arg.x;
	point_end.y += point_arg.y;
	point_end.z += point_arg.z;

	M_viewer->addArrow(point_end, point_start, (float)(point_arg.r) / 255.,
		(float)(point_arg.g) / 255., (float)(point_arg.b) / 255., false, name_id);

	M_s_arrow_vec.push_back(name_id);

	return;
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::expandDiagram(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, float times_)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (int i = 0; i < cloud_arg->size(); i++)
	{
		pcl::PointXYZRGB point_;
		point_ = cloud_arg->points[i];
		point_.x = cloud_arg->points[i].x * times_;
		point_.y = cloud_arg->points[i].y * times_;
		point_.z = cloud_arg->points[i].z * times_;
		cloud_->push_back(point_);
	}

	return cloud_;
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawNumber_pointcloud(
	pcl::PointXYZRGB point_center, int num_arg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	double length_horizontal_ = 0.5;
	int index_ = 0;
	double range_ = 0.8;
	double height_gap = 0.1;
	//point_center.z += height_gap * M_num_number;
	while (1)
	{
		*cloud_ += *drawNumber_OnetoNine_pointcloud(point_center, num_arg % 10, length_horizontal_);
		num_arg /= 10;
		if (num_arg == 0) break;
		point_center.x += -range_;
		index_++;
	}

	//M_num_number++;

	return cloud_;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::drawNumber(pcl::PointXYZRGB point_center, int num_arg, string s_name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	double length_horizontal_ = 0.5;
	int index_ = 0;
	double range_ = 0.8;
	double height_gap = 0.1;
	point_center.z -= height_gap * (M_s_number_vec.size() + 1);

	string name_id;
	if (s_name.size() == 0)
		name_id = "number" + to_string(M_s_number_vec.size());
	else
		name_id = s_name;

	while (1)
	{
		drawNumber_OnetoNine(point_center, num_arg % 10, length_horizontal_, name_id + "_one" + to_string(index_));
		num_arg /= 10;
		if (num_arg == 0) break;
		point_center.x += -range_;
		index_++;
	}

	M_s_number_vec.push_back(name_id);

	return;
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawNumber_OnetoNine_pointcloud(
	pcl::PointXYZRGB point_center, int num_arg, double length_horizontal_arg)
{
	//minus atomawashi

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	//double length_half_vertical = 0.25;
	//double length_horizontal = 0.3;
	double length_horizontal = length_horizontal_arg;
	double length_half_vertical = length_horizontal / 1.2;

	if (!(0 <= num_arg && num_arg <= 9))
	{
		throw std::runtime_error("ERROR: Given number is out of range.");
	}

	pcl::PointXYZRGB point_LeftUp;
	pcl::PointXYZRGB point_LeftCenter;
	pcl::PointXYZRGB point_LeftDown;
	pcl::PointXYZRGB point_RightUp;
	pcl::PointXYZRGB point_RightCenter;
	pcl::PointXYZRGB point_RightDown;

	point_LeftUp = point_center;
	point_LeftCenter = point_center;
	point_LeftDown = point_center;
	point_RightUp = point_center;
	point_RightCenter = point_center;
	point_RightDown = point_center;

	point_LeftUp.x = 0.;
	point_LeftCenter.x = 0.;
	point_LeftDown.x = 0.;
	point_LeftUp.y = length_half_vertical * 2;
	point_LeftCenter.y = length_half_vertical;
	point_LeftDown.y = 0.;
	point_RightUp.x = length_horizontal;
	point_RightCenter.x = length_horizontal;
	point_RightDown.x = length_horizontal;
	point_RightUp.y = length_half_vertical * 2;
	point_RightCenter.y = length_half_vertical;
	point_RightDown.y = 0.;

	//https://qiita.com/hiloki@github/items/647cd302616d57ac84ec
	switch (num_arg)
	{
	case 0:
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		//*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 1:
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		//*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		//*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 2:
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		//*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 3:
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 4:
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		//*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 5:
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		//*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 6:
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		//*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 7:
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		//*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 8:
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	case 9:
		*cloud_ += *drawLine_pointcloud(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine_pointcloud(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine_pointcloud(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine_pointcloud(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine_pointcloud(point_RightCenter, point_RightUp);
		break;
	}

	Eigen::Affine3f Trans_ = Eigen::Affine3f::Identity();
	Trans_ = CPointcloudFunction::calcAffine3fFromHomogeneousMatrix(
		CPointcloudFunction::calcHomogeneousMatrixFromVector6d(
			-length_horizontal / 2. + point_center.x, -length_half_vertical + point_center.y,
			point_center.z, 0., 0., 0.));
	pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

	return cloud_;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::drawNumber_OnetoNine(
	pcl::PointXYZRGB point_center, int num_arg, double length_horizontal_arg, string s_name)
{
	//minus atomawashi
	double length_horizontal = length_horizontal_arg;
	double length_half_vertical = length_horizontal / 1.2;
	double width_ = 0.05;

	string name_id;
	name_id = s_name;

	if (!(0 <= num_arg && num_arg <= 9))
	{
		throw std::runtime_error("ERROR: Given number is out of range.");
	}

	pcl::PointXYZRGB point_LeftUp;
	pcl::PointXYZRGB point_LeftCenter;
	pcl::PointXYZRGB point_LeftDown;
	pcl::PointXYZRGB point_RightUp;
	pcl::PointXYZRGB point_RightCenter;
	pcl::PointXYZRGB point_RightDown;

	point_LeftUp = point_center;
	point_LeftCenter = point_center;
	point_LeftDown = point_center;
	point_RightUp = point_center;
	point_RightCenter = point_center;
	point_RightDown = point_center;

	point_LeftUp.x = 0.;
	point_LeftCenter.x = 0.;
	point_LeftDown.x = 0.;
	point_LeftUp.y = length_half_vertical * 2;
	point_LeftCenter.y = length_half_vertical;
	point_LeftDown.y = 0.;
	point_RightUp.x = length_horizontal;
	point_RightCenter.x = length_horizontal;
	point_RightDown.x = length_horizontal;
	point_RightUp.y = length_half_vertical * 2;
	point_RightCenter.y = length_half_vertical;
	point_RightDown.y = 0.;

	point_center.x -= point_RightUp.x / 2.;
	point_center.y -= point_RightUp.y / 2.;

	point_LeftUp = addPointXYZRGB(point_LeftUp, point_center);
	point_LeftCenter = addPointXYZRGB(point_LeftCenter, point_center);
	point_LeftDown = addPointXYZRGB(point_LeftDown, point_center);
	point_RightUp = addPointXYZRGB(point_RightUp, point_center);
	point_RightCenter = addPointXYZRGB(point_RightCenter, point_center);
	point_RightDown = addPointXYZRGB(point_RightDown, point_center);

	//https://qiita.com/hiloki@github/items/647cd302616d57ac84ec
	switch (num_arg)
	{

	case 0:
		drawLine_cylinder(point_LeftDown, point_RightDown, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_LeftCenter, point_RightCenter, width_);
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftDown, point_LeftCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_LeftUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	case 1:
		//drawLine_cylinder(point_LeftDown, point_RightDown, width_);
		//drawLine_cylinder(point_LeftCenter, point_RightCenter, width_);
		//drawLine_cylinder(point_LeftUp, point_RightUp, width_);
		//drawLine_cylinder(point_LeftDown, point_LeftCenter, width_);
		//drawLine_cylinder(point_LeftCenter, point_LeftUp, width_);
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	case 2:
		drawLine_cylinder(point_LeftDown, point_RightDown, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftDown, point_LeftCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_LeftCenter, point_LeftUp, width_);
		//drawLine_cylinder(point_RightDown, point_RightCenter, width_);
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	case 3:
		drawLine_cylinder(point_LeftDown, point_RightDown, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_LeftDown, point_LeftCenter, width_);
		//drawLine_cylinder(point_LeftCenter, point_LeftUp, width_);
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	case 4:
		//drawLine_cylinder(point_LeftDown, point_RightDown, width_);
		drawLine_cylinder(point_LeftCenter, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_LeftUp, point_RightUp, width_);
		//drawLine_cylinder(point_LeftDown, point_LeftCenter, width_);
		drawLine_cylinder(point_LeftCenter, point_LeftUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	case 5:
		drawLine_cylinder(point_LeftDown, point_RightDown, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_LeftDown, point_LeftCenter, width_);
		drawLine_cylinder(point_LeftCenter, point_LeftUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_RightCenter, point_RightUp, width_);
		break;
	case 6:
		drawLine_cylinder(point_LeftDown, point_RightDown, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftDown, point_LeftCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_LeftUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_RightCenter, point_RightUp, width_);
		break;
	case 7:
		//drawLine_cylinder(point_LeftDown, point_RightDown, width_);
		//drawLine_cylinder(point_LeftCenter, point_RightCenter, width_);
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_LeftDown, point_LeftCenter, width_);
		//drawLine_cylinder(point_LeftCenter, point_LeftUp, width_);
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	case 8:
		drawLine_cylinder(point_LeftDown, point_RightDown, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftDown, point_LeftCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_LeftUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	case 9:
		drawLine_cylinder(point_LeftDown, point_RightDown, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftCenter, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_LeftUp, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		//drawLine_cylinder(point_LeftDown, point_LeftCenter, width_);
		drawLine_cylinder(point_LeftCenter, point_LeftUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightDown, point_RightCenter, width_, name_id + to_string(M_s_cylinder_vec.size()));
		drawLine_cylinder(point_RightCenter, point_RightUp, width_, name_id + to_string(M_s_cylinder_vec.size()));
		break;
	}

	return;
}

template < typename T_PointType >
vector<std::uint8_t> CPointVisualization<T_PointType>::getRGBwithValuebyHSV(double value_, double value_max, double value_min)
{
	//digital gazou shori(2020/2/26), pp. 81-82.
	float H_;
	int h_;
	H_ = ((float)value_ - (float)value_min) / ((float)value_max - (float)value_min) * 2. * M_PI;
	if (H_ == 2. * M_PI) H_ = 0.;
	//cout << "H_:" << H_ * R2D << "[deg]" << endl;
	h_ = (int)(3. / M_PI * H_);
	float I_, P_, Q_, S_, T_;
	I_ = 1.;
	S_ = 1.;
	P_ = I_ * (1. - S_);
	Q_ = I_ * (1. - S_ * (3. / M_PI * H_ - (float)h_));
	T_ = I_ * (1. - S_ * (1. - 3. / M_PI * H_ + (float)h_));
	std::uint8_t I_uint8, P_uint8, Q_uint8, T_uint8;
	I_uint8 = (std::uint8_t)(int)(255 * I_);
	P_uint8 = (std::uint8_t)(int)(255 * P_);
	Q_uint8 = (std::uint8_t)(int)(255 * Q_);
	T_uint8 = (std::uint8_t)(int)(255 * T_);
	std::uint8_t R_, G_, B_;
	switch (h_)
	{
	case 0:
		R_ = I_uint8; G_ = T_uint8; B_ = P_uint8;
		break;
	case 1:
		R_ = Q_uint8; G_ = I_uint8; B_ = P_uint8;
		break;
	case 2:
		R_ = P_uint8; G_ = I_uint8; B_ = T_uint8;
		break;
	case 3:
		R_ = P_uint8; G_ = Q_uint8; B_ = I_uint8;
		break;
	case 4:
		R_ = T_uint8; G_ = P_uint8; B_ = I_uint8;
		break;
	case 5:
		R_ = I_uint8; G_ = P_uint8; B_ = Q_uint8;
		break;
	}
	vector<std::uint8_t> color_vec;
	color_vec.push_back(R_);
	color_vec.push_back(G_);
	color_vec.push_back(B_);
	return color_vec;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::useNormal(double radius_arg, int level_arg, float scale_arg)
{
	M_radius_nor = radius_arg;
	M_level_nor_vis = level_arg;
	M_scale_nor_vis = scale_arg;
	M_b_useNormal = true;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::drawTrajectory(vector<Eigen::Vector6d> trajectory_vec_vec, int i_frame, 
	bool b_showNumber, bool b_showArrow, string s_name)
{
	if (i_frame > trajectory_vec_vec.size() - 1) i_frame = trajectory_vec_vec.size() - 1;

	if (M_s_trajectory_vec.size() >= 2) throw std::runtime_error("ERROR: Readed trajectory number is over.");

	for (int i = 0; i < M_s_trajectory_vec.size(); i++)
	{
		if (s_name == M_s_trajectory_vec[i])
			deleteTrajectory(M_s_trajectory_vec[i]);
	}

	string name_id;
	if (s_name.size() == 0)
		name_id = "trajectory" + to_string(M_s_trajectory_vec.size());
	else
		name_id = s_name;

	vector<unsigned char> color_trajectory_first;
	vector<unsigned char> color_number_first;
	vector<unsigned char> color_arrow_first;
	//color_trajectory_first << 1, 1, 1;

	color_trajectory_first.push_back(153);
	color_trajectory_first.push_back(0);
	color_trajectory_first.push_back(255);
	color_number_first.push_back(255);
	color_number_first.push_back(0);
	color_number_first.push_back(0);
	color_arrow_first.push_back(255);
	color_arrow_first.push_back(102);
	color_arrow_first.push_back(0);

	vector<unsigned char> color_trajectory_second;
	vector<unsigned char> color_number_second;
	vector<unsigned char> color_arrow_second;
	color_trajectory_second.push_back(255);
	color_trajectory_second.push_back(0);
	color_trajectory_second.push_back(0);
	color_number_second.push_back(255);
	color_number_second.push_back(0);
	color_number_second.push_back(102);
	color_arrow_second.push_back(255);
	color_arrow_second.push_back(102);
	color_arrow_second.push_back(0);

	//draw trajectory
	for (int i = 0; i < i_frame + 1; i++)
	{
		//draw arraw
		pcl::PointXYZRGB point_pose_arraw;
		point_pose_arraw.x = trajectory_vec_vec[i](0, 0);
		point_pose_arraw.y = trajectory_vec_vec[i](1, 0);
		point_pose_arraw.z = trajectory_vec_vec[i](2, 0);
		if (M_s_trajectory_vec.size() == 0)
		{
			point_pose_arraw.r = color_arrow_first[0];
			point_pose_arraw.g = color_arrow_first[1];
			point_pose_arraw.b = color_arrow_first[2];
		}
		else if (M_s_trajectory_vec.size() == 1)
		{
			point_pose_arraw.r = color_arrow_second[0];
			point_pose_arraw.g = color_arrow_second[1];
			point_pose_arraw.b = color_arrow_second[2];
		}
		if(b_showArrow)
			drawArrow(point_pose_arraw, trajectory_vec_vec[i](3, 0), trajectory_vec_vec[i](4, 0), trajectory_vec_vec[i](5, 0),
				name_id + "_arrow" + to_string(i));

		//draw frame number
		pcl::PointXYZRGB point_frame;
		point_frame = point_pose_arraw;
		point_frame.z += -0.5;
		if (M_s_trajectory_vec.size() == 0)
		{
			point_frame.r = color_number_first[0];
			point_frame.g = color_number_first[1];
			point_frame.b = color_number_first[2];
		}
		else if (M_s_trajectory_vec.size() == 1)
		{
			point_frame.r = color_number_second[0];
			point_frame.g = color_number_second[1];
			point_frame.b = color_number_second[2];
		}
		if(b_showNumber) 
			drawNumber(point_frame, i, name_id + "_number" + to_string(i));

		if (i != 0)
		{
			//draw line
			pcl::PointXYZRGB point_pose_current;
			point_pose_current = point_pose_arraw;
			point_pose_current.r = 255;
			point_pose_current.g = 0;
			point_pose_current.b = 255;
			if (M_s_trajectory_vec.size() == 0)
			{
				point_pose_current.r = color_trajectory_first[0];
				point_pose_current.g = color_trajectory_first[1];
				point_pose_current.b = color_trajectory_first[2];
			}
			else if (M_s_trajectory_vec.size() == 1)
			{
				point_pose_current.r = color_trajectory_second[0];
				point_pose_current.g = color_trajectory_second[1];
				point_pose_current.b = color_trajectory_second[2];
			}
			pcl::PointXYZRGB point_pose_before;
			point_pose_before = point_pose_current;
			point_pose_before.x = trajectory_vec_vec[i - 1](0, 0);
			point_pose_before.y = trajectory_vec_vec[i - 1](1, 0);
			point_pose_before.z = trajectory_vec_vec[i - 1](2, 0);
			drawLine(point_pose_before, point_pose_current, name_id + "_line" + to_string(i));
		}
	}

	M_s_trajectory_vec.push_back(name_id);

	return;
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::deleteTrajectory(string s_name)
{
	string s_trajectory_delete;
	s_trajectory_delete = s_name;

	vector<string> s_line_vec_new, s_cylinder_vec_new, s_arrow_vec_new, s_number_vec_new, s_trajectory_vec_new;

	//delete all trajectory
	if (s_name.size() == 0)
	{
		s_trajectory_vec_new = M_s_trajectory_vec;
		for (int i = 0; i < s_trajectory_vec_new.size(); i++)
			deleteTrajectory(s_trajectory_vec_new[i]);
		return;
	}

	//remove trajectory name
	bool b_trajectory_erased = false;
	for (int i = 0; i < M_s_trajectory_vec.size(); i++)
	{
		vector<int> find_vec = CTimeString::find_all(M_s_trajectory_vec[i], s_trajectory_delete);
		if (find_vec.size() != 0)
		{
			M_s_trajectory_vec.erase(M_s_trajectory_vec.begin() + i);
			b_trajectory_erased = true;
			break;
		}
	}
	if (!b_trajectory_erased) throw std::runtime_error("ERROR: Trajectory not found.");

	//remove number name
	for (int i = 0; i < M_s_number_vec.size(); i++)
	{
		vector<int> find_vec = CTimeString::find_all(M_s_number_vec[i], s_trajectory_delete);
		if (find_vec.size() != 0) {}
	}
	M_s_number_vec.clear();
	M_s_number_vec = s_number_vec_new;

	//remove line
	for (int i = 0; i < M_s_line_vec.size(); i++)
	{
		vector<int> find_vec = CTimeString::find_all(M_s_line_vec[i], s_trajectory_delete);
		if (find_vec.size() != 0)
			M_viewer->removeShape(M_s_line_vec[i]);
		else s_line_vec_new.push_back(M_s_line_vec[i]);
	}
	M_s_line_vec.clear();
	M_s_line_vec = s_line_vec_new;

	//remove cylinder
	for (int i = 0; i < M_s_cylinder_vec.size(); i++)
	{
		vector<int> find_vec = CTimeString::find_all(M_s_cylinder_vec[i], s_trajectory_delete);
		if (find_vec.size() != 0)
			M_viewer->removeShape(M_s_cylinder_vec[i]);
		else s_cylinder_vec_new.push_back(M_s_cylinder_vec[i]);

	}
	M_s_cylinder_vec.clear();
	M_s_cylinder_vec = s_cylinder_vec_new;

	//remove arrow
	for (int i = 0; i < M_s_arrow_vec.size(); i++)
	{
		vector<int> find_vec = CTimeString::find_all(M_s_arrow_vec[i], s_trajectory_delete);
		if (find_vec.size() != 0)
			M_viewer->removeShape(M_s_arrow_vec[i]);
		else s_arrow_vec_new.push_back(M_s_arrow_vec[i]);
	}
	M_s_arrow_vec.clear();
	M_s_arrow_vec = s_arrow_vec_new;

	return;
}
