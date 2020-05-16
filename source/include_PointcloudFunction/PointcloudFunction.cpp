#include "PointcloudFunction.h"

Eigen::Matrix4d CPointcloudFuction::calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
	double Roll_, double Pitch_, double Yaw_) {
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

Eigen::Affine3f CPointcloudFuction::calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat) {
	Eigen::Affine3f Trans_Affine = Eigen::Affine3f::Identity();
	Eigen::Vector6d Trans_Vec = Eigen::Vector6d::Identity();
	Trans_Vec = calcVector6dFromHomogeneousMatrix(input_Mat);
	Trans_Affine.translation() << Trans_Vec(0, 0), Trans_Vec(1, 0), Trans_Vec(2, 0);
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(5, 0), Eigen::Vector3f::UnitZ()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(4, 0), Eigen::Vector3f::UnitY()));
	Trans_Affine.rotate(Eigen::AngleAxisf(Trans_Vec(3, 0), Eigen::Vector3f::UnitX()));
	return Trans_Affine;
}

Eigen::Vector6d CPointcloudFuction::calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d transformation_Node) {
	Eigen::Vector6d XYZRPY = Eigen::Vector6d::Zero();
	double X_, Y_, Z_, Roll_, Pitch_, Yaw_;
	X_ = transformation_Node(0, 3);
	Y_ = transformation_Node(1, 3);
	Z_ = transformation_Node(2, 3);
	if (transformation_Node(2, 0) == -1.) {
		Pitch_ = M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(transformation_Node(1, 2), transformation_Node(1, 1));
	}
	else if (transformation_Node(2, 0) == 1.) {
		Pitch_ = -M_PI / 2.0;
		Roll_ = 0.;
		Yaw_ = atan2(-transformation_Node(1, 2), transformation_Node(1, 1));
	}
	else {
		Yaw_ = atan2(transformation_Node(1, 0), transformation_Node(0, 0));
		Roll_ = atan2(transformation_Node(2, 1), transformation_Node(2, 2));
		double cos_Pitch;
		if (cos(Yaw_) == 0.) {
			cos_Pitch = transformation_Node(0, 0) / sin(Yaw_);
		}
		else 	cos_Pitch = transformation_Node(0, 0) / cos(Yaw_);

		Pitch_ = atan2(-transformation_Node(2, 0), cos_Pitch);
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
CPointVisualization<T_PointType>::CPointVisualization()
{
	//// PCL Visualizer
	//m_viewer.reset(new pcl::visualization::PCLVisualizer("Velodyne Viewer"));
	//m_viewer->addCoordinateSystem(3.0, "coordinate");
	//m_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	//m_viewer->initCameraParameters();
	//m_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
	setViewer("Velodyne Viewer");


	//m_viewer->removeCoordinateSystem("coordinate");		//remove axis in viewer

	// Point Cloud Color Handler
	//if (type == typeid(pcl::PointXYZ)) {
	//	std::vector<double> color = { 255.0, 255.0, 255.0 };
	//	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(color[0], color[1], color[2]));
	//	m_handler = color_handler;
	//}
	//else if (type == typeid(pcl::PointXYZI)) {
	//	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> color_handler(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
	//	m_handler = color_handler;
	//}
	//else if (type == typeid(pcl::PointXYZRGBA)) {
	//	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA>());
	//	m_handler = color_handler;
	//}
	//else if (type == typeid(pcl::PointXYZRGB)) {
	//	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>());
	//	m_handler = color_handler;
	//}
	//else {
	//	throw std::runtime_error("This PointType is unsupported.");
	//}
	// Point Cloud Color Handler
	if (T_PointType == typeid(pcl::PointXYZ)) {
		std::vector<double> color = { 255.0, 255.0, 255.0 };
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(color[0], color[1], color[2]));
		m_handler = color_handler;
	}
	else if (T_PointType == typeid(pcl::PointXYZI)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> color_handler(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
		m_handler = color_handler;
	}
	else if (T_PointType == typeid(pcl::PointXYZRGBA)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA>());
		m_handler = color_handler;
	}
	else if (T_PointType == typeid(pcl::PointXYZRGB)) {
		boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>> color_handler(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>());
		m_handler = color_handler;
	}
	else {
		throw std::runtime_error("This PointType is unsupported.");
	}
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::setViewer(const string name_window_arg)
{
	m_viewer.reset(new pcl::visualization::PCLVisualizer(name_window_arg));
	m_viewer->addCoordinateSystem(3.0, "coordinate");
	m_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	m_viewer->initCameraParameters();
	m_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);

}

