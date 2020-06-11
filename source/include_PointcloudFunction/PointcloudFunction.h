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
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/ModelCoefficients.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

////https://akio-tanaka.tumblr.com/page/2
//#pragma comment(lib,"opengl32.lib")	
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

#include "../include_ICP/KataokaPCL.h"

//should be under pcl includes
#include<windows.h>
#include "TimeString.h"


#define M_PI 3.14159265359
#define D2R 0.017453288888889
#define R2D 57.29579143313326


using namespace std;
//typedef pcl::PointXYZI PointType;
//typedef pcl::PointXYZRGB PointType;

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

class CPointcloudFuction
{

public:
	CPointcloudFuction()
	{

	}

	static Eigen::Matrix4d calcHomogeneousMatrixFromVector6d(double X_, double Y_, double Z_,
		double Roll_, double Pitch_, double Yaw_);

	static Eigen::Affine3f calcAffine3fFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

	static Eigen::Vector6d calcVector6dFromHomogeneousMatrix(Eigen::Matrix4d input_Mat);

	void all_process();
	void show_sequent();
	void getPCDFromCSV_naraha();
	void getPCDFromCSV_gotFromPCAP(string dir_save, string dir_data, string file_RelativePath_);
	void FreeSpace();
	void filterNIRPointCloud_naraha();
	void getCSVFromPointCloud();
	void HandRegistration();
	void combinePointCloud_naraha();

	template <class T_PointType>
	void detectPlane(pcl::PointCloud<T_PointType> &cloud_,double th_distance, bool b_cout = false, bool b_remove = false)
	{
		//https://qiita.com/akachochin/items/47f1470565e76adb1880
		//https://www.slideshare.net/masafuminoda/pcl-11030703
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		// Create the segmentation object  
		pcl::SACSegmentation<T_PointType> seg;
		// Optional  
		seg.setOptimizeCoefficients(true);
		// Mandatory  
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(200);
		seg.setDistanceThreshold(th_distance);	
		seg.setInputCloud(cloud_.makeShared());
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
			PCL_ERROR("Could not estimate a planar model for the given dataset.");

		if (b_cout)
		{
			cout << "Model coefficients:";
			for (int i = 0; i < coefficients->values.size(); i++)
				cout << " " << coefficients->values[i];
			cout << " (pitch[deg]: " << -asin(coefficients->values[0]) * 180. / M_PI << ")";
			cout << endl;
			std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
		}
		for (size_t i = 0; i < inliers->indices.size(); ++i)
			changeColor_plane(cloud_.points[inliers->indices[i]]);

		if (b_remove)
		{
			pcl::ExtractIndices<T_PointType> extract;
			extract.setInputCloud(cloud_.makeShared());
			extract.setIndices(inliers);
			extract.setNegative(true); //true: removing plane, false: removing except plane
			extract.filter(cloud_);
		}
	}

	void changeColor_plane(pcl::PointXYZRGB &point_);
	void changeColor_plane(pcl::PointXYZI &point_);

	void DynamicTranslation();

	void FileProcess();
	void FileProcess_copy(string dir_from, string dir_to);
	void FileProcess_delete(string dir);
	void FileProcess_evacuate(string dir);

private:
	enum KEYNUM {
		NONE,
		ZERO,
		X_,
		Y_,
		Z_,
		ROLL_,
		PITCH_,
		YAW_,
		X_MINUS,
		Y_MINUS,
		Z_MINUS,
		ROLL_MINUS,
		PITCH_MINUS,
		YAW_MINUS,
		ENTER,
		RSHIFT,
		RCTRL,
		ESC
	};
public:
	//should declare under enum type declaration
	KEYNUM getKEYNUM();
	void DrawTrajectory();
	void DoSegmentation();
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, double th_tolerance);
	vector<pcl::PointIndices> getSegmentation_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, double th_tolerance);
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getSegmentation_rest(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rest, double th_tolerance);

};

template <class T_PointType>
class CPointVisualization
{
	//https://stackoverflow.com/questions/40544174/class-templating-to-process-pclpointcloud-objects-with-different-point-types
	typedef typename pcl::visualization::PointCloudColorHandler<T_PointType> T_ColorHandler;
	typedef typename pcl::PointCloud<T_PointType>::Ptr T_PointCloudPtr;
	typedef typename pcl::visualization::PointCloudColorHandlerCustom<T_PointType> T_ColorHandlerCustom;				//XYZ
	typedef typename pcl::visualization::PointCloudColorHandlerGenericField<T_PointType> T_ColorHandlerGenericField;	//XYZI
	typedef typename pcl::visualization::PointCloudColorHandlerRGBField<T_PointType> T_ColorHandlerRGBField;			//XYZRGB

	boost::shared_ptr<pcl::visualization::PCLVisualizer> M_viewer;
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
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawNumber_OnetoNine(
		pcl::PointXYZRGB point_center, int num_arg, double length_horizontal_arg);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr expandDiagram(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_arg, float times_);

	int num_NumberPointcloud;
public:

	CPointVisualization();
	~CPointVisualization();

	void setWindowName(const string name_window_arg);

	void setPointCloud(T_PointCloudPtr cloud_arg);
	void updateViewer();
	void closeViewer();
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawLine(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr proliferation_rand(pcl::PointXYZRGB point_arg, int times);
	static  bool isPointOutOnLineSegmant(
		pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_, pcl::PointXYZRGB point3_);
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawArrow(
		pcl::PointXYZRGB point_arg, double Roll_, double Pitch_, double Yaw);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawNumber(
		pcl::PointXYZRGB point_center, int num_arg);
	static vector<std::uint8_t> getRGBwithValuebyHSV(double value_, double value_max, double value_min);
	void useNormal(double radius_arg, int leven_arg, float scale_arg);

};

//int main()
//{
//	string foldername_;
//	foldername_ = "../../data/temp";
//	string s_filename_PC;
//	s_filename_PC = foldername_ + "/000.pcd";
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZI>());
//	pcl::io::loadPCDFile(s_filename_PC, *cloud_1);
//	pcl::io::loadPCDFile(s_filename_PC, *cloud_2);
//	CPointVisualization<pcl::PointXYZ> pv1;
//	CPointVisualization<pcl::PointXYZI> pv2;
//	pv1.setWindowName("show XYZI");
//	pv2.setWindowName("show XYZI2");
//	while (1)
//	{
//		pv1.setPointCloud(cloud_1);
//		pv1.updateViewer();
//		pv2.setPointCloud(cloud_2);
//		pv2.updateViewer();
//	}
//	cout << "finish" << endl;
//	return 0;
//}

//https://qiita.com/i153/items/38f9688a9c80b2cb7da7
template < typename T_PointType >
CPointVisualization<T_PointType>::CPointVisualization()
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

	M_cloud_ = (new pcl::PointCloud<T_PointType>)->makeShared();
	M_cloud_normal = (new pcl::PointCloud<pcl::Normal>)->makeShared();

	M_b_useNormal = false;


	num_NumberPointcloud = 0;
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
	M_viewer->addCoordinateSystem(3.0, "coordinate");
	M_viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
	M_viewer->initCameraParameters();
	M_viewer->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
}

template < typename T_PointType >
void CPointVisualization<T_PointType>::setPointCloud(T_PointCloudPtr cloud_arg)
{
	//mutex with scoped_lock
	boost::mutex::scoped_lock lock(M_mutex);
	//update pointcloud
	M_cloud_->clear();
	pcl::copyPointCloud(*cloud_arg, *M_cloud_);

	if (!M_b_useNormal || M_cloud_->size() == 0) return;
	//http://virtuemarket-lab.blogspot.com/2015/02/blog-post_35.html
	pcl::NormalEstimation<T_PointType, pcl::Normal> ne;
	ne.setInputCloud(M_cloud_);
	pcl::search::KdTree<T_PointType>::Ptr tree(new pcl::search::KdTree<T_PointType>());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(M_radius_nor);
	ne.compute(*M_cloud_normal);
	cout << "M_cloud_normal->size():" << M_cloud_normal->size() << endl;
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
		if (lock.owns_lock() && M_cloud_)
		{
			//cout << "locked";
			//cout << "(WindowName:" << M_name_window << ")" << endl;
			M_handler->setInputCloud(M_cloud_);
			//update viewer
			if (!M_viewer->updatePointCloud(M_cloud_, *M_handler, "cloud"))
				M_viewer->addPointCloud(M_cloud_, *M_handler, "cloud");
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
	pcl::copyPointCloud(*cloud_arg,*M_cloud_);
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
		cout << "elapsed:" << time_.getTimeElapsefrom2Strings_millisec(t1,t2) << endl;
	}
}

template < typename T_PointType >
CPointVisualization<T_PointType>::~CPointVisualization()
{
	if(M_thread.joinable()) M_thread.join();
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawLine(pcl::PointXYZRGB point1_, pcl::PointXYZRGB point2_)
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawArrow(
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

	*cloud_ += *drawLine(point_Root, point_BottomCenter);
	*cloud_ += *drawLine(point_BottomLeft, point_BottomRight);
	*cloud_ += *drawLine(point_BottomRight, point_Top);
	*cloud_ += *drawLine(point_BottomLeft, point_Top);

	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::copyPointCloud(*cloud_, *cloud_temp);
		cloud_->clear();
		for (int i = 0; i < cloud_temp->size(); i++)
			*cloud_ += *proliferation_rand(cloud_temp->points[i], 2);
	}
	//cloud_ = expandDiagram(cloud_, 5.);

	Eigen::Affine3f Trans_ = Eigen::Affine3f::Identity();
	Trans_ = CPointcloudFuction::calcAffine3fFromHomogeneousMatrix(
		CPointcloudFuction::calcHomogeneousMatrixFromVector6d(
			point_arg.x, point_arg.y, point_arg.z, Roll_, Pitch_, Yaw_));
	pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

	return cloud_;
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawNumber(
	pcl::PointXYZRGB point_center, int num_arg)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZRGB>());
	double length_horizontal_ = 0.5;
	int index_ = 0;
	double range_ = 0.8;
	double height_gap = 0.1;
	point_center.z += height_gap * num_NumberPointcloud;
	while (1)
	{
		*cloud_ += *drawNumber_OnetoNine(point_center, num_arg % 10, length_horizontal_);
		num_arg /= 10;
		if (num_arg == 0) break;
		point_center.x += -range_;
		index_++;
	}

	num_NumberPointcloud++;

	return cloud_;
}

template < typename T_PointType >
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CPointVisualization<T_PointType>::drawNumber_OnetoNine(
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
		*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		//*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 1:
		//*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		//*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		//*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 2:
		*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		//*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 3:
		*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 4:
		//*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		//*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 5:
		*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		//*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 6:
		*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		//*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 7:
		//*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		//*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		//*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 8:
		*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	case 9:
		*cloud_ += *drawLine(point_LeftDown, point_RightDown);
		*cloud_ += *drawLine(point_LeftCenter, point_RightCenter);
		*cloud_ += *drawLine(point_LeftUp, point_RightUp);
		//*cloud_ += *drawLine(point_LeftDown, point_LeftCenter);
		*cloud_ += *drawLine(point_LeftCenter, point_LeftUp);
		*cloud_ += *drawLine(point_RightDown, point_RightCenter);
		*cloud_ += *drawLine(point_RightCenter, point_RightUp);
		break;
	}

	Eigen::Affine3f Trans_ = Eigen::Affine3f::Identity();
	Trans_ = CPointcloudFuction::calcAffine3fFromHomogeneousMatrix(
		CPointcloudFuction::calcHomogeneousMatrixFromVector6d(
			-length_horizontal / 2. + point_center.x, -length_half_vertical + point_center.y,
			point_center.z, 0., 0., 0.));
	pcl::transformPointCloud(*cloud_, *cloud_, Trans_);

	return cloud_;
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
void CPointVisualization<T_PointType>::useNormal(double radius_arg, int leven_arg, float scale_arg)
{
	M_radius_nor = radius_arg;
	M_level_nor_vis = leven_arg;
	M_scale_nor_vis = scale_arg;
	M_b_useNormal = true;
}
