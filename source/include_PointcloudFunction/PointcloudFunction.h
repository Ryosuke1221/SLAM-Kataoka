#pragma once

#include<iostream>
#include <vector>

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

////https://akio-tanaka.tumblr.com/page/2
//#pragma comment(lib,"opengl32.lib")	
//#include <vtkAutoInit.h>
//VTK_MODULE_INIT(vtkRenderingOpenGL);
//VTK_MODULE_INIT(vtkInteractionStyle);

//should be under pcl includes
#include<windows.h>
#include "TimeString.h"


#define M_PI 3.14159265359


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
	void moveFile();
	void getPCDFromCSV_naraha();
	void getPCDFromCSV_gotFromPCAP(string dir_save, string dir_data, string file_RelativePath_);
	void FreeSpace();
	void filterNIRPointCloud_naraha();
	void getCSVFromPointCloud();
	void HandRegistration();
	void combinePointCloud_naraha();

	template <class T_PointType>
	void detectPlane(pcl::PointCloud<T_PointType> &cloud_, bool b_remove = false)
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
		//seg.setDistanceThreshold(0.1);
		//seg.setDistanceThreshold(0.05);	//velo
		seg.setDistanceThreshold(0.01);		//nir	
		seg.setInputCloud(cloud_.makeShared());
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
			PCL_ERROR("Could not estimate a planar model for the given dataset.");

		cout << "Model coefficients:";
		for (int i = 0; i < coefficients->values.size(); i++)
			cout << " " << coefficients->values[i];
		cout << " (pitch[deg]: " << -asin(coefficients->values[0]) * 180. / M_PI << ")";
		cout << endl;
		std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
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
	T_PointType M_point;

	boost::thread M_thread;

	bool M_b_showWindow;

	void startThread(T_PointCloudPtr cloud_arg);
	void doThread();

public:

	CPointVisualization();
	~CPointVisualization();

	void setWindowName(const string name_window_arg);

	void setPointCloud(T_PointCloudPtr cloud_arg);
	void updateViewer();
	void closeViewer();
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

//merge:6:47
//aiueo
//kakikukeko
