#pragma once

#include <iostream>
#include <vector>
#include <random>

//detect plane
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/filters/extract_indices.h>

#include "PointcloudBasic.h"
#include "PointVisualization.h"

//should be under pcl includes
#include <windows.h>
#include "TimeString.h"

using namespace std;

class CPointcloudBasicProcess : public CPointcloudBasic
{

public:
	CPointcloudBasicProcess()
	{

	}

	void all_process();
	void FreeSpace();
	
	void FileProcess();
	void FileProcess_copy(string dir_from, string dir_to);
	void FileProcess_delete(string dir);
	void FileProcess_evacuate(string dir);
	void FileProcess_FolderInFolder(string dir_, vector<string> &folder_vec);

	void changeColor_plane(pcl::PointXYZRGB &point_);
	void changeColor_plane(pcl::PointXYZI &point_);

	template <class T_PointType>
	void detectPlane(pcl::PointCloud<T_PointType> &cloud_, double th_distance, bool b_cout = false, bool b_remove = false)
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

	void show_sequent();
	void DrawTrajectory();
	void DoMappingFromTrajectory();

};
