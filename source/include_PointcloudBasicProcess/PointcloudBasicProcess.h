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
	
	void FileProcess(string dir_);
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

	template<typename T_PointType>
	void show_sequent_template(string dir_, vector<string> filenames_)
	{
		bool b_useTXT = false;
		//b_useTXT = true;

		bool b_plane = false;
		//b_plane = true;

		bool b_select = false;
		//b_select = true;

		bool b_normal = false;
		//b_normal = true;

		CPointVisualization<T_PointType> pv;
		if (typeid(T_PointType) == typeid(pcl::PointXYZI))
			pv.setWindowName("show XYZI");
		else if (typeid(T_PointType) == typeid(pcl::PointXYZRGB))
			pv.setWindowName("show XYZRGB");
		else
			throw std::runtime_error("This PointType is unsupported.");

		//show normal
		if (b_normal)
		{
			cout << "input: normal radius" << endl;
			cout << "->";
			float radius_normal;
			cin >> radius_normal;
			pv.useNormal(radius_normal, 10, 0.1);
		}

		if (filenames_.size() == 0)
		{
			cout << "no pointcloud found" << endl;
			pv.closeViewer();
			return;
		}

		//ignore some files
		if (b_select)
		{
			bool b_showAll = true;
			cout << "select: Do you watch all .pcd or not?  Yes:1  No:0" << endl;
			cout << "->";
			cin >> b_showAll;
			if (!b_showAll)
			{
				for (int i = 0; i < filenames_.size(); i++)
				{
					string s_i = to_string(i);
					if (s_i.size() < 3) s_i = " " + s_i;
					if (s_i.size() < 3) s_i = " " + s_i;
					cout << "i:" << s_i << " " << filenames_[i] << endl;

				}
				cout << endl;
				cout << "select(index): files you watch and separeted with spaces" << endl;
				cout << "->";
				vector<string> s_input_vec;
				s_input_vec = CTimeString::inputSomeString();
				vector<string> filenames_temp;
				filenames_temp = filenames_;
				filenames_.clear();
				for (int i = 0; i < s_input_vec.size(); i++)
				{
					int index_temp = stoi(s_input_vec[i]);
					if (!(0 <= index_temp && index_temp < filenames_temp.size()))
					{
						cout << "ERROR: ignored invalud:" << index_temp << endl;
						continue;
					}
					filenames_.push_back(filenames_temp[index_temp]);
					cout << "filenames_.back():" << filenames_.back() << endl;
				}

				if (filenames_.size() == 0) filenames_ = filenames_temp;
			}

		}

		if (filenames_.size() == 0)
		{
			cout << "ERROR: no file found" << endl;
			return;
		}

		pcl::PointCloud<T_PointType>::Ptr cloud_(new pcl::PointCloud<T_PointType>());

		int index_ = 0;

		vector<string> filename_use;
		cout << "Press space to next" << endl;
		while (1)
		{
			//short key_num = GetAsyncKeyState(VK_SPACE);
			if ((GetAsyncKeyState(VK_SPACE) & 1) == 1)
			{
				if (index_ == filenames_.size())
				{
					cout << "index over" << endl;
					break;
				}
				cout << "index_: " << index_ << endl;
				//cout << "reading:" << filenames_[index_] << endl;
				pcl::io::loadPCDFile(dir_ + "/" + filenames_[index_], *cloud_);
				cout << "showing:" << filenames_[index_] << " size:" << cloud_->size() << endl;
				//pv.setPointCloud(cloud_);
				pv.setPointCloud(cloud_, filenames_[index_]);

				//remove ground plane
				if (cloud_->size() != 0 && b_plane)
				{
					detectPlane<T_PointType>(*cloud_, 0.05, true, true);	//velo
					//detectPlane<T_PointType>(*cloud_, 0.01, true, true);	//nir
				}
				index_++;
			}
			////save
			//if ((GetAsyncKeyState(VK_RETURN) & 1) == 1 && b_useTXT)
			//{
			//	filename_use.push_back(filenames_[index_ - 1]);
			//	cout << "add: " << filenames_[index_ - 1] << endl;
			//}
			//escape
			//short key_num_esc = GetAsyncKeyState(VK_ESCAPE);
			if ((GetAsyncKeyState(VK_ESCAPE) & 1) == 1) {
				cout << "toggled!" << endl;
				break;
			}
			pv.updateViewer();
		}

		//for (int i = 0; i < filename_use.size(); i++)
		//{
		//	//cout << "file " << i << ": " << filename_use[i] << endl;
		//	cout << filename_use[i] << endl;
		//}

		//vector<vector<string>> save_vec_vec;
		//for (int i = 0; i < filename_use.size(); i++)
		//{
		//	vector<string> save_vec;
		//	save_vec.push_back(filename_use[i]);
		//	save_vec_vec.push_back(save_vec);
		//}
		//if (b_useTXT)
		//	CTimeString::getCSVFromVecVec(save_vec_vec, dir_ + "/_usePointCloud.csv");

		pv.closeViewer();

	}
	void show_sequent_PointTypes(string dir_);

	void DrawTrajectory(string dir_);
	void DoMappingFromTrajectory(string dir_);

};
