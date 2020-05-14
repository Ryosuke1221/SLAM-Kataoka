#include<iostream>
#include <vector>
#include<windows.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/console/parse.h>
//#include <pcl/filters/approximate_voxel_grid.h>

#include "TimeString.h"
#include "PointcloudFunction.h"

typedef pcl::PointXYZI PointType;

using namespace std;


int main()
{
	string foldername_;
	foldername_ = "../savedfolder/temp";

	cout << "HandRegistration started!" << endl;
	Sleep(1 * 1000);

	pcl::PointCloud<PointType>::Ptr cloud_show(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_show_static(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_moving(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_moving_before(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());

	CTimeString time_;

	Eigen::Affine3f Trans_;
	Eigen::Matrix4d HM_free = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d HM_Trans_now = Eigen::Matrix4d::Identity();

	double disp_translation = 0.;
	double disp_rotation = 0.;
	disp_translation = 0.05;
	disp_rotation = 0.5 * M_PI / 180.;

	int index_PC_now = 0;
	bool b_makeNewPC = true;
	bool b_first = true;
	bool b_escaped = false;
	bool b_break = false;

	enum KEYNUM {
		NONE,
		UP,
		DOWN,
		RIGHT,
		LEFT,
		TURN_R,
		TURN_L,
		ENTER,
		SUBTRACT
	};
	KEYNUM key_;

	vector<string> filenames_;
	time_.getFileNames_extension(foldername_, filenames_, ".pcd");

	std::string filename_txt;

	vector<Eigen::Matrix4d>	HM_displacement_vec;

	{
		vector<vector<double>> trajectory_vec_vec;

		//input txt
		vector<string> filenames__txt;
		time_.getFileNames_extension(foldername_, filenames__txt, ".txt");

		if (filenames__txt.size() == 0)
		{
			cout << "found no txt file and make it." << endl;

			//generation
			filename_txt = foldername_ + "/transformation.txt";
			for (int i = 0; i < filenames_.size(); i++)
			{
				vector<double> trajectory_vec;
				trajectory_vec.push_back(i);
				for (int j = 0; j < 6; j++) trajectory_vec.push_back(0.);
				trajectory_vec_vec.push_back(trajectory_vec);
			}

		}

		else if (filenames__txt.size() == 1)
		{
			trajectory_vec_vec = time_.getVecVecFromCSV<double>(foldername_ + "/" + filenames__txt[0]);
			filename_txt = foldername_ + "/" + filenames__txt[0];
		}

		else
		{
			cout << "error:too many txt files exist." << endl;
			return;
		}

		vector<Eigen::Matrix4d>	HM_trajectory_vec;
		for (int i = 0; i < trajectory_vec_vec.size(); i++)
		{
			Eigen::Matrix4d HM_trajectory = Eigen::Matrix4d::Identity();
			HM_trajectory = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(
				trajectory_vec_vec[i][1],
				trajectory_vec_vec[i][2],
				trajectory_vec_vec[i][3],
				trajectory_vec_vec[i][4],
				trajectory_vec_vec[i][5],
				trajectory_vec_vec[i][6]);
			HM_trajectory_vec.push_back(HM_trajectory);
		}

		for (int i = 0; i < HM_trajectory_vec.size(); i++)
		{
			if (i == 0)
			{
				HM_free = Eigen::Matrix4d::Identity();
				//HM_displacement_vec.push_back(HM_trajectory_vec[i] * HM_free.inverse());
				HM_displacement_vec.push_back(HM_free.inverse() * HM_trajectory_vec[i]);
			}
			else HM_displacement_vec.push_back(HM_trajectory_vec[i - 1].inverse() * HM_trajectory_vec[i]);
		}
		cout << "HM_displacement_vec size = " << HM_displacement_vec.size() << endl;
	}

	while (1) {
		//input next PointCloud
		if (b_makeNewPC) {

			cloud_moving_before->clear();

			string filename_PC;
			filename_PC = filenames_[index_PC_now];
			filename_PC = foldername_ + "/" + filename_PC;

			if (-1 == pcl::io::loadPCDFile(filename_PC, *cloud_moving_before)) break;

			//turn pitch(camera axis)
			{
				double pitch_init;
				pitch_init = 22. * M_PI / 180.;
				HM_free = Eigen::Matrix4d::Identity();
				HM_free = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., pitch_init, 0.);
				Trans_ = Eigen::Affine3f::Identity();
				Trans_ = CPointcloudFuction::calcAffine3fFromHomogeneousMatrix(HM_free);
				pcl::transformPointCloud(*cloud_moving_before, *cloud_moving_before, Trans_);
			}

			//ground
			bool b_RemoveGround = false;
			b_RemoveGround = true;
			if (b_RemoveGround)
			{
				double th_height;
				//th_height = -0.1;	//naraha summer
				th_height = -0.3;
				cloud_temp->clear();
				pcl::copyPointCloud(*cloud_moving_before, *cloud_temp);
				cloud_moving_before->clear();
				for (size_t i = 0; i < cloud_temp->size(); i++)
				{
					if (th_height > cloud_temp->points[i].z) continue;
					cloud_moving_before->push_back(cloud_temp->points[i]);
				}
			}


			cout << "PC(" << index_PC_now << ") number :" << cloud_moving_before->size() << endl;

			cout << endl;
			cout << "**********( key option )**********" << endl;
			cout << "Use numpad" << endl;
			cout << " TURN_LEFT:7    UP:8  TURN_RIGHT:9" << endl;
			cout << "      LEFT:4    ----       RIGHT:6" << endl;
			cout << "      ------  DOWN:2       -------" << endl;

			cout << endl;
			cout << "Reset:-(numpad)" << endl;

			cout << "Switch:ENTER" << endl;
			cout << "Escape:ESC" << endl;
			cout << "**********************************" << endl;
			cout << endl;

			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();

			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i <= index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_Trans_now = HM_free;
			Trans_ = CPointcloudFuction::calcAffine3fFromHomogeneousMatrix(HM_free);
			pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);

			b_makeNewPC = false;
		}

		//https://www.slideshare.net/masafuminoda/pcl-11030703
		//Viewer
		//左ドラッグ：視点の回転
		//Shift+左ドラッグ：視点の平行移動．
		//Ctrl+左ドラッグ：画面上の回転
		//右ドラッグ：ズーム
		//g：メジャーの表示
		//j：スクリーンショットの保存

		//input key
		short key_num_up = GetAsyncKeyState(VK_NUMPAD8);
		short key_num_down = GetAsyncKeyState(VK_NUMPAD2);
		short key_num_right = GetAsyncKeyState(VK_NUMPAD6);
		short key_num_left = GetAsyncKeyState(VK_NUMPAD4);
		short key_num_turn_r = GetAsyncKeyState(VK_NUMPAD9);
		short key_num_turn_l = GetAsyncKeyState(VK_NUMPAD7);
		short key_num_enter = GetAsyncKeyState(VK_RETURN);
		short key_num_escape = GetAsyncKeyState(VK_ESCAPE);
		short key_num_subt_numpad = GetAsyncKeyState(VK_SUBTRACT);

		if ((key_num_up & 1) == 1) key_ = UP;
		else if ((key_num_down & 1) == 1) key_ = DOWN;
		else if ((key_num_right & 1) == 1) key_ = RIGHT;
		else if ((key_num_left & 1) == 1) key_ = LEFT;
		else if ((key_num_turn_r & 1) == 1) key_ = TURN_R;
		else if ((key_num_turn_l & 1) == 1) key_ = TURN_L;
		else if ((key_num_enter & 1) == 1) key_ = ENTER;
		else if ((key_num_subt_numpad & 1) == 1) key_ = SUBTRACT;
		else if ((key_num_escape & 1) == 1)
		{
			cout << "ESC called" << endl;
			b_escaped = true;
			break;
		}
		else key_ = NONE;

		if (b_first)
		{
			key_ = NONE;
			b_first = false;
		}

		//if(key_ != NONE) cout << "key_ = " << key_ << endl;

		//determine transformation by key
		switch (key_) {
		case UP:
			HM_Trans_now = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(0., disp_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case DOWN:
			HM_Trans_now = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(0., -disp_translation, 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case RIGHT:
			HM_Trans_now = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(disp_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case LEFT:
			HM_Trans_now = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(-disp_translation, 0., 0., 0., 0., 0.)
				* HM_Trans_now;
			break;

		case TURN_R:
			HM_Trans_now = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., -disp_rotation)
				* HM_Trans_now;
			break;

		case TURN_L:
			HM_Trans_now = CPointcloudFuction::calcHomogeneousMatrixFromVector6d(0., 0., 0., 0., 0., disp_rotation)
				* HM_Trans_now;
			break;

		case ENTER:
			*cloud_show_static += *cloud_moving;
			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i < index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_displacement_vec[index_PC_now] = HM_free.inverse() * HM_Trans_now;
			index_PC_now++;
			if (index_PC_now == HM_displacement_vec.size()) b_break = true;
			b_makeNewPC = true;
			cout << "ENTER pressed" << endl;
			break;

		case SUBTRACT:
			HM_free = Eigen::Matrix4d::Identity();
			for (int i = 0; i <= index_PC_now; i++) HM_free = HM_free * HM_displacement_vec[i];
			HM_Trans_now = HM_free;
			cout << "-(numpad) pressed" << endl;
			break;

		default:
			break;
		}

		if (!(key_ == NONE || key_ == ENTER)) {
			cloud_moving->clear();
			Trans_ = Eigen::Affine3f::Identity();
			Trans_ = CPointcloudFuction::calcAffine3fFromHomogeneousMatrix(HM_Trans_now);
			pcl::transformPointCloud(*cloud_moving_before, *cloud_moving, Trans_);
		}
		cloud_show->clear();
		*cloud_show += *cloud_show_static;
		*cloud_show += *cloud_moving;

		if (cloud_show->size() != 0) {
			ShowPcdFile(cloud_show);
		}

		if (b_break) break;
	}

	//output txt
	if (!b_escaped)
	{
		vector<vector<double>> trajectory_vec_vec;
		HM_free = Eigen::Matrix4d::Identity();
		for (int i = 0; i < HM_displacement_vec.size(); i++)
		{
			HM_free = HM_free * HM_displacement_vec[i];
			Eigen::Vector6d State_;
			State_ = CPointcloudFuction::calcVector6dFromHomogeneousMatrix(HM_free);
			vector<double> trajectory_vec;
			trajectory_vec.push_back(i);
			trajectory_vec.push_back(State_[0]);
			trajectory_vec.push_back(State_[1]);
			trajectory_vec.push_back(State_[2]);
			trajectory_vec.push_back(State_[3]);
			trajectory_vec.push_back(State_[4]);
			trajectory_vec.push_back(State_[5]);
			trajectory_vec_vec.push_back(trajectory_vec);
		}
		time_.getCSVFromVecVec(trajectory_vec_vec, filename_txt);
		cout << "file has saved!" << endl;
	}
	else cout << "file has not saved!" << endl;

	m_viewer->close();

	return 0;
}