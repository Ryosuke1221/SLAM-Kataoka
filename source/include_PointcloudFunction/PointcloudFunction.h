#pragma once

#include<iostream>
#include <vector>
#include <random>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_plotter.h>

#include "PointcloudBasicProcess.h"
#include "ExtendableICP.h"
#include "FPFH_PCL.h"

//should be under pcl includes
#include<windows.h>
#include "TimeString.h"

using namespace std;

class CPointcloudFunction : public CPointcloudBasicProcess
{

public:
	CPointcloudFunction()
	{

	}

	void all_process();
	void FreeSpace();

	void DynamicTranslation(string dir_);

};
