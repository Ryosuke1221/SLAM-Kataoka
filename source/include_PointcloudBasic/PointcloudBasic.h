#pragma once

#include <iostream>
#include <vector>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

//detect plane
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

//#include "PointVisualization.h"

//should be under pcl includes
#include <windows.h>
#include "TimeString.h"

#define M_PI 3.14159265359
#define D2R 0.017453288888889
#define R2D 57.29579143313326

using namespace std;

namespace Eigen {

	/// Extending Eigen namespace by adding frequently used matrix type
	typedef Eigen::Matrix<double, 6, 6> Matrix6d;
	typedef Eigen::Matrix<double, 6, 1> Vector6d;

}    // namespace Eigen

class CPointcloudBasic
{

public:
	CPointcloudBasic()
	{

	}

};
