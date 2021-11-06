#pragma once

#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "PointcloudBasicProcess.h"
#include "FPFH_PCL.h"

using namespace std;

class CPointcloudGeneration : public CPointcloudBasicProcess
{
public:
	void mainProcess();
	void FreeSpace();
private:

	void PCDGeneration_fromCSV(string dir_);	//debug

	void PCDGeneration_fromPCD(string dir_);	//debug

	void NarahaWinter202001(string dir_);
	void NarahaWinter202001_PCDGeneration(string dir_);
	void NarahaWinter202001_Filtering(string dir_);
	void NarahaWinter202001_Combination(string dir_);

	void getCSVFromPointCloud();

	template <class T_PointType>
	static vector<pcl::PointIndices> getSegmentation_indices(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg, float Tolerance, int MinClusterSize)
	{
		//https://vml.sakura.ne.jp/koeda/PCL/tutorials/html/cluster_extraction.html#cluster-extraction
		//https://www.slideshare.net/masafuminoda/pcl-11030703
		//std::cout << "PointCloud before filtering has: " << cloud_arg->points.size() << " data points." << std::endl;
		// Creating the KdTree object for the search method of the extraction
		boost::shared_ptr<pcl::search::KdTree<T_PointType>> tree(new pcl::search::KdTree<T_PointType>);
		tree->setInputCloud(cloud_arg);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<T_PointType> ec;
		ec.setClusterTolerance(Tolerance);//threshold; distance of clusters 
		ec.setMinClusterSize(MinClusterSize);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud_arg);
		ec.extract(cluster_indices);
		return cluster_indices;
	}

	template <class T_PointType>
	static vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> getSegmentation_rest(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest, float Tolerance, int MinClusterSize)
	{
		std::vector<pcl::PointIndices> cluster_indices;
		cluster_indices = getSegmentation_indices(cloud_arg, Tolerance, MinClusterSize);

		vector<bool> b_rest_vec;
		for (int i = 0; i < cloud_arg->size(); i++)
			b_rest_vec.push_back(true);
		for (int j = 0; j < cluster_indices.size(); j++)
		{
			for (int i = 0; i < cluster_indices[j].indices.size(); i++)
				b_rest_vec[cluster_indices[j].indices[i]] = false;
		}

		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec;
		for (int j = 0; j < cluster_indices.size(); j++)
		{
			pcl::PointCloud<T_PointType>::Ptr cloud_cluster(new pcl::PointCloud<T_PointType>);
			for (int i = 0; i < cluster_indices[j].indices.size(); i++)
				cloud_cluster->push_back(cloud_arg->points[cluster_indices[j].indices[i]]);
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
			//std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
			cloud_cluster_vec.push_back(cloud_cluster);
		}

		cloud_rest->clear();
		for (int i = 0; i < cloud_arg->size(); i++)
		{
			if (b_rest_vec[i] == true)
			{
				auto point = cloud_arg->points[i];
				cloud_rest->push_back(point);
			}
		}

		return cloud_cluster_vec;
	}

	template <class T_PointType>
	static vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> getSegmentation_robust(
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest, float Tolerance, int MinClusterSize)
	{

		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec;
		cloud_rest->clear();

		cloud_cluster_vec = getSegmentation_rest(cloud_arg, cloud_rest, Tolerance, MinClusterSize);

		////next clustering (to rest)
		//{
		//	vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec_next;
		//	boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest_next(new pcl::PointCloud<T_PointType>);
		//	cloud_cluster_vec_next = getSegmentation_rest(cloud_rest, cloud_rest_next, Tolerance * 1.5, MinClusterSize);
		//	for (int i = 0; i < cloud_cluster_vec_next.size(); i++)
		//		cloud_cluster_vec.push_back(cloud_cluster_vec_next[i]);
		//	cloud_rest->clear();
		//	pcl::copyPointCloud(*cloud_rest_next, *cloud_rest);
		//}

		//next clustering (to cluster)
		{
			vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec_next;
			for (int j = 0; j < cloud_cluster_vec.size(); j++)
			{
				vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec_in1cluster;
				boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_rest_in1cluster(new pcl::PointCloud<T_PointType>);

				cloud_cluster_vec_in1cluster = getSegmentation_rest(cloud_cluster_vec[j], cloud_rest_in1cluster, Tolerance * 0.5, (float)MinClusterSize * 0.1);
				for (int i = 0; i < cloud_cluster_vec_in1cluster.size(); i++)
					cloud_cluster_vec_next.push_back(cloud_cluster_vec_in1cluster[i]);
				*cloud_rest += *cloud_rest_in1cluster;
			}
			cloud_cluster_vec.clear();
			cloud_cluster_vec = cloud_cluster_vec_next;
		}

		return cloud_cluster_vec;
	}

	template <class T_PointType>
	static void rejectOutlier(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> &cloud_output, float Tolerance, int MinClusterSize)
	{
		vector<boost::shared_ptr<pcl::PointCloud<T_PointType>>> cloud_cluster_vec;
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_temp(new pcl::PointCloud<T_PointType>);
		cloud_cluster_vec = getSegmentation_robust(cloud_arg, cloud_temp, Tolerance, MinClusterSize);
		//cloud_cluster_vec = getSegmentation_rest(cloud_arg, cloud_temp, Tolerance, MinClusterSize);

		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_output_temp(new pcl::PointCloud<T_PointType>);
		for (int i = 0; i < cloud_cluster_vec.size(); i++)
			*cloud_output_temp += *cloud_cluster_vec[i];
		cloud_output->clear();
		pcl::copyPointCloud(*cloud_output_temp, *cloud_output);
	}

	template <class T_PointType>
	static void remove_outliers(boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_arg,
		boost::shared_ptr<pcl::PointCloud<T_PointType>> &cloud_output, int MeanK, float StddevMulThresh)
	{
		//http://virtuemarket-lab.blogspot.com/2015/03/outlier.html
		boost::shared_ptr<pcl::PointCloud<T_PointType>> cloud_filtered(new pcl::PointCloud<T_PointType>);
		pcl::StatisticalOutlierRemoval<T_PointType> sor;
		sor.setInputCloud(cloud_arg);
		sor.setMeanK(MeanK);
		sor.setStddevMulThresh(StddevMulThresh);
		sor.setNegative(false);
		sor.filter(*cloud_filtered);
		cloud_output->clear();
		pcl::copyPointCloud(*cloud_filtered, *cloud_output);
	}

	void DoSegmentation();

	void DoOutlierRejector_ground(string dir_);
	void DoOutlierRejector_clustering(string dir_);
	void DoOutlierRejector(string dir_);

	void ThermalCloudGeneration(string dir_);

};
