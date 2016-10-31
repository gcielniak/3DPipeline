#pragma once
#include "Module.h"
#include <iostream>
#include <pcl/registration/icp.h>

using namespace std;
using namespace pcl;

template <typename PointT>
class ZLogFilter : public Module<PointT> {

public:
	virtual void operator()(const typename PointCloud<PointT>::Ptr cloud) {

		for (size_t i = 0; i < cloud->points.size(); i++) {
			cerr << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
		}
		this->pc_signal(cloud);
	}
};


template <typename PointT>
class BoxFilter : public Module<PointT> {

public:
	virtual void operator()(const typename PointCloud<PointT>::Ptr cloud) {

		//estimate distance to ground
		std::vector<float> z_values(cloud->points.size());

		for (size_t i = 0; i < cloud->points.size(); i++)
			z_values[i] = cloud->points[i].z;

		sort(z_values.begin(), z_values.end());

		float ground_distance = nanf("");
		if (z_values.size()) {
			int ground_index = (int)(z_values.size() * 99.0 / 100.0);
			ground_distance = z_values[ground_index] - 0.25;
		}

		//		cerr << "Ground distance: " << ground_distance << endl;

		typename PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
		// Create the filtering object
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, ground_distance);
		pass.filter(*cloud_filtered);
		this->pc_signal(cloud_filtered);
	}
};

//Correct the camera pose so it is parallel to the ground
template <typename PointT>
class PoseCorrection : public Module<PointT> {
	typename PointCloud<PointT>::Ptr cloud_prev;

public:

	virtual void operator()(const typename PointCloud<PointT>::Ptr cloud) {
		if (cloud_prev != nullptr) {
			IterativeClosestPoint<PointT, PointT> icp;
			icp.setInputCloud(cloud);
			icp.setInputTarget(cloud_prev);
			PointCloud<PointT> cloud_reg;
			icp.align(cloud_reg);
//			std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
			Eigen::Matrix4f transformation = icp.getFinalTransformation();	
			std::cout << transformation(0, 3) << " " << transformation(1, 3) << " " << transformation(2, 3) << " " << std::endl;
		}

		cloud_prev = cloud;
	}
};

//Estimate the ground surface and remove from the cloud
template <typename PointT>
class GroundRemoval : public Module<PointT> {
};

template <typename PointT>
class OutlierStatsFilter : public Module<PointT> {
public:
	int num_NN;

	OutlierStatsFilter() :
		num_NN(1) {
	}
		
	virtual void operator()(const typename PointCloud<PointT>::Ptr cloud) {
		std::vector<int> mapping;
		StatisticalOutlierRemoval<PointT> outlier;
		mapping.clear();
		outlier.setInputCloud(cloud);
		outlier.setMeanK(num_NN);//parameter
		outlier.setStddevMulThresh(1.0);//parameter
		outlier.filter(mapping);

		typename PointCloud<PointT>::Ptr cloud_processed(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud<PointT>(*cloud, mapping, *cloud_processed);
		this->pc_signal(cloud_processed);
	}
};

template <typename PointT>
class OutlierRadiusFilter : public Module<PointT> {
	double searchRadius;
	int min_NN;

public:
	OutlierRadiusFilter() :
		min_NN(100), searchRadius(1.0) {
	}

	virtual void operator()(const typename PointCloud<PointT>::Ptr cloud) {
		vector<int> indices;
		pcl::RadiusOutlierRemoval<PointType> radius_filter;
		radius_filter.setInputCloud(input_cloud);
		radius_filter.setRadiusSearch(searchRadius);
		radius_filter.setMinNeighborsInRadius(min_NN);
		radius_filter.filter(indices);

		typename PointCloud<PointT>::Ptr cloud_processed(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud<PointT>(*cloud, indices, *cloud_processed);
		this->pc_signal(cloud_processed);
	}
};

template <typename PointT>
class PassthroughFilter : public Module<PointT> {
public:
	double z_min, z_max;

	PassthroughFilter() :
		z_min(0.0), z_max(double::INFINITY) {
	}

	virtual void operator()(const typename PointCloud<PointT>::Ptr cloud) {
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		int sz = cloud->points.size();
		double depth[sz];
		float sum = 0; float dev; float mean;
		for (int i = 0; i<sz; i++) {
			depth[i] = cloud->points[i].z;
			sum = sum + depth[i];
		}
		std::sort(depth, depth + sz);
		const int N = sizeof(depth) / sizeof(int);
		double max_depth = 0.12*(*max_element(depth, depth + sz));
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z_min, z_max);

		typename PointCloud<PointT>::Ptr cloud_processed(new pcl::PointCloud<PointT>);
		pass.filter(*cloud_processed);
		this->pc_signal(cloud_processed);
	}

};
