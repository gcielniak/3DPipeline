#pragma once

#include <boost/signals2/signal.hpp>
#include <boost/bind.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

template <typename PointT>
class Module {
protected:
	boost::signals2::signal<void(const typename PointCloud<PointT>::Ptr)> pc_signal;
	typename PointCloud<PointT>::Ptr cloud_processed;

public:
	virtual void operator()(const typename PointCloud<PointT>::Ptr) = 0;

	void Connect(Module<PointT>& module) {
		pc_signal.connect(bind(&Module<PointT>::operator(), &module, _1));
	}
};

