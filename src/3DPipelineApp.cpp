#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#pragma warning( disable : 4996)
#endif

#include <string>
#include <boost/make_shared.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>

using namespace std;
using namespace pcl;

template <typename PointT>
class Module {
protected:
	boost::signals2::signal<void(const typename PointCloud<PointT>::ConstPtr)> pc_signal;

public:
	virtual void operator()(const typename PointCloud<PointT>::ConstPtr) {
	}

	void Connect(Module<PointT>& module) {
		pc_signal.connect(bind(&Module<PointT>::operator(), &module, _1));
	}
};

template <typename PointT>
class PCLoader : public Module<PointT> {
	typename PointCloud<PointT>::Ptr cloud;

	vector<string> file_names;
	int file_counter;
	bool running, quit;
	boost::thread thread;

	//return all file names matching the extension ext in a given directory dir
	static void FileNamebyExt(boost::filesystem::path const& dir, string const& ext, vector<string>& file_names) {
		boost::filesystem::directory_iterator pos(dir);
		boost::filesystem::directory_iterator end;

		for (; pos != end; ++pos) {
			if (boost::filesystem::is_regular_file(pos->status())) {
				if (boost::filesystem::extension(*pos) == ext) {
#if BOOST_FILESYSTEM_VERSION == 3
					file_names.push_back(pos->path().string());
#else
					file_names.push_back(pos->path());
#endif
				}
			}
		}
	}

public:

	PCLoader(const string& dir_name) {
		try {
			FileNamebyExt(dir_name, ".pcd", file_names);
		}
		catch (boost::filesystem::filesystem_error&) {
		}
		file_counter = 0;
		running = false;
		quit = false;
	}

	void Load(const string& file_name) {
		cloud = boost::make_shared< PointCloud<PointT> >();

		//load point cloud file
		if (io::loadPCDFile<PointT>(file_name, *cloud) == -1) {
			PCL_ERROR("Couldn't read file %s\n", file_name.c_str());
		}
		else {
			//remove NAN points from the cloud
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
			pc_signal(cloud);
		}
	}

	void Start() {
		running = true;
		thread = boost::thread(&PCLoader<PointT>::threadFunction, this);
	}

	void Stop() {
		quit = true;
		running = false;
	}

	bool isRunning() const {
		return running;
	}

	void threadFunction() {

		while (!quit && (file_counter < file_names.size())) {
			Load(file_names[file_counter]);
			file_counter++;
		}

		running = false;
	}

};

template <typename PointT>
class ZLogFilter : public Module<PointT> {

public:
	virtual void operator()(const typename PointCloud<PointT>::ConstPtr cloud) override {

		for (size_t i = 0; i < cloud->points.size(); i++) {
			cerr << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
		}
		pc_signal(cloud);
	}
};


template <typename PointT>
class BoxFilter : public Module<PointT> {

public:
	virtual void operator()(const typename PointCloud<PointT>::ConstPtr cloud) override {

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

		PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
		// Create the filtering object
		pcl::PassThrough<PointT> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.0, ground_distance);
		pass.filter(*cloud_filtered);
		pc_signal(cloud_filtered);
	}
};

template <typename PointT>
class PCViewer : public Module<PointT>{
	visualization::PCLVisualizer::Ptr visualizer;
	typename PointCloud<PointT>::ConstPtr cloud_;
	boost::mutex cloud_mutex;

public:
	PCViewer() {
		visualizer = boost::make_shared<visualization::PCLVisualizer>();
		visualizer->addCoordinateSystem(1.0);
		visualizer->setCameraPosition(0.0, 0.0, -5.0, 0.0, -1.0, 0.0);
	}

	virtual void operator()(const typename PointCloud<PointT>::ConstPtr cloud) override {
		boost::mutex::scoped_lock lock(cloud_mutex);
		cloud_ = cloud;
		lock.unlock();
	}
	
	bool SpinOnce() {
		if (!visualizer->wasStopped()) {
			boost::shared_ptr<const PointCloud<PointT> > cloud;

			if (cloud_mutex.try_lock()) {
				cloud_.swap(cloud);
				cloud_mutex.unlock();
			}

			if (cloud) {
				visualization::PointCloudColorHandlerRGBField<PointT> color_h(cloud);
				if (!visualizer->updatePointCloud<PointT>(cloud, color_h))
					visualizer->addPointCloud<PointT>(cloud, color_h);

				visualizer->spinOnce();
			}
			return true;
		}

		return false;
	}
};

int main(int argc, char **argv) {
	string file_name;

	//handle command line arguments
	for (int i = 1; i < argc; i++)
	{
		if ((strcmp(argv[i], "-f") == 0) && (i < (argc - 1))) {
			file_name = argv[++i];
		}
	}

	typedef PointXYZRGBA PointT;

	PCLoader<PointT> loader(file_name);

	//show point cloud
	PCViewer<PointT> viewer, viewer2;

	BoxFilter<PointT> filter;
	ZLogFilter<PointT> zlog;

	loader.Connect(filter);
	loader.Connect(zlog);

	loader.Connect(viewer);

	filter.Connect(viewer2);

	loader.Start();

	while (viewer.SpinOnce() && viewer2.SpinOnce() && loader.isRunning())
		;

	return 0;
}

