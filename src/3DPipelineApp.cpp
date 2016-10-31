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
#include <pcl/common/transforms.h>
#include "Module.h"
#include "Filters.h"

using namespace std;
using namespace pcl;

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

	PCLoader(const string& path_name) {
        file_counter = 0;
        running = false;
        quit = false;

        boost::filesystem::path path(path_name);
        
        if (!boost::filesystem::exists(path))
            cerr << "No valid file name given!" << endl;
        else if (boost::filesystem::is_directory(path))
            FileNamebyExt(path_name, ".pcd", file_names);
        else
            file_names.push_back(path_name);
 	}

	virtual void operator()(const typename PointCloud<PointT>::Ptr) {
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
            /*
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            
            // The same rotation matrix as before; theta radians arround Z axis
            transform.rotate (Eigen::AngleAxisf (0.15, Eigen::Vector3f::UnitX()));

            pcl::transformPointCloud (*cloud, *cloud, transform);
			*/

			this->pc_signal(cloud);
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
class PCViewer : public Module<PointT>{
	visualization::PCLVisualizer::Ptr visualizer;
	typename PointCloud<PointT>::Ptr cloud_;
	boost::mutex cloud_mutex;

public:
	PCViewer() {
		visualizer = boost::make_shared<visualization::PCLVisualizer>();
		visualizer->addCoordinateSystem(1.0);
		visualizer->setCameraPosition(0.0, 0.0, -5.0, 0.0, -1.0, 0.0);
	}

	virtual void operator()(const typename PointCloud<PointT>::Ptr cloud) {
		boost::mutex::scoped_lock lock(cloud_mutex);
		cloud_ = cloud;
		lock.unlock();
	}
	
	bool SpinOnce() {
		if (!visualizer->wasStopped()) {
			boost::shared_ptr< PointCloud<PointT> > cloud;
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
	PCViewer<PointT> viewer;

	BoxFilter<PointT> filter;
	ZLogFilter<PointT> zlog;
	PoseCorrection<PointT> pose_corrector;

//	loader.Connect(filter);
//	loader.Connect(zlog);

	loader.Connect(pose_corrector);

	loader.Connect(viewer);

//	filter.Connect(viewer2);

	loader.Start();

	while (viewer.SpinOnce() && loader.isRunning())
		;

	return 0;
}

