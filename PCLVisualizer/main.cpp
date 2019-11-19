#include "PCLVisualizer.h"
#include <QtWidgets/QApplication>
#include <malloc.h>
#include <k4a/k4a.hpp>
#include <iostream>
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include "k4a_grabber.h"
using namespace std;
using namespace boost;
using namespace pcl;
typedef pcl::PointXYZRGBA PointType;

int generate_vpc()
{
	int i = 0;
	int n_ply = 1000;
	const uint32_t deviceCount = k4a::device::get_installed_count();//读取kinect数目
	if (deviceCount == 0)
	{
		//cout << "no azure kinect devices detected!" << endl;
		return 0;
	}
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));

	// Point Cloud
	pcl::PointCloud<PointType>::ConstPtr cloud;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;

	//回调函数是个lambda函数,cloud和mutex以引用(变量)的形式捕捉,传值(常量)
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex,&n_ply](const pcl::PointCloud<PointType>::ConstPtr& ptr)
	{
		boost::mutex::scoped_lock lock(mutex);

		/* Point Cloud Processing */

		cloud = ptr->makeShared();
		Sleep(33);
		pcl::io::savePLYFile("test"+to_string(n_ply++)+".ply", *cloud);

	};

	// KinectAzureDKGrabber
	boost::shared_ptr<pcl::Grabber> grabber =
		boost::make_shared<pcl::KinectAzureDKGrabber>(0, K4A_DEPTH_MODE_NFOV_2X2BINNED, K4A_IMAGE_FORMAT_COLOR_BGRA32, K4A_COLOR_RESOLUTION_720P);

	boost::shared_ptr<pcl::KinectAzureDKGrabber> grabber_ = boost::dynamic_pointer_cast<pcl::KinectAzureDKGrabber>(grabber);

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);//回调函数来处理点云

	// Start Grabber
	grabber->start();


	k4a::calibration calibration = grabber_->getCalibration();
	k4a_calibration_intrinsic_parameters_t *intrinsics = &calibration.color_camera_calibration.intrinsics.parameters;
	Eigen::Matrix3f intrinsics_eigen;
	intrinsics_eigen <<
		intrinsics->param.fx, 0.0f, intrinsics->param.cx,
		0.0f, intrinsics->param.fy, intrinsics->param.cy,
		0.0f, 0.0f, 1.0f;
	Eigen::Matrix4f extrinsics_eigen = Eigen::Matrix4f::Identity();
	viewer->setCameraParameters(intrinsics_eigen, extrinsics_eigen);

	while (!viewer->wasStopped())
	{
		// Update Viewer
		viewer->spinOnce();
		boost::mutex::scoped_lock lock(mutex);
		if (lock.owns_lock() && cloud)
		{
			// Update Point Cloud
			if (!viewer->updatePointCloud(cloud, "cloud"))
			{
				viewer->addPointCloud(cloud, "cloud");
			}
		}
	}

	// Stop Grabber

	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected())
	{
		connection.disconnect();
	}
	return 0;
}

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	PCLVisualizer w;
	w.show();
	return a.exec();

	//generate_vpc();

}

