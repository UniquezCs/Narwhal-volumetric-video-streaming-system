#pragma   once
#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
#include <QtWidgets/QMainWindow>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/cloud_viewer.h>
#include "ui_PCLVisualizer.h"
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/vtk_io.h>
#include<pcl/io/vtk_lib_io.h>
#include<QtNetwork>
#include<vector>
#include <thread>
extern int pcd_number;
using namespace std;
const int FPS =30;
const int BUFFER_TIME = 4;
const int MAX_ = 4;

class PCLVisualizer : public QMainWindow
{
	Q_OBJECT

public:

	PCLVisualizer(QWidget *parent = 0);
	~PCLVisualizer();
	 vector<vector<string>> buffer;//存储frames
private:
	Ui::PCLVisualizerClass ui;
	vector<string> frames;//存储30帧
	vector<vector<float>> xyzs;//一个文件中的所有坐标（包含坐标中的坐标点）
	typedef pcl::PointXYZ PointT;
	int n_pcd;
	int n_frame;

	//点云数据存储
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//初始化vtk部件
	void initialVtkWidget();
	//请求头
	QNetworkRequest request;
	QNetworkAccessManager manager;
	int bufferContorl(int n_pcd);
	vector<vector<float>> frame2xyz(string frame);
	vector<float> getXYZ(string point);//一个点

private slots:
	//创建打开槽
	void play();
	void download();
	void replyFinished(QNetworkReply *reply);
};
#endif PCLVISUALIZER_H