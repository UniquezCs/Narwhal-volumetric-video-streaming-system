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
#include<pcl/io/buffers.h>
#include<vector>
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

private:
	Ui::PCLVisualizerClass ui;
	//char p[160000]="0";

	vector<char> p;
	vector<vector<string>> buffer;
	vector<string> frames;
	vector<vector<float>> xyzs;//一个文件中的所有坐标（包含坐标中的坐标点）


	string s;
	int n_pcd;
	int n_frame;
	int n_buffer;
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
	vector<vector<float>> frames2xyz(string frame);
	vector<float> getXYZ(string point);//一个点

private slots:
	//创建打开槽
	void onOpen();
	void play();
	void download();
	void replyFinished(QNetworkReply *reply);


};
#endif PCLVISUALIZER_H