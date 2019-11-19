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
#include<qthread.h>
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
	 //vector<vector<string>> buffer;//存储frame

	int bufferContorl(int n_pcd);
	//vector<vector<float>> frame2xyz(string frame);
	//vector<float> getXYZ(string point);//一个点
	void initialVtkWidget();	//初始化vtk部件
private:
	//点云数据存储
	int Frame_number = 300;

	//请求头
	QNetworkRequest request;
	QNetworkAccessManager manager;

public slots:
	//创建打开槽
	void play();
	void download();
	// 响应结束，进行结果处理
	void replyFinished(QNetworkReply *reply);


};

class play_thread : public QThread{
	Q_OBJECT
public:
	play_thread(QObject *parent = 0);

private:

	void run();

};


#endif PCLVISUALIZER_H