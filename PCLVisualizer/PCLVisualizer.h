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
	 vector<vector<string>> buffer;//�洢frames
private:
	Ui::PCLVisualizerClass ui;
	vector<string> frames;//�洢30֡
	vector<vector<float>> xyzs;//һ���ļ��е��������꣨���������е�����㣩
	typedef pcl::PointXYZ PointT;
	int n_pcd;
	int n_frame;

	//�������ݴ洢
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//��ʼ��vtk����
	void initialVtkWidget();
	//����ͷ
	QNetworkRequest request;
	QNetworkAccessManager manager;
	int bufferContorl(int n_pcd);
	vector<vector<float>> frame2xyz(string frame);
	vector<float> getXYZ(string point);//һ����

private slots:
	//�����򿪲�
	void play();
	void download();
	void replyFinished(QNetworkReply *reply);
};
#endif PCLVISUALIZER_H