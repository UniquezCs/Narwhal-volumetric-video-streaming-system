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
#include <QSemaphore>
#include <QFileDialog>
#include <iostream>
#include <vtkRenderWindow.h>
#include <Qstring.h>
#include <iomanip>
#include <string>
#include <Windows.h>
#include <thread>
#include <QtConcurrent>
#include <queue>
#include "PCCCommon.h"
#include "PCCChrono.h"
#include "PCCMemory.h"
#include "PCCDecoder.h"
#include "PCCMetrics.h"
#include "PCCChecksum.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCBitstream.h"
#include "PCCGroupOfFrames.h"
#include "PCCDecoderParameters.h"
#include "PCCMetricsParameters.h"
#include <cinttypes>
#include <program_options_lite.h>
#include <tbb/tbb.h>
#include <QCloseEvent>
#include <boost/lockfree/queue.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
extern int pcd_number;
using namespace std;
using namespace pcc;

class PCLVisualizer : public QMainWindow
{
	Q_OBJECT
public:

	PCLVisualizer(QWidget *parent = 0);
	~PCLVisualizer();
	void initialVtkWidget();	//初始化vtk部件
	void decode_thread(int is_tile);
	void readbuffer_thread();
	int writebuffer_thread(int is_tile);
	Ui::PCLVisualizerClass* getui() { return &ui; }
	queue<map<int, PCCPointSet3>> buffer;

private:
	
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	QNetworkRequest request;//请求头
	QNetworkAccessManager manager;//tile
	QNetworkAccessManager manager2;
	Ui::PCLVisualizerClass ui;
	QThreadPool threadpool;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("NarWhal"));

	string tile_compressed;
	string tile_uncompressed;


signals://返回值是void,只需要声明,不需要实现，因为只是个信号，不具有动作意义
	void sigrender();
	void siggsend();
	void siggsend2();
	void sig_ply2fusion(string ply, PCCPointSet3 frame, int gof_id,int frame_id);
	void sig_ply2fusion_add(string ply, PCCPointSet3 &frame, int gof_id, int frame_id);

public slots://高版本qt,槽函数可以写到public或者全局函数,需要声明和实现，因为需要具体反应动作
	//创建打开槽
	void download();
	void download2();
	void send();//后续的请求
	void send2();//
	// 响应结束，进行结果处理
	void replyFinished(QNetworkReply *reply);
	void replyFinished2(QNetworkReply *reply);
	int frame2xyzrgb(string ply, int gof_id,int frame_id,int is_tile);
	int frame2xyzrgb_add(string ply, PCCPointSet3 *frame, int gof_id, int frame_id);
	int frame2xyzrgb2(string ply, int gof_id, int frame_id);

};



#endif PCLVISUALIZER_H