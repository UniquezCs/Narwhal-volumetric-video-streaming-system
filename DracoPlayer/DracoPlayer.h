#pragma once
#include <QtWidgets/QMainWindow>
#include "ui_DracoPlayer.h"
#include <QtWidgets/QMainWindow>
#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/cloud_viewer.h>
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
#include <cinttypes>
#include "draco/compression/decode.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/file_utils.h"
#include "draco/io/obj_encoder.h"
#include "draco/io/parser_utils.h"
#include "draco/io/ply_encoder.h"


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
#include <tbb/tbb.h>

#include <QCloseEvent>
#include <boost/lockfree/queue.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
extern int pcd_number;
using namespace std;
using namespace pcc;
class DracoPlayer : public QMainWindow
{
	Q_OBJECT

public:

	DracoPlayer(QWidget *parent = 0);
	~DracoPlayer();
	void initialVtkWidget();	//��ʼ��vtk����
	void decode_thread(int is_tile);
	void readbuffer_thread();
	int writebuffer_thread(int is_tile);
	Ui::DracoPlayerClass* getui() { return &ui; }
	queue<map<int, PCCPointSet3>> buffer;
	std::unique_ptr<draco::PointCloud> pc;
private:
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	QNetworkRequest request;//����ͷ
	QNetworkAccessManager manager;//tile
	QNetworkAccessManager manager2;
	Ui::DracoPlayerClass ui;
	QThreadPool threadpool;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

	//boost::shared_ptr<pcl::visualization::DracoPlayer> viewer(new pcl::visualization::DracoPlayer("NarWhal"));

	string tile_compressed;
	string tile_uncompressed;


signals://����ֵ��void,ֻ��Ҫ����,����Ҫʵ�֣���Ϊֻ�Ǹ��źţ������ж�������
	void sigrender();
	void siggsend();
	void siggsend2();
	void sig_ply2fusion(string ply, PCCPointSet3 frame, int gof_id, int frame_id);
	void sig_ply2fusion_add(string ply, PCCPointSet3 &frame, int gof_id, int frame_id);

public slots://�߰汾qt,�ۺ�������д��public����ȫ�ֺ���,��Ҫ������ʵ�֣���Ϊ��Ҫ���巴Ӧ����
	//�����򿪲�
	void download();
	void download2();
	void send();//����������
	void send2();//
	// ��Ӧ���������н������
	void replyFinished(QNetworkReply *reply);
	void replyFinished2(QNetworkReply *reply);
	int frame2xyzrgb(string ply, int gof_id, int frame_id, int is_tile);
	int frame2xyzrgb_add(string ply, PCCPointSet3 *frame, int gof_id, int frame_id);
	int frame2xyzrgb2(string ply, int gof_id, int frame_id);

};




