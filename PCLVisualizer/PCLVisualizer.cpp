#include "PCLVisualizer.h"
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
#define INITIAL_BUFFER_SIZE 60;
#define MAX_BUFFER_SIZE 150;
#define MIN_BUFFER_SIZE 30;

using namespace std;
mutex mtx;
QString stuff = ".pcd";
QString stuff2 = ".ply";
int n_pcd = 0;
int buffer_size;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
Ui::PCLVisualizerClass ui;
vector<vector<float>> xyzs;//一帧的所有坐标（包含坐标中的坐标点）
queue<string> buffer;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
typedef pcl::PointXYZ PointT;
//play_thread playthread_;

vector<float> getXYZ(string point) {
	int pos_block = 0;
	vector<float> x_y_z;
	for (int i = 0; i < 3; i++) {
		float xyz = stof(point.substr(pos_block, point.find(' ', pos_block) - pos_block));
		pos_block = point.find(' ', pos_block) + 1;
		x_y_z.push_back(xyz);

	}
	return x_y_z;
}

vector<vector<float>> frame2xyz(string frame) {
	size_t size = frame.size();
	size_t n_point = count(frame.begin(), frame.end(), '\r');//点个数
	size_t offset = 0;
	string point;
	for (size_t i = 0; i < n_point; i++) {

		point = frame.substr(offset, frame.find('\r', offset) - offset);
		offset = frame.find('\r', offset + 2) + 2;
		xyzs.push_back(getXYZ(point));
		point.clear();
	}
	size_t pos = frame.find("/n", 0);
	return xyzs;
}

void play_thread::run() {
	int video_fraems = 300;
	int i = 0;
	viewer->addCoordinateSystem(1.0);
	while (true) {
		if (buffer.size() > 30) {
			while (i < video_fraems) {
				xyzs = frame2xyz(buffer.front());//一帧点云转化为坐标
				for (int k = 0; k < xyzs.size(); k++) {
					PointT p;
					p.x = xyzs.at(k).at(0);
					p.y = xyzs.at(k).at(1);
					p.z = xyzs.at(k).at(2);
					cloud2->points.push_back(p);//添加点云
				}
				//mymutex.lock();
				viewer->updatePointCloud(cloud2, "cloud2");
				qApp->processEvents();//刷新,代替spin()
				viewer->resetCamera();//重置相机
				ui.qvtkWidget->update();
				cloud2->clear();
				xyzs.clear();
				i++;
				buffer.pop();
				//buffer.erase(buffer.begin());		
			}

		}
		else continue;	
	}
}

PCLVisualizer::PCLVisualizer(QWidget *parent)
{
	ui.setupUi(this);
	//初始化
	initialVtkWidget();

	//连接信号和槽
	connect(ui.button_play, SIGNAL(clicked()), this, SLOT(download()));
}

PCLVisualizer::~PCLVisualizer()
{
	delete &ui;
}

//按钮入口(向服务器下载请求)
void PCLVisualizer::download() {
	//下载完成后的槽
	connect(&manager, SIGNAL(finished(QNetworkReply *)), this, SLOT(replyFinished(QNetworkReply *)));
	//playthread_.start();

	QtConcurrent::run([]() {
		int i = 0;
		viewer->addCoordinateSystem(1.0);
		while (true) {
			if (buffer.size() > 10) {	
					Sleep(33);
					xyzs = frame2xyz(buffer.front());//一帧点云转化为坐标
					for (int k = 0; k < xyzs.size(); k++) {
						PointT p;
						p.x = xyzs.at(k).at(0);
						p.y = xyzs.at(k).at(1);
						p.z = xyzs.at(k).at(2);
						cloud2->points.push_back(p);//添加点云
					}
					viewer->updatePointCloud(cloud2, "cloud2");
					qApp->processEvents();//刷新,代替spin()
					viewer->resetCamera();//重置相机
					ui.qvtkWidget->update();
					ui.label->setText(QString::fromStdString(to_string(buffer.size())));
					
					cloud2->clear();
					xyzs.clear();

					buffer.pop();
					//buffer.erase(buffer.begin());		
				
			}
			else continue;
		}
	});
	for (int i = 0; i < Frame_number; i++) {
		std::cout << i << std::endl;
		QString n_server = QString::fromStdString(to_string(i));
		//QString n_server = QString::fromStdString(to_string(i+1000));
		request.setUrl(QUrl("http://localhost/pcd_test/test_pcd" + n_server + stuff));
		//request.setUrl(QUrl("http://114.213.214.160/loot/loot_vox10_" + n_server + stuff2));
		manager.get(request);
	}
}

void PCLVisualizer::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	cloud2.reset(new pcl::PointCloud<pcl::PointXYZ>);
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud, "cloud");
	viewer->addPointCloud(cloud2, "cloud2");
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	ui.qvtkWidget->update();
}

//下载的数据写入buffer
void PCLVisualizer::replyFinished(QNetworkReply *reply)
{
	if (reply->error() == QNetworkReply::NoError)
	{
		boost::mutex::scoped_lock(mymutex);
		QString n_client = QString::fromStdString(to_string(n_pcd++));
		QByteArray bytes = reply->readAll();
		string frame = bytes.mid(180, bytes.size());
		//string frame = bytes.mid(300, bytes.size());%xyzrgb点云
		buffer.push(frame);
		frame.clear();
		reply->deleteLater();
	}
	else
	{
		//no reply
	}
}


//读取缓存中的点云数据并渲染播放
void PCLVisualizer::play() {
	string pcd_name;
	int video_fraems = 300;
	int i = 0;
	viewer->addCoordinateSystem(1.0);
	while (true) {
		if (buffer.size() > 30) {
			while (i < video_fraems) {
				xyzs = frame2xyz(buffer.front());//一帧点云转化为坐标
				for (int k = 0; k < xyzs.size(); k++) {
					PointT p;
					p.x = xyzs.at(k).at(0);
					p.y = xyzs.at(k).at(1);
					p.z = xyzs.at(k).at(2);
					cloud2->points.push_back(p);//添加点云
				}
				//mymutex.lock();
				viewer->updatePointCloud(cloud2, "cloud2");
				qApp->processEvents();//刷新,代替spin()
				viewer->resetCamera();//重置相机
				ui.qvtkWidget->update();
				cloud2->clear();
				xyzs.clear();
				i++;
				buffer.pop();
				//buffer.erase(buffer.begin());
			}
		}
		else
		continue;
			
	}

}

