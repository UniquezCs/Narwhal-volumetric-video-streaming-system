#include "PCLVisualizer.h"
#include <QFileDialog>
#include <iostream>
#include <vtkRenderWindow.h>
#include <Qstring.h>
#include <iomanip>
#include <string>


using namespace std;

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
	int n_pcd = 0;
	int n_buffer = FPS * BUFFER_TIME;
	
}
//读取缓存中的点云数据并渲染播放

void PCLVisualizer::play() {
	std::string pcd_name;
	int video_time = 10;
	int i = 0;
	viewer->addCoordinateSystem(1.0);
	while (i < video_time) {
		
		for (int j = 0; j < 30; j++) {
			xyzs = frame2xyz(buffer.at(i).at(j));//一帧点云转化为坐标
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
			cloud2->clear();
			xyzs.clear();
		}
		i++;
		buffer.pop_back();
	}
}

//向服务器下载请求
void PCLVisualizer::download() {
	int pcd_number = 300;
	QString stuff = ".pcd";
	QString stuff2 = ".ply";
	connect(&manager, SIGNAL(finished(QNetworkReply *)), this, SLOT(replyFinished(QNetworkReply *)));
	for (int i = 0; i < pcd_number; i++) {
		QString n_server = QString::fromStdString(to_string(i));
		//QString n_server = QString::fromStdString(to_string(i+1000));
		request.setUrl(QUrl("http://localhost/pcd_test/test_pcd" + n_server + stuff));
		//request.setUrl(QUrl("http://114.213.214.160/loot/loot_vox10_" + n_server + stuff2));
		manager.get(request);
	}
	n_frame = 0;
	buffer.clear();
}

// 响应结束，进行结果处理
void PCLVisualizer::replyFinished(QNetworkReply *reply)
{
	QString stuff = ".pcd";
	if (reply->error() == QNetworkReply::NoError)
	{
		QString n_client = QString::fromStdString(to_string(n_pcd++));
		QByteArray bytes = reply->readAll();
		string frame = bytes.mid(180, bytes.size());
		//string frame = bytes.mid(300, bytes.size());%xyzrgb点云
		frames.push_back(frame);
		n_frame++;
		if (n_frame >= 30) {
			n_frame = 0;//重置
			buffer.push_back(frames);
			frames.clear();
			if (buffer.size() == 2) {
				play();
			}
		}
		reply->deleteLater();
	}
	else
	{
		//no reply
	}
}

int PCLVisualizer::bufferContorl(int n_pcd) {
	return 1;
}

vector<vector<float>> PCLVisualizer::frame2xyz(string frame) {
	size_t size = frame.size();
	size_t n_point = count(frame.begin(), frame.end(), '\r');//点个数
	size_t offset = 0;
	string point;
	for (size_t i=0; i < n_point; i++) {
		
		point = frame.substr(offset, frame.find('\r', offset)-offset);
		offset = frame.find('\r', offset+2)+2;
		xyzs.push_back(getXYZ(point));
		point.clear();
	}
	size_t pos = frame.find("/n", 0);

	return xyzs;
}

vector<float> PCLVisualizer::getXYZ(string point) {
	int pos_block = 0;
	vector<float> x_y_z;
	for (int i = 0; i < 3; i++) {
		float xyz = stof(point.substr(pos_block, point.find(' ', pos_block)-pos_block));
		pos_block = point.find(' ', pos_block)+1;
		x_y_z.push_back(xyz);


	}
	return x_y_z;

}