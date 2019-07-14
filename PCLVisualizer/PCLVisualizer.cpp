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
	//��ʼ��
	initialVtkWidget();
	//�����źźͲ�
	//connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui.button_play, SIGNAL(clicked()), this, SLOT(pla7y()));
	connect(ui.button_download, SIGNAL(clicked()), this, SLOT(download()));
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
	int n_frame = 0;
}
//��ȡ�ı��ͺͶ������͵�������
void PCLVisualizer::onOpen()
{
	//ֻ�ܴ�PCD�ļ�
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open PointCloud"), "."
	);
	if (!fileName.isEmpty())
	{
		std::string file_name = fileName.toStdString();
		pcl::PCLPointCloud2::Ptr cloud2;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		int pcd_version;
		int data_type;
		unsigned int data_idx;
		int offset = 0;
		pcl::PCDReader rd;
		std::string addr = "D:/Holographic/dataset/loot/loot/Ply/";
		std::string pcd_name;
		int pcd_number = 1300;
		int i = 1000;
		viewer->addCoordinateSystem(1.0);
		while (i < pcd_number) {
			viewer->resetCamera();
			pcd_name = addr + "loot_vox10_" + to_string(i++) + ".ply";
			//rd.readHeader(pcd_name, *cloud2, origin, orientation, pcd_version, data_type, data_idx);
			pcl::io::loadPLYFile(pcd_name, *cloud);
			viewer->updatePointCloud(cloud, "cloud");
			qApp->processEvents();
			ui.qvtkWidget->update();

		}
	}
}

void PCLVisualizer::play() {
	std::string pcd_name;
	int pcd_number = 300;
	int i = 0;
	viewer->addCoordinateSystem(1.0);
	while (i < pcd_number) {
		viewer->resetCamera();
		for (int j = 0; j < 30; j++) {
			for (int k = 0; k < 5000; k++) {
				xyzs = frames2xyz(buffer.at(i).at(j));
				

				}
				
				

			
			

		}
		

		viewer->updatePointCloud(cloud2, "cloud2");
		qApp->processEvents();//����spin(),����ԭ�����
		ui.qvtkWidget->update();
		i++;
	}
}

void PCLVisualizer::download() {
	int n = 0;
	int pcd_number = 300;
	QString stuff = ".pcd";
	connect(&manager, SIGNAL(finished(QNetworkReply *)), this, SLOT(replyFinished(QNetworkReply *)));
	for (int i = 0; i < pcd_number; i++) {
		QString n_server = QString::fromStdString(to_string(i));
		request.setUrl(QUrl("http://114.213.214.160/pcd_test/test_pcd" + n_server + stuff));
		manager.get(request);
	}
	//play();
}

// ��Ӧ���������н������
void PCLVisualizer::replyFinished(QNetworkReply *reply)
{
	QString stuff = ".pcd";
	if (reply->error() == QNetworkReply::NoError)
	{
		QString n_client = QString::fromStdString(to_string(n_pcd++));
		QByteArray bytes = reply->readAll();
		// ���浽����

		for (size_t i = 180; i < bytes.size(); i++) {
			if (i >= bytes.size() || i < 0) { cout << i; break; }//����±��Ƿ����
			char by = bytes.at(i);
			p.push_back(by);
		}
 		string frame(p.begin(),p.end());
		p.clear();
		frames.push_back(frame);
		n_frame++;
		if (n_frame == 30) {
			n_frame = 0;//����
			buffer.push_back(frames);
			frames.clear();
		}


	}

	else
	{
		cout<< "erros";
	}
}


int PCLVisualizer::bufferContorl(int n_pcd) {




	return 1;
}

vector<vector<float>> PCLVisualizer::frames2xyz(string frame) {
	size_t size = frame.size();
	size_t n_point = count(frame.begin(), frame.end(), '\r');//�����
	size_t offset = 0;
	string point;
	for (size_t i=0; i <= n_point; i++) {
		
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
		float xyz = stod(point.substr(pos_block, point.find(' ', pos_block)-pos_block));
		pos_block = point.find(' ', pos_block)+1;
		x_y_z.push_back(xyz);


	}
	return x_y_z;



}

