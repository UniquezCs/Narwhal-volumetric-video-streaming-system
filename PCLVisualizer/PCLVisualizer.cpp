#include "PCLVisualizer.h"
#include <qlist.h>
#include <render_thread.h>
#include <algorithm>
#ifdef TIXML_USE_STL
#include <iostream>
#include <sstream>
using namespace std;
#else
#include <stdio.h>
#endif
#if defined( WIN32 ) && defined( TUNE )
#include <crtdbg.h>
_CrtMemState startMemState;
_CrtMemState endMemState;
#endif
#include "tinyxml.h"
#include "tinyxml.h"
#include <string>
#include <map>

using namespace pcc;
using namespace std;

QString stuff = ".pcd";
QString stuff_ply = ".ply";
int Frame_number = 300;
int GOF_number = 10;
int n_pcd = 0;
int buffer_size;
QMutex mymutex(QMutex::Recursive);

vector<vector<float>> xyzs;//һ֡���������꣨���������е�����㣩
queue<string> buffer;
typedef pcl::PointXYZRGB PointT;
double start;
double cost;
vector<double> speed;
vector<vector<int>> x;
int a[12][5];
int e[12] = {1,0,1,0,0,1,0,0,0,0,0,0};
enum SuccessEnum { FAILURE, SUCCESS };
map <string, string> mpd;
map<int, PCCPointSet3> fusions[15];//�洢5�� frame_id->

int gof_tile[1] = {1};//��֤�Ƿ��ں���� gof_id->tilenumber
map<QString, vector<uint8_t>> bin_store;//(tile_id,gof_id��->bin




//����������

template <typename T>
static std::istream& readUInt(std::istream& in, T& val) {
	unsigned int tmp;
	in >> tmp;
	val = T(tmp);
	return in;
}

namespace pcc {
	static std::istream& operator>>(std::istream& in, PCCColorTransform& val) { return readUInt(in, val); }
}  

SuccessEnum loadXML(const char *filename)
{
	// XML�ĵ�
	TiXmlDocument doc;
	string frame_n;
	string gof_n;
	string tile_n;
	string baseurl;

	// ����XML�ĵ�
	if (!doc.LoadFile(filename))
	{
		//cerr << doc.ErrorDesc() << endl;
		return FAILURE;
	}

	// ������ڵ��������ֵΪ�ĵ��ĵ�һ�����ڵ�
	TiXmlElement *root = doc.FirstChildElement();
	// ���û���ҵ����ڵ�,˵���ǿ�XML�ĵ����߷�XML�ĵ�
	if (root == NULL)
	{
		//cerr << "Failed to load file: No root element." << endl;
		// �����ڴ�
		doc.Clear();
		return FAILURE;
	}
	else
	{
		frame_n = root->Attribute("frames");
		gof_n = root->Attribute("GOF");
		tile_n = root->Attribute("Tiles");
		baseurl = root->Attribute("BaseURL");
		mpd.insert(pair<string, string>("frame_n", frame_n));
		mpd.insert(pair<string, string>("gof_n", gof_n));
		mpd.insert(pair<string, string>("tile_n", tile_n));
	}

	// �����ӽڵ�
	for (TiXmlElement *elem = root->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
	{
		// ��ȡԪ����
		string elemName = elem->Value();
		string gof_id;

			gof_id = elem->Attribute("id");
			//�ж�adp����
			for (TiXmlElement *AdaptationSet = elem->FirstChildElement(); AdaptationSet != NULL; AdaptationSet = AdaptationSet->NextSiblingElement()) {
				string adp_id = AdaptationSet->Attribute("id");
				// δѹ���ĵ�����Ϣ
				if (adp_id._Equal("1")) {
					for (TiXmlElement *Rep = AdaptationSet->FirstChildElement(); Rep != NULL; Rep = Rep->NextSiblingElement()) {
						string r_level = Rep->Attribute("id");
						for (TiXmlElement *frame = Rep->FirstChildElement(); frame != NULL; frame = frame->NextSiblingElement()) {
							string frame_id = frame->Attribute("id");
							string frame_pointn = frame->Attribute("PointN");
							mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level + "frame" + frame_id + "pointn", frame_pointn));
							string frame_size = frame->Attribute("size");
							mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level + "frame" + frame_id + "size", frame_size));
							for (TiXmlElement *tile = frame->FirstChildElement(); tile != NULL; tile = tile->NextSiblingElement()) {
								string tile_id = tile->Attribute("id");
								string tile_pointn = tile->Attribute("PointN");
								mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level + "frame" + frame_id + "tile" + tile_id + "pointn", tile_pointn));
								string tile_size = tile->Attribute("size");
								mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level + "frame" + frame_id + "tile" + tile_id + "size", tile_size));
								TiXmlNode *tile_url = tile->FirstChild();
								string tile_urls = baseurl+tile_url->ToText()->Value();
								mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level + "frame" + frame_id + "tile" + tile_id + "url", tile_urls));
							}
						}
					}
				}

				//ѹ����ĵ�����Ϣ
				else {
					for (TiXmlElement *tile = AdaptationSet->FirstChildElement(); tile != NULL; tile = tile->NextSiblingElement()) {
						string tile_id = tile->Attribute("id");
						for (TiXmlElement *rep = tile->FirstChildElement(); rep != NULL; rep = rep->NextSiblingElement()) {
							string r_level = rep->Attribute("id");
							string tile_binsize = rep->Attribute("binsize");
							mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level+"tile"+tile_id + "binsize", tile_binsize));
							string tile_C = rep->Attribute("CR");
							mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level + "tile" + tile_id + "C", tile_C));
							TiXmlNode *tile_url = rep->FirstChild();
							string tile_urls = baseurl+tile_url->ToText()->Value();
							mpd.insert(pair<string, string>("gof" + gof_id + "adp" + adp_id + "r" + r_level + "tile" + tile_id + "url", tile_urls));
						}
					}
				}
			}
		}
	
	// �����ڴ�
	doc.Clear();
	return SUCCESS;
}

PCLVisualizer::PCLVisualizer(QWidget *parent)
{
	thread = new render_thread(this);
	thread->setObj(this);
	ui.setupUi(this);
	threadpool.setMaxThreadCount(4);
	//��ʼ��
	initialVtkWidget();
	connect(ui.button_play, SIGNAL(clicked()), this, SLOT(download()));//���Ű�ť�Ĳ�
	connect(this, SIGNAL(sigrender()), this, SLOT(slotrender()));
	connect(&manager, SIGNAL(finished(QNetworkReply *)), this, SLOT(replyFinished(QNetworkReply *)));	//������ɺ�Ĳ�
}

void PCLVisualizer::slotrender() {
	//qApp->processEvents();//ˢ��,����spin()
	//ui.qvtkWidget->update();
}

void PCLVisualizer::run_render_thread() {
		emit sigrender();	
}

PCLVisualizer::~PCLVisualizer()
{
	delete &ui;
}

void PCLVisualizer::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud, "cloud");
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	viewer->addText("", 5, 5, "fov");
	viewer->addText("", 10, 10, "tile_parse");
	viewer->setCameraPosition(438, 905, 1656, 0,0, 1);
}

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
	size_t n_point = count(frame.begin(), frame.end(), '\r');//�����
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

bool getXYZRGB(string point,PCCPoint3D &position,PCCColor3B &color) {
	int pos_block = 0;
	vector<int16_t> x_y_z;
	vector<uint8_t> r_g_b;
	int16_t xyzrgb[6];
	
	
	for (int i = 0; i < 6; i++) {
		xyzrgb[i] = stof(point.substr(pos_block, point.find(' ', pos_block) - pos_block));
		pos_block = point.find(' ', pos_block) + 1;
	

	}

	PCCPoint3D a(xyzrgb[0], xyzrgb[1], xyzrgb[2]);
	PCCColor3B b(xyzrgb[3], xyzrgb[4], xyzrgb[5]);

	position = a;
	color = b;

	
	return true;
}

int frame2xyzrgb(string ply, PCCPointSet3 &frame) {

	
	size_t size = ply.size();
	size_t n_point = count(ply.begin(), ply.end(), '\r');//�����
	size_t offset = 0;
	string point;
	for (size_t i = 0; i < n_point; i++) {
		point = ply.substr(offset, ply.find('\r', offset) - offset);
		offset = ply.find('\r', offset + 2) + 2;
		PCCPoint3D position;
		PCCColor3B color;
		getXYZRGB(point, position, color);
		frame.addPoint(position, color);
		point.clear();
	}
	size_t pos = ply.find("/n", 0);
	return 1;
}

vector<string> tile_choose(int a[][5], int *e, map<string,string> mdp,const int current_gof,vector<string> &tile_id,vector<int> &frame_id) {

	vector<string> tiles;
	map<int, vector<int>> tile_info;
	map<int, vector<int>>::iterator iter;
	map<int, vector<string>> tiles_url;
	vector<string> urls;
	vector<string> request;
	int r;
	vector<int> r_form;

	//x,eת����tile_info
	for (int i = 0; i < 12; i++) {
		r = 0;
		for (int j = 0; j < 5; j++) {
			int s = a[i][j];
			r = s + r;
		}

		if (r != 0) {
	
			int form = e[i];
			r_form.push_back(r);
			r_form.push_back(form);
			tile_info.insert(pair<int, vector<int>>(i, r_form));
			r_form.clear();

		}
	}



	//tile_infoת��Ϊmpd�еļ�
		for (iter = tile_info.begin(); iter != tile_info.end(); iter++) {
			int tile_id=iter->first+1;
			int form = iter->second.at(1);
			int r=iter->second.at(0);
			
			//bin��ʽ
			if (form == 1) {

				string tile_url = "gof" + to_string(current_gof) + "adp2"+"r"+to_string(r)+"tile"+to_string(tile_id)+"url";
				urls.push_back(tile_url);
				tiles_url.insert(pair<int, vector<string>>(tile_id, urls));
				frame_id.push_back(1);

			}
			//ply
			else {
				for (int i = 0; i < 10; i++) {

					string tile_url = "gof" + to_string(current_gof) + "adp1" + "r" + to_string(r) + "frame" + to_string(i + 1) + "tile" + to_string(tile_id) + "url";
					urls.push_back(tile_url);
					frame_id.push_back(i+1);
				}
				tiles_url.insert(pair<int, vector<string>>(tile_id, urls));		
			}			
			urls.clear();
			
		}
	
	//ƥ��mpd�м���Ӧֵ
	map<int, vector<string>>::iterator tiles_url_iter;//tile)id->simpleurl
	map<string, string>::iterator mpd_iter;// simleurl->url
	for (tiles_url_iter = tiles_url.begin(); tiles_url_iter != tiles_url.end(); tiles_url_iter++) {
		vector<string>::iterator tile_iter;//simpleurl
		for (tile_iter = tiles_url_iter->second.begin(); tile_iter != tiles_url_iter->second.end(); tile_iter++) {
			mpd_iter = mpd.find(*tile_iter);
				//�ж�Ӧ��
			if (mpd_iter!=mpd.end()) {

				string tile_url = mpd_iter->second;
				tile_id.push_back(to_string(tiles_url_iter->first));
				request.push_back(tile_url);
			}
		}	
	}
		return request;
}
	

//bool BestQoE(double Bw,int buffer_size,)

void PCLVisualizer::decode_thread() {
	//���������߳�
	QtConcurrent::run(&threadpool,[this]() {
		map<QString, vector<uint8_t>>::iterator iterbin;
		while (true) {
			if (bin_store.size() > 0) {
				iterbin = bin_store.begin();
				QString gof_tiles = iterbin->first;
				QRegExp rx(QString("(\\d+)_(\\d+)"));
				int pos = gof_tiles.indexOf(rx);
				int gof_id = rx.cap(1).toInt();
				int tile_id = rx.cap(2).toInt();
				PCCDecoderParameters decoderParams;
				PCCMetricsParameters metricsParams;
				PCCBitstream bitstream;
				bitstream.initialize(iterbin->second);

				if (!bitstream.readHeader()) { return -1; }
				size_t      frameNumber = decoderParams.startFrameNumber_;
				PCCMetrics  metrics;
				PCCChecksum checksum;
				metrics.setParameters(metricsParams);
				checksum.setParameters(metricsParams);
				std::vector<std::vector<uint8_t>> checksumsRec, checksumsDec;
				if (metricsParams.computeChecksum_) { checksum.read(decoderParams.compressedStreamPath_); }
				PCCDecoder decoder;
				decoder.setParameters(decoderParams);
				while (bitstream.size() < bitstream.capacity()) {
					PCCGroupOfFrames reconstructs;
					PCCContext       context;
					int ret = decoder.decode(bitstream, context, reconstructs);//�ؼ�
					if (ret) { return ret; }
					vector<PCCPointSet3>::iterator iter;
					map<int, PCCPointSet3>::iterator itermap;

					for (iter = reconstructs.begin(); iter != reconstructs.end(); iter++) {
						int frame_id = frameNumber++ - 999;
						itermap = fusions[gof_id - 1].find(frame_id);

						if (itermap != fusions[gof_id - 1].end()) {
							//����,һ������
							vector<PCCPoint3D> points = iter->getPositions();
							
							vector<PCCPoint3D>::iterator iterpoint;
							int i = 0;
							for (iterpoint = points.begin(); iterpoint != points.end(); iterpoint++) {
								itermap->second.addPoint(*iterpoint,iter->getColor(i++));
								
							}
							gof_tile[gof_id - 1]--;
						}
						else {
							//��һ��
							fusions[gof_id - 1].insert(pair<int, PCCPointSet3>(frame_id, *iter));
							gof_tile[gof_id - 1]--;
						}

					}
				}
				bin_store.erase(iterbin);
				
			}
		}
	});


}


void PCLVisualizer::readbuffer_thread() {
	QtConcurrent::run(&threadpool,[this]() {
		int i = 0;
		viewer->addCoordinateSystem(1.0);
		while (true) {
			int *p;
			p = std::find(gof_tile, gof_tile + 1, 0);	
			if (p != gof_tile + 1) {
				int pos = p - gof_tile;
				*p = 1;//
				
				map<int, PCCPointSet3>::iterator iter;
				for (iter = fusions[pos].begin(); iter != fusions[pos].end(); iter++) {
					Sleep(333);
					mymutex.lock();
					cloud->clear();
					int point_n = iter->second.getPointCount();
					cloud->points.resize(point_n);
					for (int i = 0; i < point_n; i++) {									
						cloud->points[i].x = iter->second[i].x();
						cloud->points[i].y = iter->second[i].y();
						cloud->points[i].z = iter->second[i].z();
						cloud->points[i].r = iter->second.getColor(i).r();
						cloud->points[i].g = iter->second.getColor(i).g();
						cloud->points[i].b = iter->second.getColor(i).b();				
					}
					
					getviewer()->updatePointCloud(getcloud(), "cloud");
					mymutex.unlock();
					
				}
			}		
		}
	});
}

//play��ť(���������������)
void PCLVisualizer::download() {
	decode_thread();//�����߳�
	readbuffer_thread();//������ȡ������߳�

	loadXML("mainfest.xml"); 
	//����tiles,buffer,Bw, �õ�x e

		//ģ������Ӧ�㷨�Ľ��
	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 5; j++) {
			if (j == 0) {
				a[i][j] = 1;
			}
			else
				a[i][j] = 0;
		}
	}
	a[0][0] = 0;
	a[1][0] = 0;
	a[2][0] = 0;
	a[3][0] = 0;
	a[4][0] = 0;
	a[5][0] = 0;
	a[6][0] = 0;
	a[7][0] = 0;


	vector<string>::iterator tiles_url_iter;
	
	for (int current_gof = 1; current_gof <= 30; current_gof++) {
		//ÿ�μ���a,e��ֵ
		//if(!bestQoE(Bw,buffer.size(),FOV,a,x))
		int tile_n = 40;
		gof_tile[current_gof - 1] = tile_n;
		vector<string> tile_id;//ÿ������ply�ļ���bin�ļ�
		vector<int> frame_id;
		start = GetTickCount();
		//ͨ��a,e�õ���Ӧmpd�õ�tile��ַ
		vector<string> tiles_url = tile_choose(a, e, mpd, current_gof,tile_id,frame_id);
		int i = 0;
		for (tiles_url_iter = tiles_url.begin(); tiles_url_iter != tiles_url.end(); tiles_url_iter++) {
			QString url = QString::fromStdString(*tiles_url_iter);	
			QString reg = ".+(.bin)$";
			QRegExp rx(reg);
			bool match = rx.exactMatch(url);
			string tile_ids = tile_id[i];
			int frame_ids = frame_id[i];
			i++;
			if (match) {
				request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof)) + "tile_" + QString::fromStdString(tile_ids)));
				manager.get(request);
			}
			else {
				request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof)) + "tile_" + QString::fromStdString(tile_ids)+"frame_"+ QString::fromStdString(to_string(frame_ids))));
				manager.get(request);
			}	
		}
	}

	while (true) {
		mymutex.lock();
		qApp->processEvents();
		ui.qvtkWidget->update();
		
		Eigen::Affine3f s;
		s = viewer->getViewerPose();
		Eigen::Matrix4f M;
		M = s.matrix();
		float a0 = M(0);
		float a1 = M(1);
		float a2 = M(2);
		float a3 = M(3);
		float a4 = M(4);
		float a5 = M(5);
		float a6 = M(6);
		float a7 = M(7);
		float a8 = M(8);
		float a9 = M(9);
		float a10 = M(10);
		float a11 = M(11);
		float a12 = M(12);
		float a13 = M(13);
		float a14 = M(14);
		string fov = to_string(a0) + "_" + to_string(a1) + "_" + to_string(a2) + "_" + to_string(a3) + "_" + to_string(a4) + "_" + to_string(a5) + "_" + to_string(a6) + "_" + to_string(a7) + to_string(a8) + "_" + to_string(a9) + "_" + to_string(a10) + "_" + to_string(a11) + "_" + to_string(a12) + "_" + to_string(a13) + "_" + to_string(a14);
		viewer->updateText(fov, 15, 15, "fov");
		viewer->updateText(to_string(gof_tile[0]), 100, 100, "tile_parse");
		mymutex.unlock();
	}
}

//����
void PCLVisualizer::replyFinished(QNetworkReply *reply)
{
	if (reply->error() == QNetworkReply::NoError)
	{
		double end = GetTickCount();
		cost = (end-start)/1000;
		QString fileName;

		if (reply->hasRawHeader(QString("Content-Disposition").toUtf8()))
		{
			fileName = reply->rawHeader(QString("Content-Disposition").toUtf8());

		}
		//����ƥ��
		QString reg = ".+(.bin)$";
		QRegExp rx(reg);
		bool match = rx.exactMatch(fileName);
		QByteArray bytes = reply->readAll();
		
		if (match) {
			//�����bin�ļ�
			QRegExp rx(QString("(\\d+)tile_(\\d+)"));
			int pos = fileName.indexOf(rx);
			QString gof_id = rx.cap(1);
			QString tile_id = rx.cap(2);
			QString gof_tile = gof_id + "_" + tile_id;
			vector<uint8_t> temp(bytes.begin(),bytes.end());
			bin_store.insert(pair<QString, vector<uint8_t>>(gof_tile, temp));//д���ѹ��
		}

		//�����ply�ļ�,����,��ȡframe_id
		else {
			QRegExp rx(QString("(\\d+)tile_(\\d+)frame_(\\d+)"));
			//QRegExp rx(QString("(?<=_)(\\d+)"));
			int pos = fileName.indexOf(rx);	
			int gof_id = rx.cap(1).toInt();
			int tile_id = rx.cap(2).toInt();
			int frame_id = rx.cap(3).toInt();
			int offset=bytes.lastIndexOf('r');
			string ply = bytes.mid(offset+3, bytes.size());
			map<int, PCCPointSet3>::iterator iter;
			iter = fusions[gof_id - 1].find(frame_id);
			if (iter!=fusions[gof_id-1].end()) {
				//����Ѿ��ж�Ӧ��֡  ������tile�ĵ�
				frame2xyzrgb(ply, iter->second);
				gof_tile[gof_id-1]--;
			}
			else {
				//��һ������
				PCCPointSet3 frame;
				frame2xyzrgb(ply, frame);
				fusions[gof_id-1].insert(pair<int, PCCPointSet3>(frame_id,frame));
				gof_tile[gof_id - 1]--;
			}
		}
		reply->deleteLater();
	}
	else
	{
		//noreply
	}
}