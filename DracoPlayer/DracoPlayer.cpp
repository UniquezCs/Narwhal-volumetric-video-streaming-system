#include "DracoPlayer.h"
#include <qlist.h>
#include <algorithm>
#ifdef TIXML_USE_STL
#include <iostream>
#include <sstream>
#include <qdebug>
using namespace std;
#else
#include <stdio.h>
#endif
#if defined( WIN32 ) && defined( TUNE )
#include <crtdbg.h>
_CrtMemState startMemState;
_CrtMemState endMemState;
#endif
#include "tinyxml2.h"
#include <string>
#include <map>
#include "my3d.h"
#define PI acos(-1)
#ifndef abcde//ifndef做预处理，进行条件编译

//typedef IloArray<IloIntVarArray> IntVarMatrix;
//typedef IloArray<IloNumVarArray> NumVarMatrix;
//typedef IloArray<IloIntArray> IntMatrix;
//typedef IloArray<IloNumArray> NumMatrix;
//定义各种数组
#endif

using namespace pcc;
using namespace std;

//模拟hololens2的fov
#define FOV_Y (29)
#define FOV_X (43)
double fovx2fovy = tan(FOV_Y / 2 * PI / 180) / tan(FOV_X / 2 * PI / 180);//长度转换


//xml解析出来的结构体:
//每个tile的信息,ply形式
struct frame_tile {
	int tile_id;
	int point_n;
	double size;//kb
	string url;
};

//frmae 信息,ply
struct  frame_st
{
	int frame_id;
	int point_n;
	int r;
	string url;
	vector<frame_tile> tiles;
};

//每个tile的信息,bin形式
struct tile_gof {
	int tile_id;
	int r;
	double binsize;
	double CR;
	string url;
	point3 points[8];//每个tile八个点的坐标
	point3 focal;//中心点坐标
};

struct notile_gof {
	int r;
	string url;
};
//gof信息
struct gof {
	int minx;//包围的长方体大小
	int miny;
	int minz;
	int maxx;
	int maxy;
	int maxz;
	int gof_id;
	vector<tile_gof> tiles;//bin
	vector<frame_st> frames;//ply
	vector<notile_gof> notiles;
};
//总视频信息
struct object {
	int frame_n;
	int gof_n;
	int cuboid_n;//切块方式nxnxm
	int cuboid_m;
	int flag;//切块方向,
	string baseurl;
	vector<gof> gofs;

};

QString stuff_ply = ".ply";
int current_gof = 1;//全局变量 记录当前处于哪个gof
map<string, string>mpd;
object obj;//视频对象
vector<pcl::visualization::Camera> camera;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

vector<vector<pcl::visualization::Camera>> fovs;//记录播放开始前发送每个gof时刻的视角
//多线程的锁
QMutex mymutex(QMutex::Recursive);
QMutex bin_mutex(QMutex::Recursive);
QMutex fusion_mutex(QMutex::Recursive);
QMutex buffer_mutex(QMutex::Recursive);

vector<vector<float>> xyzs;//一帧的所有坐标（包含坐标中的坐标点）

vector<int> ev;//传输形式
vector<int> cv;//是否传输
vector<int> xv;//质量等级
//记录数据:
vector<vector<int>> re_ev;
vector<vector<int>> re_cv;
vector<vector<int>> re_xv;
vector<vector<point3>> re_fov;
//显示视角信息:

string fov_focal;
string fov_pos;
int scheme;//1=tile_compressed 2=tile_uncomp 3=joint,4=notile_compres  5=notile_uncomp

map<int, PCCPointSet3> fusion;// frame_id->
int tile_parse;
int gof_tile[30] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1 };//验证是否融合完成 gof_id->tilenumber


map<QString, vector<uint8_t>> bin_store;//(tile_id,gof_id）->bin


int bandwidth = 1000;

//切块相关 为了与mpd文件中切块对应
point3* move2x(point3 *tile_xyz, float x) {
	point3 temp[8];
	for (int i = 0; i < 8; i++) {
		temp[i].x = tile_xyz[i].x + x;
		temp[i].y = tile_xyz[i].y;
		temp[i].z = tile_xyz[i].z;
	}
	return temp;
}


point3* move2y(point3 *tile_xyz, float y) {

	point3 temp[8];
	for (int i = 0; i < 8; i++) {
		temp[i].x = tile_xyz[i].x;
		temp[i].y = tile_xyz[i].y + y;
		temp[i].z = tile_xyz[i].z;
	}

	return temp;
}


point3* move2z(point3 *tile_xyz, float z) {

	point3 temp[8];
	for (int i = 0; i < 8; i++) {
		temp[i].x = tile_xyz[i].x;
		temp[i].y = tile_xyz[i].y;
		temp[i].z = tile_xyz[i].z + z;
	}
	return temp;
}


void movex(tile_gof &tile_xyz, float x) {

	for (int i = 0; i < 8; i++) {
		tile_xyz.points[i].x += x;
	}

}


void movey(tile_gof &tile_xyz, float y) {

	for (int i = 0; i < 8; i++) {
		tile_xyz.points[i].y += y;
	}

}


void movez(tile_gof &tile_xyz, float z) {

	for (int i = 0; i < 8; i++) {
		tile_xyz.points[i].z += z;
	}

}


//每层的切块坐标
vector<tile_gof> gettile_seg(int j, const int n, float tile_h, float tile_x, float tile_z) {

	tile_gof tile1;
	tile1.tile_id = j * n*n;
	vector<tile_gof> tiles_seg;

	for (int k = 0; k < 4; k++) {//创建每层的初始块（顶点在(0,y,0)）处
		point3 point1;
		if (k == 0) {
			point1.x = 0;
			point1.z = 0;
			point1.y = j * tile_h;
			tile1.points[k] = point1;
			point1.y = j * tile_h + tile_h;
			tile1.points[k + 4] = point1;
		}
		else if (k == 1) {
			point1.x = tile_x;
			point1.z = 0;
			point1.y = j * tile_h;
			tile1.points[k] = point1;
			point1.y = j * tile_h + tile_h;
			tile1.points[k + 4] = point1;
		}
		else if (k == 2) {
			point1.x = 0;
			point1.z = tile_z;
			point1.y = j * tile_h;
			tile1.points[k] = point1;
			point1.y = j * tile_h + tile_h;
			tile1.points[k + 4] = point1;
		}
		else if (k == 3) {
			point1.x = tile_x;
			point1.z = tile_z;
			point1.y = j * tile_h;
			tile1.points[k] = point1;
			point1.y = j * tile_h + tile_h;
			tile1.points[k + 4] = point1;
		}

	}

	tile1.tile_id = j * n*n;
	tiles_seg.push_back(tile1);


	for (int i = 0; i < n; i++) {
		if (i == 0) {
			for (int f = 1; f < n; f++) {
				for (int k = 0; k < 8; k++) {
					tile1.points[k] = *(move2x(tiles_seg[f - 1].points, tile_x) + k);
				}
				tile1.tile_id = f + j * n*n;
				tiles_seg.push_back(tile1);
			}
		}
		else {
			for (int s = 0; s < n; s++) {
				for (int k = 0; k < 8; k++) {
					tile1.points[k] = *(move2z(tiles_seg[s + (i - 1)*n].points, tile_z) + k);
				}
				tile1.tile_id = s + n * i + j * n*n;
				tiles_seg.push_back(tile1);
			}
		}
	}
	return tiles_seg;
}


//读取mpd文件
bool loadXML(const char *filename)
{
	// XML文档
	tinyxml2::XMLDocument doc;
	while (true) {
		if (doc.LoadFile(filename))
		{
			qApp->processEvents();
			continue;
		}
		else {
			break;
		}
	}

	tinyxml2::XMLElement *root = doc.FirstChildElement("info");
	if (root == NULL)
	{
		doc.Clear();
		return false;
	}
	else
	{
		obj.frame_n = stoi(root->Attribute("frames"));
		obj.gof_n = stoi(root->Attribute("GOF"));
		obj.cuboid_n = stoi(root->Attribute("n"));
		obj.cuboid_m = stoi(root->Attribute("m"));
		obj.flag = stoi(root->Attribute("flag"));
		obj.baseurl = root->Attribute("BaseURL");
	}
	// 遍历子节点
	for (tinyxml2::XMLElement *elem = doc.FirstChildElement("GOF"); elem != NULL; elem = elem->NextSiblingElement())
	{
		// 获取元素名
		gof gof_tmp;
		gof_tmp.gof_id = stoi(elem->Attribute("id"));
		gof_tmp.minx = stoi(elem->Attribute("minx"));
		gof_tmp.miny = stoi(elem->Attribute("miny"));
		gof_tmp.minz = stoi(elem->Attribute("minz"));
		gof_tmp.maxx = stoi(elem->Attribute("maxx"));
		gof_tmp.maxy = stoi(elem->Attribute("maxy"));
		gof_tmp.maxz = stoi(elem->Attribute("maxz"));

		//判断adp类型
		for (tinyxml2::XMLElement *AdaptationSet = elem->FirstChildElement("AdaptationSet"); AdaptationSet != NULL; AdaptationSet = AdaptationSet->NextSiblingElement()) {
			string adp_id = AdaptationSet->Attribute("id");
			// 未压缩的点云信息
			if (adp_id._Equal("1")) {
				for (tinyxml2::XMLElement *Rep = AdaptationSet->FirstChildElement(); Rep != NULL; Rep = Rep->NextSiblingElement()) {
					string r_level = Rep->Attribute("id");
					for (tinyxml2::XMLElement *frame = Rep->FirstChildElement(); frame != NULL; frame = frame->NextSiblingElement()) {
						frame_st frame_tmp;
						frame_tmp.frame_id = stoi(frame->Attribute("id"));
						frame_tmp.point_n = stoi(frame->Attribute("PointN"));
						frame_tmp.r = stoi(r_level);

						for (tinyxml2::XMLElement *tile = frame->FirstChildElement(); tile != NULL; tile = tile->NextSiblingElement()) {

							frame_tile frame_tile_tmp;
							frame_tile_tmp.point_n = stoi(tile->Attribute("PointN"));
							frame_tile_tmp.tile_id = stoi(tile->Attribute("id"));
							tinyxml2::XMLNode *tile_url = tile->FirstChild();
							frame_tile_tmp.url = obj.baseurl + tile_url->ToText()->Value();
							frame_tmp.tiles.push_back(frame_tile_tmp);

							mpd.insert(pair<string, string>("gof" + to_string(gof_tmp.gof_id) + "adp" + adp_id + "r" + r_level + "frame" + to_string(frame_tmp.frame_id) + "tile" + to_string(frame_tile_tmp.tile_id) + "url", frame_tile_tmp.url));

						}
						gof_tmp.frames.push_back(frame_tmp);
					}
				}
			}

			//压缩后的点云信息
			else {
				for (tinyxml2::XMLElement *tile = AdaptationSet->FirstChildElement(); tile != NULL; tile = tile->NextSiblingElement()) {

					for (tinyxml2::XMLElement *rep = tile->FirstChildElement(); rep != NULL; rep = rep->NextSiblingElement()) {
						tile_gof tile_gof_tmp;
						tile_gof_tmp.tile_id = stoi(tile->Attribute("id"));
						tile_gof_tmp.r = stoi(rep->Attribute("id"));
						tinyxml2::XMLNode *tile_url = rep->FirstChild();
						tile_gof_tmp.url = obj.baseurl + tile_url->ToText()->Value();
						gof_tmp.tiles.push_back(tile_gof_tmp);
						mpd.insert(pair<string, string>("gof" + to_string(gof_tmp.gof_id) + "adp" + adp_id + "r" + to_string(tile_gof_tmp.r) + "tile" + to_string(tile_gof_tmp.tile_id) + "url", tile_gof_tmp.url));
					}
				}
			}
		}

		//解析每个tile的八个顶点
		size_t relativexyz[3];
		vector<tile_gof> tiles;
		int tile_number = obj.cuboid_n * obj.cuboid_n*obj.cuboid_m;//块数
		float relativex = gof_tmp.maxx - gof_tmp.minx;
		float relativey = gof_tmp.maxy - gof_tmp.miny;
		float relativez = gof_tmp.maxz - gof_tmp.minz;
		//flag=1表示以xy平面为水平面
		if (obj.flag == 1) {
			//
		}
		else if (obj.flag == 2) {
			//
		}
		else if (obj.flag == 3) {
			float tile_z = (relativez / obj.cuboid_n);
			float tile_x = (relativex / obj.cuboid_n);
			float tile_h = (relativey / obj.cuboid_m);
			//获取每层的切块
			for (int i = 0; i < obj.cuboid_m; i++) {
				//获取该层切块
				vector<tile_gof> tile_seg = gettile_seg(i, obj.cuboid_n, tile_h, tile_x, tile_z);

				for (int k = 0; k < obj.cuboid_n*obj.cuboid_n; k++) {
					movex(tile_seg[k], gof_tmp.minx);
					movey(tile_seg[k], gof_tmp.miny);
					movez(tile_seg[k], gof_tmp.minz);

					double focalx = 0;
					double focaly = 0;
					double focalz = 0;

					for (int j = 0; j < 8; j++) {
						gof_tmp.tiles.at(obj.cuboid_n*obj.cuboid_n*i + k).points[j].x = tile_seg[k].points[j].x;
						gof_tmp.tiles.at(obj.cuboid_n*obj.cuboid_n*i + k).points[j].y = tile_seg[k].points[j].y;
						gof_tmp.tiles.at(obj.cuboid_n*obj.cuboid_n*i + k).points[j].z = tile_seg[k].points[j].z;
						focalx += tile_seg[k].points[j].x;
						focaly += tile_seg[k].points[j].y;
						focalz += tile_seg[k].points[j].z;

					}
					gof_tmp.tiles.at(obj.cuboid_n*obj.cuboid_n*i + k).focal.x = focalx / 8;
					gof_tmp.tiles.at(obj.cuboid_n*obj.cuboid_n*i + k).focal.y = focaly / 8;
					gof_tmp.tiles.at(obj.cuboid_n*obj.cuboid_n*i + k).focal.z = focalz / 8;
				}
			}
		}
		else {
			//error input
		}
		obj.gofs.push_back(gof_tmp);
	}

	// 清理内存
	doc.Clear();
	return true;
}


bool loadXML_notile(const char *filename) {
	// XML文档
	tinyxml2::XMLDocument doc;
	while (true) {
		if (doc.LoadFile(filename))
		{
			qApp->processEvents();
			continue;
		}
		else {
			break;
		}
	}

	tinyxml2::XMLElement *root = doc.FirstChildElement("info");
	if (root == NULL)
	{
		doc.Clear();
		return false;
	}
	else
	{
		obj.frame_n = stoi(root->Attribute("frames"));
		obj.gof_n = stoi(root->Attribute("GOF"));
		obj.flag = stoi(root->Attribute("flag"));
		obj.baseurl = root->Attribute("BaseURL");
	}
	// 遍历子节点
	for (tinyxml2::XMLElement *elem = doc.FirstChildElement("GOF"); elem != NULL; elem = elem->NextSiblingElement())
	{
		// 获取元素名
		gof gof_tmp;
		gof_tmp.gof_id = stoi(elem->Attribute("id"));
		gof_tmp.minx = stoi(elem->Attribute("minx"));
		gof_tmp.miny = stoi(elem->Attribute("miny"));
		gof_tmp.minz = stoi(elem->Attribute("minz"));
		gof_tmp.maxx = stoi(elem->Attribute("maxx"));
		gof_tmp.maxy = stoi(elem->Attribute("maxy"));
		gof_tmp.maxz = stoi(elem->Attribute("maxz"));

		//判断adp类型
		for (tinyxml2::XMLElement *AdaptationSet = elem->FirstChildElement("AdaptationSet"); AdaptationSet != NULL; AdaptationSet = AdaptationSet->NextSiblingElement()) {
			string adp_id = AdaptationSet->Attribute("id");
			// 未压缩的点云信息
			if (adp_id._Equal("1")) {
				for (tinyxml2::XMLElement *Rep = AdaptationSet->FirstChildElement(); Rep != NULL; Rep = Rep->NextSiblingElement()) {
					string r_level = Rep->Attribute("id");
					for (tinyxml2::XMLElement *frame = Rep->FirstChildElement(); frame != NULL; frame = frame->NextSiblingElement()) {
						frame_st frame_tmp;
						frame_tmp.frame_id = stoi(frame->Attribute("id"));
						frame_tmp.point_n = stoi(frame->Attribute("PointN"));
						frame_tmp.r = stoi(r_level);
						tinyxml2::XMLNode *frame_url = frame->FirstChild();
						frame_tmp.url = obj.baseurl + frame_url->ToText()->Value();
						mpd.insert(pair<string, string>("gof" + to_string(gof_tmp.gof_id) + "adp" + adp_id + "r" + r_level + "frame" + to_string(frame_tmp.frame_id) + "url", frame_tmp.url));
						gof_tmp.frames.push_back(frame_tmp);
					}
				}
			}
			//压缩后的点云信息
			else {
				for (tinyxml2::XMLElement *rep = AdaptationSet->FirstChildElement(); rep != NULL; rep = rep->NextSiblingElement()) {
					notile_gof notile;
					notile.r = stoi(rep->Attribute("id"));
					tinyxml2::XMLNode *url = rep->FirstChild();
					notile.url = obj.baseurl + url->ToText()->Value();
					mpd.insert(pair<string, string>("gof" + to_string(gof_tmp.gof_id) + "adp" + adp_id + "r" + to_string(notile.r) + "url", notile.url));
					gof_tmp.notiles.push_back(notile);
				}
			}
		}
		obj.gofs.push_back(gof_tmp);
	}
	// 清理内存
	doc.Clear();
	return true;

}


//构造函数
DracoPlayer::DracoPlayer(QWidget *parent)
{
	ui.setupUi(this);
	threadpool.setMaxThreadCount(4);//线程池
	initialVtkWidget();

	/*******************************************连接按钮信号和槽************************************************/
	//connect(ui.button_play_Tile, SIGNAL(clicked()), this, SLOT(download()));
	//connect(ui.button_play_notile, SIGNAL(clicked()), this, SLOT(download2()));

	/******************************************************??*************************************************/
	connect(this, SIGNAL(siggsend()), this, SLOT(send()));
	connect(this, SIGNAL(siggsend2()), this, SLOT(send2()));
	connect(this, SIGNAL(sig_ply2fusion(string, PCCPointSet3, int, int)), this, SLOT(frame2xyzrgb(string, PCCPointSet3, int, int)));
	connect(this, SIGNAL(sig_ply2fusion_add(string, &PCCPointSet3, int, int)), this, SLOT(frame2xyzrgb_add(string, PCCPointSet3, int, int)));

	/******************************************下载完成后的对应槽*******************************************/
	connect(&manager, SIGNAL(finished(QNetworkReply *)), this, SLOT(replyFinished(QNetworkReply *)));
	connect(&manager2, SIGNAL(finished(QNetworkReply *)), this, SLOT(replyFinished2(QNetworkReply *)));
}


DracoPlayer::~DracoPlayer()
{
	delete &ui;
}


void DracoPlayer::initialVtkWidget()
{
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer.reset(new pcl::visualization::PCLVisualizer("NarWhal", false));
	viewer->addPointCloud(cloud, "cloud");
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	viewer->addText("", 5, 75, 40, 255, 255, 255, "tile_compressed");
	viewer->addText("", 5, 75, 40, 255, 255, 255, "tile_uncompressed");
	viewer->addText("", 5, 75, 40, 255, 255, 255, "current_gof");
	viewer->addText("", 5, 75, 40, 255, 255, 255, "buffer_size");

}


//下载后的未压缩点云,字符串格式转坐标点
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


//string类型的point转化为positon和color
bool getXYZRGB(string point, PCCPoint3D &position, PCCColor3B &color) {
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


//下载的ply形式点云直接放入fusion
int DracoPlayer::frame2xyzrgb(string ply, int gof_id, int frame_id, int is_tile) {
	PCCPointSet3 frame;
	frame.clear();
	size_t n_point = count(ply.begin(), ply.end(), '\r');//点个数
	size_t offset = 0;
	string point;
	PCCPoint3D position;
	fusion_mutex.lock();
	PCCColor3B color;
	for (size_t i = 0; i < n_point; i++) {
		point = ply.substr(offset, ply.find('\r', offset) - offset);
		offset = ply.find('\r', offset + 2) + 2;
		getXYZRGB(point, position, color);
		frame.addPoint(position, color);
	}
	fusion.insert(pair<int, PCCPointSet3>(frame_id, frame));
	fusion_mutex.unlock();
	gof_tile[gof_id - 1]--;
	if (gof_tile[gof_id - 1] == 0) {
		QtConcurrent::run(&threadpool, this, &DracoPlayer::writebuffer_thread, is_tile);
	}
	return 1;
}


//传输原始
int DracoPlayer::frame2xyzrgb2(string ply, int gof_id, int frame_id) {

	PCCPointSet3 frame;
	frame.clear();
	size_t n_point = count(ply.begin(), ply.end(), '\n');//点个数
	size_t offset = 0;
	string point;
	PCCPoint3D position;
	PCCColor3B color;
	for (size_t i = 0; i < n_point; i++) {
		point = ply.substr(offset, ply.find('\n', offset) - offset);
		offset = ply.find('\n', offset + 1) + 1;
		getXYZRGB(point, position, color);
		frame.addPoint(position, color);
	}
	fusion.insert(pair<int, PCCPointSet3>(frame_id, frame));

	return 1;
}


int DracoPlayer::frame2xyzrgb_add(string ply, PCCPointSet3 *frame, int gof_id, int frame_id) {

	fusion_mutex.lock();
	size_t n_point = count(ply.begin(), ply.end(), '\r');//点个数
	size_t offset = 0;
	string point;
	PCCPoint3D position;
	PCCColor3B color;
	for (size_t i = 0; i < n_point; i++) {
		point = ply.substr(offset, ply.find('\r', offset) - offset);
		offset = ply.find('\r', offset + 2) + 2;
		getXYZRGB(point, position, color);
		frame->addPoint(position, color);
	}
	gof_tile[gof_id - 1]--;

	fusion_mutex.unlock();
	if (gof_tile[gof_id - 1] == 0) {
		QtConcurrent::run(&threadpool, this, &DracoPlayer::writebuffer_thread, 1);
	}

	return 1;
}


//将所得的x,e变量转化成对应的tile url
vector<string> tile_choose(vector<int> x, vector<int> e, map<string, string> mdp, const int current_gof, vector<string> &tile_id, vector<int> &frame_id) {

	vector<string> tiles;
	map<int, vector<int>> tile_info;
	map<int, vector<int>>::iterator iter;
	map<int, vector<string>> tiles_url;
	vector<string> urls;
	vector<string> request;

	vector<int> r_form;
	//x,e转化成tile_info
	for (int i = 0; i < 12; i++) {
		int r = 0;
		for (int j = 0; j < 5; j++) {

			if (x[12 * j + i] == 1) {
				r = j + 1;
				break;
			}
		}

		if (r != 0) {

			int form = e[i];
			r_form.push_back(r);
			r_form.push_back(form);
			tile_info.insert(pair<int, vector<int>>(i, r_form));
			r_form.clear();

		}


	}
	//tile_info转化为mpd中的键
	for (iter = tile_info.begin(); iter != tile_info.end(); iter++) {
		int tile_id = iter->first + 1;
		int form = iter->second.at(1);
		int r = iter->second.at(0);

		//bin形式
		if (form == 1) {

			string tile_url = "gof" + to_string(current_gof) + "adp2" + "r" + to_string(r) + "tile" + to_string(tile_id) + "url";
			urls.push_back(tile_url);
			tiles_url.insert(pair<int, vector<string>>(tile_id, urls));
			frame_id.push_back(1);
		}
		//ply
		else {
			for (int i = 0; i < 10; i++) {

				string tile_url = "gof" + to_string(current_gof) + "adp1" + "r" + to_string(r) + "frame" + to_string(i + 1) + "tile" + to_string(tile_id) + "url";
				urls.push_back(tile_url);
				frame_id.push_back(i + 1);
			}
			tiles_url.insert(pair<int, vector<string>>(tile_id, urls));
		}
		urls.clear();
	}

	//匹配mpd中键对应值
	map<int, vector<string>>::iterator tiles_url_iter;//tile)id->simpleurl
	map<string, string>::iterator mpd_iter;// simleurl->url
	for (tiles_url_iter = tiles_url.begin(); tiles_url_iter != tiles_url.end(); tiles_url_iter++) {
		vector<string>::iterator tile_iter;//simpleurl
		for (tile_iter = tiles_url_iter->second.begin(); tile_iter != tiles_url_iter->second.end(); tile_iter++) {
			mpd_iter = mpd.find(*tile_iter);
			//有对应项
			if (mpd_iter != mpd.end()) {

				string tile_url = mpd_iter->second;
				tile_id.push_back(to_string(tiles_url_iter->first));
				request.push_back(tile_url);
			}
		}
	}
	return request;
}


vector<string> tile_choose_notile(vector<int> x, vector<int> e, map<string, string> mdp, const int current_gof, vector<int> &frame_id) {

	vector<string> tiles;
	map<int, vector<int>>::iterator iter;
	map<int, vector<string>> tiles_url;
	vector<string> urls;
	vector<string> request;
	vector<int> r_form;//这个gof的等级和形式
	//x,e转化成tile_info
	int r = 0;
	for (int j = 0; j < 5; j++) {

		if (x[1 * j] == 1) {
			r = j + 1;
			break;
		}
	}
	if (r != 0) {
		int form = e[0];
		r_form.push_back(r);
		r_form.push_back(form);
	}

	//tile_info转化为mpd中的键
	int re = r_form[0];
	int form = r_form[1];

	//bin形式
	if (form == 1) {
		string tile_url = "gof" + to_string(current_gof) + "adp2" + "r" + to_string(re) + "url";
		urls.push_back(tile_url);
		frame_id.push_back(1);
	}
	//ply
	else {
		for (int i = 0; i < 10; i++) {
			string tile_url = "gof" + to_string(current_gof) + "adp1" + "r" + to_string(re) + "frame" + to_string(i + 1) + "url";
			urls.push_back(tile_url);
			frame_id.push_back(i + 1);
		}
	}
	//匹配mpd中键对应值
	vector<string>::iterator url_iter;//tile)id->simpleurl
	map<string, string>::iterator mpd_iter;// simleurl->url
	for (url_iter = urls.begin(); url_iter != urls.end(); url_iter++) {
		mpd_iter = mpd.find(*url_iter);
		//有对应项
		if (mpd_iter != mpd.end()) {
			string tile_url = mpd_iter->second;
			request.push_back(tile_url);
		}
	}
	return request;
}


//判断块的顶点和中心点是否在fov区域内
bool is_tileinfov(pcl::visualization::Camera ca, point3 p, point3 &pt_pos, point3 &focal_1, point3 &focal_2, point3 &focal_3, point3 &focal_4) {

	pt_pos.x = ca.pos[0]; pt_pos.y = ca.pos[1]; pt_pos.z = ca.pos[2]; fov_pos = "(" + to_string(ca.pos[0]) + "," + to_string(ca.pos[1]) + "," + to_string(ca.pos[2]) + ")";//实时显示fov信息
	point3 pt_focal; pt_focal.x = ca.focal[0]; pt_focal.y = ca.focal[1]; pt_focal.z = ca.focal[2]; fov_focal = "(" + to_string(ca.focal[0]) + "," + to_string(ca.focal[1]) + "," + to_string(ca.focal[2]) + ")";

	point3 vec_fp = subt(pt_focal, pt_pos);//视角向量

	double l_view = distance(pt_pos, pt_focal);//视角向量距离;

	double l_vertical = l_view * tan(FOV_Y / 2 * PI / 180);//竖直向量长

	point3 vec_vertical; vec_vertical.x = ca.view[0]; vec_vertical.y = ca.view[1]; vec_vertical.z = ca.view[2];//竖直向量

	vec_vertical.x = l_vertical / (vlen(vec_vertical))*vec_vertical.x; vec_vertical.y = l_vertical / (vlen(vec_vertical))*vec_vertical.y; vec_vertical.z = l_vertical / (vlen(vec_vertical))*vec_vertical.z;//修正长度

	double l_horizon = l_vertical / fovx2fovy;

	point3 vec_horizon = OuterProduct(vec_fp, vec_vertical);//水平向量

	vec_horizon.x = l_horizon / (vlen(vec_horizon))*vec_horizon.x; vec_horizon.y = l_horizon / (vlen(vec_horizon))*vec_horizon.y; vec_horizon.z = l_horizon / (vlen(vec_horizon))*vec_horizon.z;//修正长度

	point3 vec_1 = addv(vec_vertical, vec_horizon);
	point3 vec_2 = inverse(vec_1);
	point3 vec_3 = addv(inverse(vec_vertical), vec_horizon);
	point3 vec_4 = inverse(vec_3);

	focal_1 = addv(pt_focal, vec_1);//四条棱与中心面交点
	focal_2 = addv(pt_focal, vec_2);
	focal_3 = addv(pt_focal, vec_3);
	focal_4 = addv(pt_focal, vec_4);

	point3 pty1 = pointinplane(p, pt_pos, focal_1, focal_3);//上下平面投影点
	point3 pty2 = pointinplane(p, pt_pos, focal_2, focal_4);

	point3 vec_ty1 = subt(pty1, p);//计算投影向量
	point3 vec_ty2 = subt(pty2, p);

	point3 pty3 = pointinplane(p, pt_pos, focal_1, focal_4);//左右平面投影点
	point3 pty4 = pointinplane(p, pt_pos, focal_2, focal_3);

	point3 vec_ty3 = subt(pty3, p);
	point3 vec_ty4 = subt(pty4, p);


	if ((dmult(vec_ty3, vec_ty4) <= 0) && (dmult(vec_ty1, vec_ty2) <= 0)) {
		return true;
	}
	else {

		return false;
	}


}


//maxQOE求解
//bool maxQoE(double bandwidth, double Buffersize, vector<int>& xx, vector<int> &ee, vector<int> c, size_t kk, int scheme) {
//
//	IloEnv env;//定义环境
//	IloModel model(env);//定义模型
//	IloExpr obj(env);//表达式
//	IloInt R = 5;//等级数
//	IloInt K = kk;//切块数目
//	IloInt bw = 100;//带宽
//
//	vector<int> CC(c);//需要传输的块
//
//	IloInt NC = 4;//核心数
//	IloInt C1 = 6;//切块最小计算资源
//	IloInt maxNcore = 16;
//	IloInt CU12C = 1;//
//	vector <double>u(16);//转换效率
//	vector<IloInt>CU(16);//各个核心数计算资源
//
//	for (int i = 0; i < maxNcore; i++)
//	{
//		u[i] = exp(0.01*(-i));
//	}
//
//	for (size_t i = 0; i < maxNcore; i++)
//	{
//		CU[i] = i * u[i] * CU12C;
//	}
//
//
//	IloInt(*tile_pn)[12] = new IloInt[5][12]();//12x5的矩阵 每个切块的总点数
//	double(*tile_C)[12] = new double[5][12]();//每个切块的C
//	double(*binsize)[12] = new double[5][12]();
//	double(*plysize)[12] = new double[5][12]();
//
//	IloInt b0 = 20;//初始buffer
//	IloInt minp = LONG_MAX;
//	size_t tile_all = 0;//总点数
//	for (size_t k = 0; k < K; k++)
//	{
//		long tmp = pow((k + 1), 1.3) * 380000;
//		for (size_t r = 0; r < R; r++)
//		{
//			tile_pn[r][k] = round(tmp*pow(r + 1, 0.3));
//			if (r == 4) {
//				tile_all += tile_pn[r][k];
//			}
//			if (minp > tile_pn[r][k]) {
//				minp = tile_pn[r][k];
//			}
//		}
//	}
//
//	for (size_t k = 0; k < K; k++)
//	{
//		for (size_t r = 0; r < R; r++)
//		{
//			plysize[r][k] = 0.02734*tile_pn[r][k] / 1024;
//		}
//	}
//
//	for (size_t r = 0; r < R; r++)
//	{
//		for (size_t k = 0; k < K; k++)
//		{
//			tile_C[r][k] = IloLog2(pow(r + 2, 3)*tile_pn[r][k] / minp)*IloLog2(r + 2);
//		}
//	}
//
//	for (size_t r = 0; r < R; r++)
//	{
//		for (size_t k = 0; k < K; k++)
//		{
//			binsize[r][k] = plysize[r][k] * 2 * pow(r + 1, 2) / 800;
//		}
//	}
//
//	vector<double> w(K);//每个切块的权重
//	for (size_t k = 0; k < K; k++)
//	{
//		size_t s = tile_pn[4][k];
//		double t = s * 1.0 / tile_all;
//		w[k] = t;
//
//	}
//
//	vector<double> dist(K);//每个块的距离
//	for (size_t k = 0; k < K; k++)
//	{
//		dist[k] = k + 1;
//	}
//
//	double(*A)[12] = new double[5][12];//质量权重 5个12个
//	for (size_t r = 0; r < R; r++)
//	{
//		for (size_t k = 0; k < K; k++)
//		{
//			if (CC[k] == 0) {//考虑传输与不传
//				A[r][k] = 0;
//			}
//			else {
//				A[r][k] = (1.0 / dist[k])*(r + 1) * 8 * w[k];
//			}
//		}
//	}
//
//	//系数矩阵
//	IloNumArray b_1(env);
//	for (size_t i = 0; i < 12; i++)
//	{
//		b_1.add(A[0][i]);
//	}
//
//	IloNumArray b_2(env);
//	for (size_t i = 0; i < 12; i++)
//	{
//		b_2.add(A[1][i]);
//	}
//	IloNumArray b_3(env);
//	for (size_t i = 0; i < 12; i++)
//	{
//		b_3.add(A[2][i]);
//	}
//	IloNumArray b_4(env);
//	for (size_t i = 0; i < 12; i++)
//	{
//		b_4.add(A[3][i]);
//	}
//	IloNumArray b_5(env);
//	for (size_t i = 0; i < 12; i++)
//	{
//		b_5.add(A[4][i]);
//	}
//	NumMatrix b(env, 5);//系数矩阵 IloNum就是double 就是doule类型的矩阵
//	for (int i = 0; i < 5; i++)
//	{
//		b[i] = IloNumArray(env, 12);
//	}
//	b[0] = b_1; b[1] = b_2; b[2] = b_3; b[3] = b_4; b[4] = b_5;
//
//	//求解0,1变量矩阵
//	IntVarMatrix x(env, 5);
//	for (int i = 0; i < 5; i++)
//	{
//		x[i] = IloIntVarArray(env, 12, 0, 1);
//	}
//
//
//
//	IloExpr v1(env);//目标函数
//	IloExpr v2(env);//x[1][i]之和为1
//	IloExpr v3(env);//Td的表达式
//	IloExpr v4(env);//Ts的表达式
//	IloExpr v5(env);//Tb的约束
//
//	for (int i = 0; i < 5; i++)
//	{
//		for (int j = 0; j < 12; j++)
//		{
//			v1 += -(x[i][j] * A[i][j]);//求最小 取负数
//		}
//	}
//	model.add(IloMinimize(env, v1));//目标
//
//	for (int k = 0; k < 12; k++)
//	{
//		for (int r = 0; r < 5; r++)
//		{
//			v2 += (x[r][k]);
//		}
//		model.add(v2 == 1);
//		v2.clear();
//	}
//
//
//	if (scheme == 1) {
//		int e[12] = { 1,1,1,1,1,1,1,1,1,1,1,1 };
//
//		//Td
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v3 += e[k] * CC[k] * tile_C[r][k] * x[r][k];
//			}
//		}
//		double tdd = 3 * (NC*CU[NC] * CU12C);//Td的系数
//		v3 = v3 / tdd;
//
//		//Ts此处只计算ply的size bin的可以忽略不计
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v4 += (1 - e[k])*CC[k] * plysize[r][k] * x[r][k];
//			}
//		}
//		v4 = v4 / bw;
//		double tuu = 1;
//		v3 = v3 / tuu;//
//		v5 = b0 - v3 - v4 + 0.33;
//
//		model.add(v5 >= 0);//添加Tb的约束
//
//
//		obj.end();
//
//		//设置模型迭代结束条件
//		IloCplex cplex(model);
//		cplex.setParam(cplex.EpAGap, 0.001);
//		cplex.setParam(cplex.EpGap, 0.001);
//
//		try
//		{
//			if (!cplex.solve())
//			{
//				if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
//					(cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded))
//				{
//					//cout << endl << "No solution - starting Conflict refinement" << endl;
//				}
//				//env.error() << "Failed to optimize LP." << endl;
//				throw(-1);
//			}
//
//		}
//		catch (IloException& e)
//		{
//			//cerr << "Concert exception caught: " << e << endl;
//			//save results
//		}
//		catch (...)
//		{
//			//cerr << "Unknown exception caught" << endl;
//		}
//
//
//		//决策变量输出
//
//		for (int r = 0; r < R; r++)
//		{
//			for (int k = 0; k < K; k++)
//			{
//				if (CC[k] == 0) {
//					xx.push_back(0);
//					continue;
//				}
//				xx.push_back(cplex.getValue(x[r][k]));
//			}
//		}
//
//		for (size_t i = 0; i < K; i++)
//		{
//			if (CC[i] == 0) {
//				ee.push_back(0);
//				continue;
//			}
//			ee.push_back(e[i]);
//		}
//		env.end();
//
//	}
//	else if (scheme == 2) {
//		int e[12] = { 0 };
//		//Td
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v3 += e[k] * CC[k] * tile_C[r][k] * x[r][k];
//			}
//		}
//		double tdd = 3 * (NC*CU[NC] * CU12C);//Td的系数
//		v3 = v3 / tdd;
//
//		//Ts此处只计算ply的size bin的可以忽略不计
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v4 += (1 - e[k])*CC[k] * plysize[r][k] * x[r][k];
//			}
//		}
//		v4 = v4 / bw;
//		double tuu = 1;
//		v3 = v3 / tuu;//
//		v5 = b0 - v3 - v4 + 0.33;
//
//		model.add(v5 >= 0);//添加Tb的约束
//
//
//		obj.end();
//
//		//设置模型迭代结束条件
//		IloCplex cplex(model);
//		cplex.setParam(cplex.EpAGap, 0.001);
//		cplex.setParam(cplex.EpGap, 0.001);
//
//		try
//		{
//			if (!cplex.solve())
//			{
//				if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
//					(cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded))
//				{
//					//cout << endl << "No solution - starting Conflict refinement" << endl;
//				}
//				//env.error() << "Failed to optimize LP." << endl;
//				throw(-1);
//			}
//
//		}
//		catch (IloException& e)
//		{
//			//cerr << "Concert exception caught: " << e << endl;
//			//save results
//		}
//		catch (...)
//		{
//			//cerr << "Unknown exception caught" << endl;
//		}
//
//
//		//决策变量输出
//
//		for (int r = 0; r < R; r++)
//		{
//			for (int k = 0; k < K; k++)
//			{
//				if (CC[k] == 0) {
//					xx.push_back(0);
//					continue;
//				}
//				xx.push_back(cplex.getValue(x[r][k]));
//			}
//		}
//
//		for (size_t i = 0; i < K; i++)
//		{
//			if (CC[i] == 0) {
//				ee.push_back(0);
//				continue;
//			}
//			ee.push_back(e[i]);
//		}
//		env.end();
//	}
//	else if (scheme == 3) {
//
//		IloIntVarArray e(env, 12, 0, 1);//求解变量0,1向量
//		//Td
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v3 += e[k] * CC[k] * tile_C[r][k] * x[r][k];
//			}
//		}
//		double tdd = 3 * (NC*CU[NC] * CU12C);//Td的系数	
//
//		v3 = v3 / tdd;
//
//		//Ts此处只计算ply的size bin的可以忽略不计
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v4 += (1 - e[k])*CC[k] * plysize[r][k] * x[r][k];
//			}
//		}
//		v4 = v4 / bw;
//		double tuu = 1;
//		v3 = v3 / tuu;//
//		v5 = b0 - v3 - v4 + 0.33;
//		model.add(v5 >= 0);//添加Tb的约束
//		obj.end();
//
//		//设置模型迭代结束条件
//		IloCplex cplex(model);
//		cplex.setParam(cplex.EpAGap, 0.001);
//		cplex.setParam(cplex.EpGap, 0.001);
//
//		try
//		{
//			if (!cplex.solve())
//			{
//				if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
//					(cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded))
//				{
//					//cout << endl << "No solution - starting Conflict refinement" << endl;
//				}
//				//env.error() << "Failed to optimize LP." << endl;
//				throw(-1);
//			}
//
//		}
//		catch (IloException& e)
//		{
//			//cerr << "Concert exception caught: " << e << endl;
//			//save results
//		}
//		catch (...)
//		{
//			//cerr << "Unknown exception caught" << endl;
//		}
//
//
//		//决策变量输出
//
//		for (int r = 0; r < R; r++)
//		{
//			for (int k = 0; k < K; k++)
//			{
//				if (CC[k] == 0) {
//					xx.push_back(0);
//					continue;
//				}
//				xx.push_back(cplex.getValue(x[r][k]));
//			}
//		}
//
//		for (size_t i = 0; i < K; i++)
//		{
//			if (CC[i] == 0) {
//				ee.push_back(0);
//				continue;
//			}
//			ee.push_back(cplex.getValue(e[i]));
//		}
//		env.end();
//	}
//}
//
//
////notile QoE
//bool maxQoE_notile(double bandwidth, double Buffersize, vector<int>& xx, vector<int> &ee, vector<int> c, size_t kk, int scheme) {
//
//	IloEnv env;//定义环境
//	IloModel model(env);//定义模型
//	IloExpr obj(env);//表达式
//	IloInt R = 5;//等级数
//	IloInt K = kk;//切块数目
//	IloInt bw = 6000;//带宽
//
//	vector<int> CC(c);//需要传输的块
//
//	IloInt NC = 2;//核心数
//	IloInt C1 = 1;//切块最小计算资源
//	IloInt maxNcore = 16;
//	IloInt CU12C = 1;//
//	vector <double>u(16);//转换效率
//	vector<IloInt>CU(16);//各个核心数计算资源
//
//	for (int i = 0; i < maxNcore; i++)
//	{
//		u[i] = exp(0.02*(-i));
//	}
//
//	for (size_t i = 0; i < maxNcore; i++)
//	{
//		CU[i] = i * u[i] * CU12C;
//	}
//
//	IloInt(*tile_pn)[1] = new IloInt[5][1]();//12x5的矩阵 每个切块的总点数
//	double(*tile_C)[1] = new double[5][1]();//每个切块的C
//	double(*binsize)[1] = new double[5][1]();
//	double(*plysize)[1] = new double[5][1]();
//
//	IloInt b0 = 30;//初始buffer
//	IloInt minp = LONG_MAX;
//	size_t tile_all = 0;//总点数
//	for (size_t k = 0; k < K; k++)
//	{
//		long tmp = pow((k + 1), 1.3) * 380000;
//		for (size_t r = 0; r < R; r++)
//		{
//			tile_pn[r][k] = round(tmp*pow(r + 1, 0.3));
//			if (r == 4) {
//				tile_all += tile_pn[r][k];
//			}
//			if (minp > tile_pn[r][k]) {
//				minp = tile_pn[r][k];
//			}
//		}
//	}
//
//	for (size_t k = 0; k < K; k++)
//	{
//		for (size_t r = 0; r < R; r++)
//		{
//			plysize[r][k] = 0.02734*tile_pn[r][k] / 1024;
//		}
//	}
//
//	for (size_t r = 0; r < R; r++)
//	{
//		for (size_t k = 0; k < K; k++)
//		{
//			tile_C[r][k] = IloLog2(pow(r + 2, 3)*tile_pn[r][k] / minp)*IloLog2(r + 2);
//		}
//	}
//
//	for (size_t r = 0; r < R; r++)
//	{
//		for (size_t k = 0; k < K; k++)
//		{
//			binsize[r][k] = plysize[r][k] * 2 * pow(r + 1, 2) / 800;
//		}
//	}
//
//
//
//	vector<double> w(K);//每个切块的权重
//	for (size_t k = 0; k < K; k++)
//	{
//		size_t s = tile_pn[4][k];
//		double t = s * 1.0 / tile_all;
//		w[k] = t;
//
//	}
//
//	vector<double> dist(K);//每个块的距离
//	for (size_t k = 0; k < K; k++)
//	{
//		dist[k] = k + 1;
//	}
//
//	double(*A)[1] = new double[5][1];//质量权重 5个12个
//	for (size_t r = 0; r < R; r++)
//	{
//		for (size_t k = 0; k < K; k++)
//		{
//			if (CC[k] == 0) {//考虑传输与不传
//				A[r][k] = 0;
//			}
//			else {
//				A[r][k] = (1.0 / dist[k])*(r + 1) * 80 * w[k];
//			}
//		}
//	}
//
//	//系数矩阵
//	IloNumArray b_1(env);
//	for (size_t i = 0; i < 1; i++)
//	{
//		b_1.add(A[0][i]);
//	}
//
//	IloNumArray b_2(env);
//	for (size_t i = 0; i < 1; i++)
//	{
//		b_2.add(A[1][i]);
//	}
//	IloNumArray b_3(env);
//	for (size_t i = 0; i < 1; i++)
//	{
//		b_3.add(A[2][i]);
//	}
//	IloNumArray b_4(env);
//	for (size_t i = 0; i < 1; i++)
//	{
//		b_4.add(A[3][i]);
//	}
//	IloNumArray b_5(env);
//	for (size_t i = 0; i < 1; i++)
//	{
//		b_5.add(A[4][i]);
//	}
//	NumMatrix b(env, 5);//系数矩阵 IloNum就是double 就是doule类型的矩阵
//	for (int i = 0; i < 5; i++)
//	{
//		b[i] = IloNumArray(env, 1);
//	}
//	b[0] = b_1; b[1] = b_2; b[2] = b_3; b[3] = b_4; b[4] = b_5;
//
//	//求解0,1变量矩阵
//	IntVarMatrix x(env, 5);
//	for (int i = 0; i < 5; i++)
//	{
//		x[i] = IloIntVarArray(env, 1, 0, 1);
//	}
//
//
//
//	IloExpr v1(env);//目标函数
//	IloExpr v2(env);//x[1][i]之和为1
//	IloExpr v3(env);//Td的表达式
//	IloExpr v4(env);//Ts的表达式
//	IloExpr v5(env);//Tb的约束
//
//	for (int i = 0; i < 5; i++)
//	{
//		for (int j = 0; j < 1; j++)
//		{
//			v1 += -(x[i][j] * A[i][j]);//求最小 取负数
//		}
//	}
//	model.add(IloMinimize(env, v1));//目标
//
//	for (int k = 0; k < 1; k++)
//	{
//		for (int r = 0; r < 5; r++)
//		{
//			v2 += (x[r][k]);
//		}
//		model.add(v2 == 1);
//		v2.clear();
//	}
//
//
//	if (scheme == 4) {
//		int e[1] = { 1 };
//
//		//Td
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v3 += e[k] * CC[k] * tile_C[r][k] * x[r][k];
//			}
//		}
//		double tdd = 3 * (NC*CU[NC] * CU12C);//Td的系数
//		v3 = v3 / tdd;
//
//		//Ts此处只计算ply的size bin的可以忽略不计
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v4 += (1 - e[k])*CC[k] * plysize[r][k] * x[r][k];
//			}
//		}
//		v4 = v4 / bw;
//		double tuu = 1;
//		v3 = v3 / tuu;//
//		v5 = b0 - v3 - v4 + 0.33;
//
//		model.add(v5 >= 0);//添加Tb的约束
//
//
//		obj.end();
//
//		//设置模型迭代结束条件
//		IloCplex cplex(model);
//		cplex.setParam(cplex.EpAGap, 0.001);
//		cplex.setParam(cplex.EpGap, 0.001);
//
//		try
//		{
//			if (!cplex.solve())
//			{
//				if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
//					(cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded))
//				{
//					//cout << endl << "No solution - starting Conflict refinement" << endl;
//				}
//				//env.error() << "Failed to optimize LP." << endl;
//				throw(-1);
//			}
//
//		}
//		catch (IloException& e)
//		{
//			//cerr << "Concert exception caught: " << e << endl;
//			//save results
//		}
//		catch (...)
//		{
//			//cerr << "Unknown exception caught" << endl;
//		}
//
//
//		//决策变量输出
//
//		for (int r = 0; r < R; r++)
//		{
//			for (int k = 0; k < K; k++)
//			{
//				if (CC[k] == 0) {
//					xx.push_back(0);
//					continue;
//				}
//				xx.push_back(cplex.getValue(x[r][k]));
//			}
//		}
//
//		for (size_t i = 0; i < K; i++)
//		{
//			if (CC[i] == 0) {
//				ee.push_back(0);
//				continue;
//			}
//			ee.push_back(e[i]);
//		}
//		env.end();
//
//	}
//	else if (scheme == 5) {
//		int e[1] = { 0 };
//		//Td
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v3 += e[k] * CC[k] * tile_C[r][k] * x[r][k];
//			}
//		}
//		double tdd = 3 * (NC*CU[NC] * CU12C);//Td的系数
//		v3 = v3 / tdd;
//
//		//Ts此处只计算ply的size bin的可以忽略不计
//		for (size_t k = 0; k < K; k++)
//		{
//			for (size_t r = 0; r < R; r++)
//			{
//				v4 += (1 - e[k])*CC[k] * plysize[r][k] * x[r][k];
//			}
//		}
//		v4 = v4 / bw;
//		double tuu = 1;
//		v3 = v3 / tuu;//
//		v5 = b0 - v3 - v4 + 0.33;
//		model.add(v5 >= 0);//添加Tb的约束
//		obj.end();
//
//		//设置模型迭代结束条件
//		IloCplex cplex(model);
//		cplex.setParam(cplex.EpAGap, 0.001);
//		cplex.setParam(cplex.EpGap, 0.001);
//
//		try
//		{
//			if (!cplex.solve())
//			{
//				if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
//					(cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded))
//				{
//					//cout << endl << "No solution - starting Conflict refinement" << endl;
//				}
//				//env.error() << "Failed to optimize LP." << endl;
//				throw(-1);
//			}
//
//		}
//		catch (IloException& e)
//		{
//			//cerr << "Concert exception caught: " << e << endl;
//			//save results
//		}
//		catch (...)
//		{
//			//cerr << "Unknown exception caught" << endl;
//		}
//
//
//		//决策变量输出
//
//		for (int r = 0; r < R; r++)
//		{
//			for (int k = 0; k < K; k++)
//			{
//				if (CC[k] == 0) {
//					xx.push_back(0);
//					continue;
//				}
//				xx.push_back(cplex.getValue(x[r][k]));
//			}
//		}
//
//		for (size_t i = 0; i < K; i++)
//		{
//			if (CC[i] == 0) {
//				ee.push_back(0);
//				continue;
//			}
//			ee.push_back(e[i]);
//		}
//		env.end();
//	}
//}


//解码线程
void DracoPlayer::decode_thread(int is_tile) {
	//开启解码线程
	if (bin_store.size() > 0) {
		bin_mutex.lock();
		
		
		bin_mutex.unlock();
	}
}


//写入buffer的线程
int DracoPlayer::writebuffer_thread(int is_tile) {
	int *p; int *e;
	p = std::find(gof_tile, gof_tile + 30, 0);
	e = std::find(gof_tile, gof_tile + 30, 1000);
	//有归零的，即该gof融合完毕了
	if (p != gof_tile + 30) {
		buffer_mutex.lock();
		fusion_mutex.lock();
		buffer.push(fusion);//写入buffer
		fusion_mutex.unlock();
		buffer_mutex.unlock();
		fusion.clear();//清空			
		int pos = p - gof_tile + 1;//第几个gof
		*p = 77;//还原
		if (pos == 30) {
			return 0;
		}
		if (is_tile) {
			siggsend();
		}
		else {
			siggsend2();
		}

	}
	else if (e != gof_tile + 30) {//没有需要传输的切块
		*e = 77;
		if (is_tile) {
			siggsend();
		}
		else {
			siggsend2();
		}
	}
	return 1;
}


//读取buffer的线程
void DracoPlayer::readbuffer_thread() {
	QtConcurrent::run([this]() {
		while (true)
		{
			if (buffer.size() > 0) {

				//map<int, PCCPointSet3>::iterator iter;
				buffer_mutex.lock();
				for (auto iter = buffer.front().begin(); iter != buffer.front().end(); iter++) {
					Sleep(33);
					cloud->clear();
					int point_n = iter->second.getPointCount();
					cloud->width = point_n;
					cloud->height = 1;
					//cloud->is_dense = false;
					cloud->points.resize(point_n);

					for (auto i = 0; i < point_n; i++) {
						cloud->points[i].x = iter->second.getPositions()[i].x();
						cloud->points[i].y = iter->second.getPositions()[i].y();
						cloud->points[i].z = iter->second.getPositions()[i].z();
						cloud->points[i].r = iter->second.getColor(i).r();
						cloud->points[i].g = iter->second.getColor(i).g();
						cloud->points[i].b = iter->second.getColor(i).b();
						/*cloud->points[j].x;
						cloud->points[j].y;
						cloud->points[j].z;*/


					}
					mymutex.lock();
					viewer->updatePointCloud(cloud);
					mymutex.unlock();
				}
				buffer.pop();
				buffer_mutex.unlock();

			}
		}
	});
}


//tile方案
void DracoPlayer::download() {
	//下载manifest.xml
	request.setUrl(QUrl(ui.mainfest_url->text() + "?n=" + "mainfest_tile"));
	manager.get(request);

	loadXML("mainfest_tile.xml");//阻塞等待
	viewer->setCameraPosition(250, 500, 1000, 250, 500, 300, 0, 0, 0);//初始视角
	viewer->getCameras(camera);
	fovs.push_back(camera);
	vector<string>::iterator tiles_url_iter;

	scheme = ui.scheme->text().toInt();

	int tile_n = obj.cuboid_n*obj.cuboid_n*obj.cuboid_m;//切块数

	int tile_trans_n = 0;//需要传输的切块数目

	int tmp = 0;

	viewer->getCameras(camera);

	point3 pt_pos; point3 focal_1; point3 focal_2; point3 focal_3; point3 focal_4;

	//验证每个tile是否在fov内
	for (int n = 0; n < tile_n; n++) {
		for (int vertex = 0; vertex < 8; vertex++) {
			if (is_tileinfov(camera.front(), obj.gofs.at(current_gof - 1).tiles.at(n).points[vertex], pt_pos, focal_1, focal_2, focal_3, focal_4)) {
				cv.push_back(1);
				tile_trans_n++;
				break;
			}
			else {
				if (vertex == 7) {//若8个顶点都不在 判断中心点
					if (is_tileinfov(camera.front(), obj.gofs.at(current_gof - 1).tiles.at(n).focal, pt_pos, focal_1, focal_2, focal_3, focal_4))
					{
						cv.push_back(1);
						tile_trans_n++;
						break;
					}
					else cv.push_back(0);
				}
			}
		}
	}

	vector<point3> fovv;

	fovv.push_back(pt_pos); fovv.push_back(focal_1); fovv.push_back(focal_2); fovv.push_back(focal_3); fovv.push_back(focal_4);

	//记录每次的fov
	re_fov.push_back(fovv);

	//每次计算a,e的值
	//maxQoE(bandwidth, buffer.size(), xv, ev, cv, tile_n, scheme);

	//仿真结果
	ev[0] = 0; ev[1] = 1; ev[2] = 1; ev[3] = 1; ev[4] = 0; ev[5] = 1; ev[6] = 0; ev[7] = 0; ev[8] = 0; ev[9] = 0; ev[10] = 0; ev[11] = 0;

	//转换成xv
	int a[5][12];
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 12; j++) {
			a[i][j] = xv[12 * i + j];
		}
	}

	gof_tile[current_gof - 1] = tile_trans_n * (obj.frame_n / obj.gof_n);//乘每个gof包含的帧数
	vector<string> tile_id;//每次请求ply文件和bin文件
	vector<int> frame_id;

	//通过a,e得到对应mpd得到tile地址

	vector<string> tiles_url = tile_choose(xv, ev, mpd, current_gof, tile_id, frame_id);
	int i = 0;
	//tile_parse = gof_tile[current_gof];

	//记录每次的gof
	re_xv.push_back(xv);
	re_cv.push_back(cv);
	re_ev.push_back(ev);
	xv.clear();
	cv.clear();
	ev.clear();

	//初始化fusion
	for (int i = 1; i <= 10; i++) {
		PCCPointSet3 frame;
		fusion.insert(pair<int, PCCPointSet3>(i, frame));
	}

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
	}
	i = 0;
	for (tiles_url_iter = tiles_url.begin(); tiles_url_iter != tiles_url.end(); tiles_url_iter++) {
		QString url = QString::fromStdString(*tiles_url_iter);
		QString reg = ".+(.bin)$";
		QRegExp rx(reg);
		bool match = rx.exactMatch(url);
		string tile_ids = tile_id[i];
		int frame_ids = frame_id[i];
		i++;
		if (!match) {
			request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof)) + "tile_" + QString::fromStdString(tile_ids) + "frame_" + QString::fromStdString(to_string(frame_ids))));
			manager.get(request);
		}
	}





	current_gof++;

	pcl::io::loadPLYFile("D://mpeg_dataset//ply//longdress//Ply//longdress_vox10_1051.ply", *cloud);
	viewer->updatePointCloud(cloud);
	while (true)
	{
		qApp->processEvents();

		viewer->updateText("buffer:" + to_string(buffer.size()), 5, 225, 40, 125, 125, 155, "buffer_size");
		//循环等待，直到加载完毕开始播放
		if (buffer.size() == 30) {//开始播放
			viewer->removeShape("buffer_size");
			int currentgof = 0;
			while (buffer.size() != 0) {
				currentgof++;
				//viewer->setCameraPosition(fovs[currentgof - 1].front().pos[0], fovs[currentgof - 1].front().pos[1], fovs[currentgof - 1].front().pos[2], fovs[currentgof - 1].front().focal[0], fovs[currentgof - 1].front().focal[1], fovs[currentgof - 1].front().focal[2], 0, 0, 0);
				tile_compressed.clear();
				tile_uncompressed.clear();
				tile_compressed = "tile_compressed:";
				tile_uncompressed = "tile_uncompressed:";
				int a[5][12];
				for (int i = 0; i < 5; i++) {
					for (int j = 0; j < 12; j++) {
						a[i][j] = re_xv[currentgof - 1].at(12 * i + j);
					}
				}
				//实时显示切块信息
				for (size_t i = 0; i < 12; i++)
				{
					if (re_cv[currentgof - 1].at(i) == 1) {
						int r = 0;
						if (re_ev[currentgof - 1].at(i) == 1) {

							for (size_t j = 0; j < 5; j++)
							{
								if (a[j][i] == 1) {
									r = j + 1;
									break;
								}
							}
							tile_compressed = tile_compressed + "tile_" + to_string(i) + "(r" + to_string(r) + ")";
						}
						else {
							for (size_t j = 0; j < 5; j++)
							{
								if (a[j][i] == 1) {
									r = j + 1;
									break;
								}
							}
							tile_uncompressed = tile_uncompressed + "tile_" + to_string(i) + "(r" + to_string(r) + ")";
						}
					}
				}
				/*viewer->removeShape("line1");
				viewer->removeShape("line2");
				viewer->removeShape("line3");
				viewer->removeShape("line4");*/
				viewer->updateText(tile_compressed, 5, 35, 25, 125, 15, 55, "tile_compressed");
				viewer->updateText(tile_uncompressed, 5, 55, 25, 145, 75, 15, "tile_uncompressed");
				viewer->updateText("gof:" + to_string(currentgof), 25, 505, 25, 85, 15, 15, "current_gof");
				//viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(re_fov[currentgof - 1].at(0).x, re_fov[currentgof - 1].at(0).y, re_fov[currentgof - 1].at(0).z), pcl::PointXYZ(re_fov[currentgof - 1].at(1).x, re_fov[currentgof - 1].at(1).y, re_fov[currentgof - 1].at(1).z), "line1");
				//viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(re_fov[currentgof - 1].at(0).x, re_fov[currentgof - 1].at(0).y, re_fov[currentgof - 1].at(0).z), pcl::PointXYZ(re_fov[currentgof - 1].at(2).x, re_fov[currentgof - 1].at(2).y, re_fov[currentgof - 1].at(2).z), "line2");
				//viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(re_fov[currentgof - 1].at(0).x, re_fov[currentgof - 1].at(0).y, re_fov[currentgof - 1].at(0).z), pcl::PointXYZ(re_fov[currentgof - 1].at(3).x, re_fov[currentgof - 1].at(3).y, re_fov[currentgof - 1].at(3).z), "line3");
				//viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(pcl::PointXYZ(re_fov[currentgof - 1].at(0).x, re_fov[currentgof - 1].at(0).y, re_fov[currentgof - 1].at(0).z), pcl::PointXYZ(re_fov[currentgof - 1].at(4).x, re_fov[currentgof - 1].at(4).y, re_fov[currentgof - 1].at(4).z), "line4");

				for (auto iter = buffer.front().begin(); iter != buffer.front().end(); iter++) {
					cloud->clear();
					viewer->getCameras(camera);
					int point_n = iter->second.getPointCount();
					cloud->width = point_n;
					cloud->height = 1;
					//cloud->is_dense = false;
					cloud->points.resize(point_n);
					buffer_mutex.lock();
					for (auto i = 0; i < point_n; i++) {
						cloud->points[i].x = iter->second.getPositions()[i].x();
						cloud->points[i].y = iter->second.getPositions()[i].y();
						cloud->points[i].z = iter->second.getPositions()[i].z();
						cloud->points[i].r = iter->second.getColor(i).r();
						cloud->points[i].g = iter->second.getColor(i).g();
						cloud->points[i].b = iter->second.getColor(i).b();
					}
					viewer->updatePointCloud(cloud);
					Sleep(33);
					qApp->processEvents();
					ui.qvtkWidget->update();
				}
				buffer.pop();
				buffer_mutex.unlock();
			}
		}
	}
}

//notile视频
void DracoPlayer::download2() {
	request.setUrl(QUrl(ui.mainfest_url->text() + "?n=" + "mainfest_notile"));
	manager2.get(request);
	cv.push_back(1);
	int tile_n = 1;
	scheme = ui.scheme->text().toInt();
	//maxQoE_notile(bandwidth, buffer.size(), xv, ev, cv, tile_n, scheme);

	loadXML_notile("mainfest_notile.xml");//阻塞等待
	viewer->setCameraPosition(250, 500, 1200, 250, 500, 300, 0, 0, 0);
	viewer->getCameras(camera);
	point3 pt_pos; point3 focal_1; point3 focal_2; point3 focal_3; point3 focal_4;
	vector<point3> fovv;
	fovv.push_back(pt_pos); fovv.push_back(focal_1); fovv.push_back(focal_2); fovv.push_back(focal_3); fovv.push_back(focal_4);
	re_fov.push_back(fovv);
	gof_tile[current_gof - 1] = 10;//乘每个gof包含的帧数
	int frame_ids = 1;

	vector<int> frame_id;
	vector<string>::iterator iter;
	vector<string> urls = tile_choose_notile(xv, ev, mpd, current_gof, frame_id);

	int i = 0;
	for (iter = urls.begin(); iter != urls.end(); iter++) {
		QString url = QString::fromStdString(*iter);
		QString reg = ".+(.bin)$";
		QRegExp rx(reg);
		bool match = rx.exactMatch(url);
		int frame_ids = frame_id[i];
		i++;
		if (match) {
			request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof))));
			manager2.get(request);
		}
		else {
			request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof)) + "frame_" + QString::fromStdString(to_string(frame_ids))));
			manager2.get(request);
		}
	}
	current_gof++;
	pcl::io::loadPLYFile("D://mpeg_dataset//ply//longdress//Ply//longdress_vox10_1051.ply", *cloud);
	viewer->updatePointCloud(cloud);
	while (true)
	{
		qApp->processEvents();
		viewer->updateText("buffer:" + to_string(buffer.size()), 5, 225, 40, 125, 125, 155, "buffer_size");
		if (buffer.size() == 30) {//开始播放
			viewer->removeShape("buffer_size");
			int currentgof = 0;
			while (buffer.size() != 0) {
				//buffer_mutex.lock();		
				currentgof++;
				viewer->updateText("gof:" + to_string(currentgof), 25, 505, 25, 85, 15, 15, "current_gof");
				//viewer->setCameraPosition(fovs[currentgof - 1].front().pos[0], fovs[currentgof - 1].front().pos[1], fovs[currentgof - 1].front().pos[2], fovs[currentgof - 1].front().focal[0], fovs[currentgof - 1].front().focal[1], fovs[currentgof - 1].front().focal[2], 0, 0, 0);			
				for (auto iter = buffer.front().begin(); iter != buffer.front().end(); iter++) {
					cloud->clear();
					int point_n = iter->second.getPointCount();
					cloud->width = point_n;
					cloud->height = 1;
					//cloud->is_dense = false;
					cloud->points.resize(point_n);
					buffer_mutex.lock();
					for (auto i = 0; i < point_n; i++) {
						cloud->points[i].x = iter->second.getPositions()[i].x();
						cloud->points[i].y = iter->second.getPositions()[i].y();
						cloud->points[i].z = iter->second.getPositions()[i].z();
						cloud->points[i].r = iter->second.getColor(i).r();
						cloud->points[i].g = iter->second.getColor(i).g();
						cloud->points[i].b = iter->second.getColor(i).b();
					}
					viewer->updatePointCloud(cloud);
					qApp->processEvents();
					ui.qvtkWidget->update();
				}
				buffer.pop();
				buffer_mutex.unlock();
			}
		}
	}
}


void DracoPlayer::send() {
	int tile_n = obj.cuboid_n*obj.cuboid_n*obj.cuboid_m;
	int tile_trans_n = 0;//需要传输的切块数目
	int tmp = 0;
	int is_tile = 1;
	viewer->getCameras(camera);
	fovs.push_back(camera);

	point3 pt_pos; point3 focal_1; point3 focal_2; point3 focal_3; point3 focal_4;

	//检测每个tile的顶点是否在fov内
	for (int n = 0; n < tile_n; n++) {
		for (int vertex = 0; vertex < 8; vertex++) {
			if (is_tileinfov(camera.front(), obj.gofs.at(current_gof - 1).tiles.at(n).points[vertex], pt_pos, focal_1, focal_2, focal_3, focal_4)) {
				cv.push_back(1);
				tile_trans_n++;
				break;
			}
			else {
				if (vertex == 7) {//若8个顶点都不在 判断中心点
					if (is_tileinfov(camera.front(), obj.gofs.at(current_gof - 1).tiles.at(n).focal, pt_pos, focal_1, focal_2, focal_3, focal_4))
					{
						cv.push_back(1);
						tile_trans_n++;
						break;
					}
					else cv.push_back(0);
				}
			}
		}
	}

	vector<point3> fovv;
	fovv.push_back(pt_pos); fovv.push_back(focal_1); fovv.push_back(focal_2); fovv.push_back(focal_3); fovv.push_back(focal_4);
	re_fov.push_back(fovv);
	vector<string>::iterator tiles_url_iter;
	//每次计算a,e的值
	//maxQoE(bandwidth, buffer.size(), xv, ev, cv, tile_n, scheme);
	ev[0] = 0; ev[1] = 1; ev[2] = 1; ev[3] = 1; ev[4] = 0; ev[5] = 0; ev[6] = 0; ev[7] = 0; ev[8] = 1; ev[9] = 1; ev[10] = 0; ev[11] = 0;
	int a[5][12];
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 12; j++) {
			a[i][j] = xv[12 * i + j];
		}
	}

	if (tile_trans_n == 0) {
		gof_tile[current_gof - 1] = 10000;
		current_gof++;
		QtConcurrent::run(&threadpool, this, &DracoPlayer::writebuffer_thread, is_tile);
		return;
	}
	//xv ev转化为 a e
	gof_tile[current_gof - 1] = tile_trans_n * 10;//乘每个gof包含的帧数

	vector<string> tile_id;//每次请求ply文件和bin文件
	vector<int> frame_id;
	;
	//通过a,e得到对应mpd得到tile地址

	vector<string> tiles_url = tile_choose(xv, ev, mpd, current_gof, tile_id, frame_id);

	//if (current_gof % 3 == 1) {

	//	ev[1] = 1; ev[2] = 1;

	//	for (size_t i = 0; i < 5; i++)
	//	{
	//		for (size_t j = 0; j < 12; j++)
	//		{
	//			if (xv[12 * i + j] == 1) {

	//				if (j == 1) {
	//					xv[j] = 1;
	//					xv[12 * i + j] = 0;
	//				}
	//				else if (j == 3) {
	//					xv[12 * 3 + j] = 1;
	//					xv[12 * i + j] = 0;

	//				}
	//				else if (j == 6) {
	//					xv[12 * 1 + j] = 1;
	//					xv[12 * i + j] = 0;
	//				}
	//			}
	//		}
	//	}

	//}
	//else if (current_gof % 3 == 2) {
	//	ev[2] = 1; ev[4] = 1;

	//	for (size_t i = 0; i < 5; i++)
	//	{
	//		for (size_t j = 0; j < 12; j++)
	//		{
	//			if (xv[12 * i + j] == 1) {

	//				if (j == 2) {
	//					xv[12 * 1 + j] = 1;
	//					xv[12 * i + j] = 0;
	//				}
	//				else if (j == 7) {
	//					xv[12 * 3 + j] = 1;
	//					xv[12 * i + j] = 0;

	//				}
	//				else if (j == 9) {
	//					xv[12 * 3 + j] = 1;
	//					xv[12 * i + j] = 0;
	//				}
	//			}
	//		}
	//	}
	//}
	//else {
	//	ev[3] = 1;
	//	for (size_t i = 0; i < 5; i++)
	//	{
	//		for (size_t j = 0; j < 12; j++)
	//		{
	//			if (xv[12 * i + j] == 1) {

	//				if (j == 1) {
	//					xv[12 * 0 + j] = 1;
	//					xv[12 * i + j] = 0;
	//				}
	//				else if (j == 1) {
	//					xv[12 * 3 + j] = 1;
	//					xv[12 * i + j] = 0;

	//				}
	//				else if (j == 6) {
	//					xv[12 * 2 + j] = 1;
	//					xv[12 * i + j] = 0;
	//				}
	//			}
	//		}
	//	}


	//}

	re_xv.push_back(xv);
	re_cv.push_back(cv);
	re_ev.push_back(ev);
	xv.clear();
	cv.clear();
	ev.clear();
	int i = 0;

	for (int i = 1; i <= 10; i++) {
		PCCPointSet3 frame;
		fusion.insert(pair<int, PCCPointSet3>(i, frame));
	}

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
	}
	i = 0;
	for (tiles_url_iter = tiles_url.begin(); tiles_url_iter != tiles_url.end(); tiles_url_iter++) {
		QString url = QString::fromStdString(*tiles_url_iter);
		QString reg = ".+(.bin)$";
		QRegExp rx(reg);
		bool match = rx.exactMatch(url);
		string tile_ids = tile_id[i];
		int frame_ids = frame_id[i];
		i++;
		if (!match) {
			request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof)) + "tile_" + QString::fromStdString(tile_ids) + "frame_" + QString::fromStdString(to_string(frame_ids))));
			manager.get(request);
		}
	}
	current_gof++;
}


void DracoPlayer::send2() {

	viewer->getCameras(camera);
	fovs.push_back(camera);
	point3 pt_pos; point3 focal_1; point3 focal_2; point3 focal_3; point3 focal_4;
	vector<point3> fovv;
	fovv.push_back(pt_pos); fovv.push_back(focal_1); fovv.push_back(focal_2); fovv.push_back(focal_3); fovv.push_back(focal_4);
	re_fov.push_back(fovv);
	//xv ev转化为 a e
	gof_tile[current_gof - 1] = 10;//乘每个gof包含的帧数
	vector<int> frame_id;
	int frame_ids = 1;
	vector<string>::iterator iter;
	vector<string> urls = tile_choose_notile(xv, ev, mpd, current_gof, frame_id);
	int i = 0;
	for (iter = urls.begin(); iter != urls.end(); iter++) {
		QString url = QString::fromStdString(*iter);
		QString reg = ".+(.bin)$";
		QRegExp rx(reg);
		bool match = rx.exactMatch(url);
		int frame_ids = frame_id[i];
		i++;
		if (match) {
			request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof))));
			manager2.get(request);
		}
		else {
			request.setUrl(QUrl(url + "?n=" + "gof_" + QString::fromStdString(to_string(current_gof)) + "frame_" + QString::fromStdString(to_string(frame_ids))));
			manager2.get(request);
		}
	}
	current_gof++;
}


//接收
void DracoPlayer::replyFinished(QNetworkReply *reply)
{
	if (reply->error() == QNetworkReply::NoError)
	{
		int is_tile = 1;
		QByteArray bytes = reply->readAll();
		QString fileName;
		if (reply->hasRawHeader(QString("Content-Disposition").toUtf8()))
		{
			fileName = reply->rawHeader(QString("Content-Disposition").toUtf8());
		}
		//如果是xml文件
		QString reg = ".+(.xml)$";
		QRegExp rxp(reg);
		bool match = rxp.exactMatch(fileName);
		if (match) {
			char* buf;
			buf = bytes.data();
			ofstream outfile("mainfest_tile.xml");
			outfile << buf;
			outfile.close();
			return;
		}
		//分类匹配
		reg = ".+(.bin)$";
		QRegExp rx(reg);
		match = rx.exactMatch(fileName);

		if (match) {
			//如果是bin文件
			QRegExp rx(QString("(\\d+)tile_(\\d+)"));
			int pos = fileName.indexOf(rx);
			QString gof_id = rx.cap(1);
			QString tile_id = rx.cap(2);
			QString gof_tiled = gof_id + "_" + tile_id;
			vector<uint8_t> temp(bytes.begin(), bytes.end());
			bin_store.insert(pair<QString, vector<uint8_t>>(gof_tiled, temp));//写入解压区
			QtConcurrent::run(&threadpool, this, &DracoPlayer::decode_thread, is_tile);
		}

		//如果是ply文件,无序,读取frame_id
		else {
			QRegExp rx(QString("(\\d+)tile_(\\d+)frame_(\\d+)"));
			int pos = fileName.indexOf(rx);
			int gof_id = rx.cap(1).toInt();
			int tile_id = rx.cap(2).toInt();
			int frame_id = rx.cap(3).toInt();
			int offset = bytes.lastIndexOf('r');
			string ply = bytes.mid(offset + 3, bytes.size());
			map<int, PCCPointSet3>::iterator iter;
			//fusion_mutex.lock();
			iter = fusion.find(frame_id);
			//fusion_mutex.unlock();
			if (iter != fusion.end()) {
				QtConcurrent::run(&threadpool, this, &DracoPlayer::frame2xyzrgb_add, ply, &(iter->second), gof_id, frame_id);
			}
			else {
				//frame2xyzrgb(ply, gof_id,frame_id,is_tile);
				QtConcurrent::run(&threadpool, this, &DracoPlayer::frame2xyzrgb, ply, gof_id, frame_id, is_tile);
			}
		}
		reply->deleteLater();
	}
	else
	{
		//noreply
	}
}


//接受原始
void DracoPlayer::replyFinished2(QNetworkReply *reply)
{
	if (reply->error() == QNetworkReply::NoError)
	{
		QByteArray bytes = reply->readAll();
		QString fileName;
		if (reply->hasRawHeader(QString("Content-Disposition").toUtf8()))
		{
			fileName = reply->rawHeader(QString("Content-Disposition").toUtf8());
		}

		int is_tile = 0;
		// 如果是xml文件
		QString reg = ".+(.xml)$";
		QRegExp rxp(reg);
		bool match = rxp.exactMatch(fileName);
		if (match) {
			char* buf;
			buf = bytes.data();
			ofstream outfile("mainfest_notile.xml");
			outfile << buf;
			outfile.close();
			return;
		}
		//分类匹配

		reg = ".+(.bin)$";
		QRegExp rx(reg);
		match = rx.exactMatch(fileName);
		if (match) {
			//如果是bin文件
			QRegExp rx(QString("(\\d+)"));
			int pos = fileName.indexOf(rx);
			QString gof_id = rx.cap(1);
			QString gof_tiled = gof_id + "_1";
			vector<uint8_t> temp(bytes.begin(), bytes.end());
			bin_store.insert(pair<QString, vector<uint8_t>>(gof_tiled, temp));//写入解压区
			QtConcurrent::run(&threadpool, this, &DracoPlayer::decode_thread, is_tile);
		}
		else {
			//如果是ply文件,无序,读取frame_id	
			QRegExp rx(QString("(\\d+)frame_(\\d+)"));
			int pos = fileName.indexOf(rx);
			int gof_id = rx.cap(1).toInt();
			int frame_id = rx.cap(2).toInt();
			int offset = bytes.lastIndexOf('r');
			string ply = bytes.mid(offset + 3, bytes.size());
			map<int, PCCPointSet3>::iterator iter;
			iter = fusion.find(frame_id);
			//第一次新增
			fusion_mutex.lock();
			QtConcurrent::run(&threadpool, this, &DracoPlayer::frame2xyzrgb, ply, gof_id, frame_id, is_tile);
			//frame2xyzrgb(ply, gof_id, frame_id);
			fusion_mutex.unlock();
		}

		reply->deleteLater();
	}
	else
	{
		//noreply
	}
}
