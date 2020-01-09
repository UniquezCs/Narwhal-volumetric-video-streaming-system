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
#include "my3d.h"
#define PI acos(-1)
#ifndef abcde//ifndef做预处理，进行条件编译
#include <ilcplex/ilocplex.h>
typedef IloArray<IloIntVarArray> IntVarMatrix;
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloIntArray> IntMatrix;
typedef IloArray<IloNumArray> NumMatrix;
//定义各种数组
#endif
ILOSTLBEGIN

using namespace pcc;
using namespace std;

struct frame_tile {
	int tile_id;
	int point_n;
	double size;//kb
	string url;

};

struct  frame_st
{
	int frame_id;
	int point_n;
	int r;
	vector<frame_tile> tiles;
};

struct tile_gof {
	int tile_id;
	int r;
	double binsize;
	double CR;
	string url;
	point3 points[8];
	point3 focal;
};

struct gof {
	int minx;//包围的cuboid
	int miny;
	int minz;
	int maxx;
	int maxy;
	int maxz;
	int gof_id;
	vector<tile_gof> tiles;
	vector<frame_st> frames;

};

struct object {
	int frame_n;
	int gof_n;
	int cuboid_n;//切块方式nxnxm
	int cuboid_m;
	int flag;
	string baseurl;
	vector<gof> gofs;

};

QString stuff = ".pcd";
QString stuff_ply = ".ply";
int buffer_size;
map<string, string>mpd;
object obj;//点云对象
vector<pcl::visualization::Camera> camera;

//模拟hololens2的fov
double fov_y = 29;
double fov_x = 43;
double fovx2fovy = tan(fov_y / 2 * PI / 180) / tan(fov_x / 2 * PI / 180);

QMutex mymutex(QMutex::Recursive);
QMutex fusion_mutex(QMutex::Recursive);
QMutex buffer_mutex(QMutex::Recursive);
vector<vector<float>> xyzs;//一帧的所有坐标（包含坐标中的坐标点）

typedef pcl::PointXYZRGB PointT;
double start;
vector<vector<int>> x;


queue<map<int, PCCPointSet3>>buffer;
map<int, PCCPointSet3> fusion;//存储5秒 frame_id->
int tile_parse;
int gof_tile[15] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};//验证是否融合完成 gof_id->tilenumber

map<QString, vector<uint8_t>> bin_store;//(tile_id,gof_id）->bin

//vector<double> bandwidth;
int bandwidth = 1000;
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
	TiXmlDocument doc;
	while (true) {
		if (!doc.LoadFile(filename))
		{
			continue;
		}
		else {
			break;

		}

	}
	

	TiXmlElement *root = doc.FirstChildElement();
	if (root == NULL)
	{
		doc.Clear();
		return false;
	}
	else
	{
		obj.frame_n =stoi( root->Attribute("frames"));
		obj.gof_n = stoi(root->Attribute("GOF"));
		obj.cuboid_n = stoi(root->Attribute("n"));
		obj.cuboid_m = stoi(root->Attribute("m"));
		obj.flag = stoi(root->Attribute("flag"));
		obj.baseurl = root->Attribute("BaseURL");
		
	}

	// 遍历子节点
	for (TiXmlElement *elem = root->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
	{
		// 获取元素名
		gof gof_tmp;

		gof_tmp.gof_id =stoi(elem->Attribute("id"));
		gof_tmp.minx = stoi(elem->Attribute("minx"));
		gof_tmp.miny = stoi(elem->Attribute("miny"));
		gof_tmp.minz = stoi(elem->Attribute("minz"));
		gof_tmp.maxx = stoi(elem->Attribute("maxx"));
		gof_tmp.maxy = stoi(elem->Attribute("maxy"));
		gof_tmp.maxz = stoi(elem->Attribute("maxz"));

			//判断adp类型
			for (TiXmlElement *AdaptationSet = elem->FirstChildElement(); AdaptationSet != NULL; AdaptationSet = AdaptationSet->NextSiblingElement()) {
				string adp_id = AdaptationSet->Attribute("id");
				// 未压缩的点云信息
				if (adp_id._Equal("1")) {
					for (TiXmlElement *Rep = AdaptationSet->FirstChildElement(); Rep != NULL; Rep = Rep->NextSiblingElement()) {
						string r_level = Rep->Attribute("id");
						for (TiXmlElement *frame = Rep->FirstChildElement(); frame != NULL; frame = frame->NextSiblingElement()) {
							frame_st frame_tmp;
							frame_tmp.frame_id = stoi( frame->Attribute("id"));
							frame_tmp.point_n = stoi(frame->Attribute("PointN"));
							frame_tmp.r = stoi(r_level);
							
							for (TiXmlElement *tile = frame->FirstChildElement(); tile != NULL; tile = tile->NextSiblingElement()) {

								frame_tile frame_tile_tmp;
								frame_tile_tmp.point_n= stoi(tile->Attribute("PointN"));
								frame_tile_tmp.size= stod(tile->Attribute("size"));
								frame_tile_tmp.tile_id= stoi(tile->Attribute("id"));
								
								TiXmlNode *tile_url = tile->FirstChild();
								frame_tile_tmp.url = obj.baseurl+tile_url->ToText()->Value();
								frame_tmp.tiles.push_back(frame_tile_tmp);

								mpd.insert(pair<string, string>("gof" + to_string(gof_tmp.gof_id) + "adp" + adp_id + "r" + r_level + "frame" + to_string(frame_tmp.frame_id) + "tile" + to_string(frame_tile_tmp.tile_id) + "url", frame_tile_tmp.url));
								
							}
							gof_tmp.frames.push_back(frame_tmp);
						}
					}
				}

				//压缩后的点云信息
				else {
					for (TiXmlElement *tile = AdaptationSet->FirstChildElement(); tile != NULL; tile = tile->NextSiblingElement()) {
							
						for (TiXmlElement *rep = tile->FirstChildElement(); rep != NULL; rep = rep->NextSiblingElement()) {
							tile_gof tile_gof_tmp;
							tile_gof_tmp.tile_id = stoi(tile->Attribute("id"));
							tile_gof_tmp.binsize= stod(rep->Attribute("binsize"));
							tile_gof_tmp.CR= stod(rep->Attribute("CR"));
							tile_gof_tmp.r= stoi(rep->Attribute("id"));					
							TiXmlNode *tile_url = rep->FirstChild();
							tile_gof_tmp.url = obj.baseurl+tile_url->ToText()->Value();
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
						//tiles.push_back(tile_seg[k]);
						double focalx=0;
						double focaly=0;
						double focalz=0;

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

PCLVisualizer::PCLVisualizer(QWidget *parent)
{

	ui.setupUi(this);
	threadpool.setMaxThreadCount(4);
	//初始化
	initialVtkWidget();

	ui.mainfest_url->setText("http://localhost/eghost_loot/mainfest.xml");
	connect(ui.button_play, SIGNAL(clicked()), this, SLOT(download()));//播放按钮的槽
	connect(this, SIGNAL(sigrender()), this, SLOT(slotrender()));
	connect(&manager, SIGNAL(finished(QNetworkReply *)), this, SLOT(replyFinished(QNetworkReply *)));	//下载完成后的槽
}

void PCLVisualizer::slotrender() {
	//qApp->processEvents();//刷新,代替spin()
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
	
	viewer->addText("", 10, 10, "tile_parse");
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
	size_t n_point = count(ply.begin(), ply.end(), '\r');//点个数
	size_t offset = 0;
	string point;
	for (size_t i = 0; i < n_point; i++) {
		qApp->processEvents();
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

vector<string> tile_choose(int a[][12], int *e, map<string,string> mdp,const int current_gof,vector<string> &tile_id,vector<int> &frame_id) {

	vector<string> tiles;
	map<int, vector<int>> tile_info;
	map<int, vector<int>>::iterator iter;
	map<int, vector<string>> tiles_url;
	vector<string> urls;
	vector<string> request;
	int r;
	vector<int> r_form;
	//x,e转化成tile_info
	for (int i = 0; i < 12; i++) {
		r = 0;
		for (int j = 0; j < 5; j++) {
			int s = a[j][i];
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



	//tile_info转化为mpd中的键
		for (iter = tile_info.begin(); iter != tile_info.end(); iter++) {
			int tile_id=iter->first+1;
			int form = iter->second.at(1);
			int r=iter->second.at(0);
			
			//bin形式
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
	
	//匹配mpd中键对应值
	map<int, vector<string>>::iterator tiles_url_iter;//tile)id->simpleurl
	map<string, string>::iterator mpd_iter;// simleurl->url
	for (tiles_url_iter = tiles_url.begin(); tiles_url_iter != tiles_url.end(); tiles_url_iter++) {
		vector<string>::iterator tile_iter;//simpleurl
		for (tile_iter = tiles_url_iter->second.begin(); tile_iter != tiles_url_iter->second.end(); tile_iter++) {
			mpd_iter = mpd.find(*tile_iter);
				//有对应项
			if (mpd_iter!=mpd.end()) {

				string tile_url = mpd_iter->second;
				tile_id.push_back(to_string(tiles_url_iter->first));
				request.push_back(tile_url);
			}
		}	
	}
		return request;
}

//判断块的顶点和中心点是否在fov区域内
bool is_tileinfov(pcl::visualization::Camera ca, point3 p) {

	point3 pt_pos; pt_pos.x = ca.pos[0]; pt_pos.y = ca.pos[1]; pt_pos.z = ca.pos[2];
	point3 pt_focal; pt_focal.x = ca.focal[0]; pt_focal.y = ca.focal[1]; pt_focal.z = ca.focal[2];

	point3 vec_fp = subt(pt_focal, pt_pos);//视角向量

	double l_view = distance(pt_pos, pt_focal);//视角向量距离;

	double l_vertical = l_view * tan(fov_y / 2 * PI / 180);//竖直向量长

	point3 vec_vertical; vec_vertical.x = ca.view[0]; vec_vertical.y = ca.view[1]; vec_vertical.z = ca.view[2];//竖直向量

	vec_vertical.x = l_vertical / (vlen(vec_vertical))*vec_vertical.x; vec_vertical.y = l_vertical / (vlen(vec_vertical))*vec_vertical.y; vec_vertical.z = l_vertical / (vlen(vec_vertical))*vec_vertical.z;//修正长度

	double l_horizon = l_vertical / fovx2fovy;

	point3 vec_horizon = OuterProduct(vec_fp, vec_vertical);//水平向量

	vec_horizon.x = l_horizon / (vlen(vec_horizon))*vec_horizon.x; vec_horizon.y = l_horizon / (vlen(vec_horizon))*vec_horizon.y; vec_horizon.z = l_horizon / (vlen(vec_horizon))*vec_horizon.z;//修正长度

	point3 vec_1 = addv(vec_vertical, vec_horizon);
	point3 vec_2 = inverse(vec_1);
	point3 vec_3 = addv(inverse(vec_vertical), vec_horizon);
	point3 vec_4 = inverse(vec_3);

	point3 focal_1 = addv(pt_focal, vec_1);//四条棱与中心面交点
	point3 focal_2 = addv(pt_focal, vec_2);
	point3 focal_3 = addv(pt_focal, vec_3);
	point3 focal_4 = addv(pt_focal, vec_4);

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

bool maxQoE(double bandwidth, double Buffersize,vector<int>& xx,int *ee,vector<int> c,size_t kk) {

	IloEnv env;//定义环境
	IloModel model(env);//定义模型
	IloExpr obj(env);//表达式
	IloInt R = 5;//等级数
	IloInt K = kk;//切块数目
	IloInt bw = 73.75;//带宽

	vector<int> CC(c);//需要传输的块

	IloInt NC = 2;//核心数
	IloInt C1 = 1;//切块最小计算资源
	IloInt maxNcore = 16;
	IloInt CU12C = 1;//
	vector <double>u(16);//转换效率
	vector<IloInt>CU(16);//各个核心数计算资源
	for (int i = 0; i < maxNcore; i++)
	{
		u[i] = exp(0.02*(-i));
	}

	for (size_t i = 0; i < maxNcore; i++)
	{
		CU[i] = i * u[i] * CU12C;
	}
	IloInt(*tile_pn)[12] = new IloInt[5][12]();//12x5的矩阵 每个切块的总点数
	double(*tile_C)[12] = new double[5][12]();//每个切块的C
	double(*binsize)[12] = new double[5][12]();
	double(*plysize)[12] = new double[5][12]();

	IloInt b0 = 4;//初始buffer
	IloInt minp = LONG_MAX;
	size_t tile_all = 0;//总点数
	for (size_t k = 0; k < K; k++)
	{
		long tmp = pow((k + 1), 1.3) * 380000;
		for (size_t r = 0; r < R; r++)
		{

			tile_pn[r][k] = round(tmp*pow(r + 1, 0.3));
			if (r == 4) {
				tile_all += tile_pn[r][k];

			}
			if (minp > tile_pn[r][k]) {
				minp = tile_pn[r][k];
			}
		}
	}

	for (size_t k = 0; k < K; k++)
	{
		for (size_t r = 0; r < R; r++)
		{
			plysize[r][k] = 0.02734*tile_pn[r][k] / 1024;
		}
	}


	for (size_t r = 0; r < R; r++)
	{
		for (size_t k = 0; k < K; k++)
		{
			tile_C[r][k] = IloLog2(pow(r + 2, 3)*tile_pn[r][k] / minp)*IloLog2(r + 2);
		}
	}

	for (size_t r = 0; r < R; r++)
	{
		for (size_t k = 0; k < K; k++)
		{
			binsize[r][k] = plysize[r][k] * 2 * pow(r + 1, 2) / 800;
		}
	}



	vector<double> w(K);//每个切块的权重
	for (size_t k = 0; k < K; k++)
	{
		size_t s = tile_pn[4][k];
		double t = s * 1.0 / tile_all;
		w[k] = t;

	}

	vector<double> dist(K);//每个块的距离
	for (size_t k = 0; k < K; k++)
	{
		dist[k] = k + 1;
	}

	double(*A)[12] = new double[5][12];//质量权重 5个12个
	for (size_t r = 0; r < R; r++)
	{
		for (size_t k = 0; k < K; k++)
		{
			if (CC[k] == 0) {//考虑传输与不传
				A[r][k] = 0;
			}
			else {
				A[r][k] = (1.0 / dist[k])*(r + 1) * 80 * w[k];
			}
		}
	}

	//系数矩阵
	IloNumArray b_1(env);
	for (size_t i = 0; i < 12; i++)
	{
		b_1.add(A[0][i]);
	}

	IloNumArray b_2(env);
	for (size_t i = 0; i < 12; i++)
	{
		b_2.add(A[1][i]);
	}
	IloNumArray b_3(env);
	for (size_t i = 0; i < 12; i++)
	{
		b_3.add(A[2][i]);
	}
	IloNumArray b_4(env);
	for (size_t i = 0; i < 12; i++)
	{
		b_4.add(A[3][i]);
	}
	IloNumArray b_5(env);
	for (size_t i = 0; i < 12; i++)
	{
		b_5.add(A[4][i]);
	}
	NumMatrix b(env, 5);//系数矩阵 IloNum就是double 就是doule类型的矩阵
	for (int i = 0; i < 5; i++)
	{
		b[i] = IloNumArray(env, 12);
	}
	b[0] = b_1; b[1] = b_2; b[2] = b_3; b[3] = b_4; b[4] = b_5;




	IntVarMatrix x(env, 5);//求解0,1变量矩阵
	IloIntVarArray e(env, 12, 0, 1);//求解变量0,1向量


	for (int i = 0; i < 5; i++)
	{
		x[i] = IloIntVarArray(env, 12, 0, 1);
	}


	IloExpr v1(env);//目标函数
	IloExpr v2(env);//x[1][i]之和为1
	IloExpr v3(env);//Td的表达式
	IloExpr v4(env);//Ts的表达式
	IloExpr v5(env);//Tb的约束
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			v1 += -(x[i][j] * b[i][j]);//求最小 取负数
		}
	}

	model.add(IloMinimize(env, v1));//目标

	for (int k = 0; k < 12; k++)
	{
		for (int r = 0; r < 5; r++)
		{
			v2 += (x[r][k]);
		}
		model.add(v2 == 1);
		v2.clear();
	}
	cout << model;
	//Td
	for (size_t k = 0; k < K; k++)
	{

		for (size_t r = 0; r < R; r++)
		{
			v3 += e[k] * CC[k] * tile_C[r][k] * x[r][k];
		}
	}
	double tdd = 3 * (NC*CU[NC] * CU12C);//Td的系数
	v3 = v3 / tdd;

	//Ts此处只计算ply的size bin的可以忽略不计
	for (size_t k = 0; k < K; k++)
	{
		for (size_t r = 0; r < R; r++)
		{
			v4 += (1 - e[k])*CC[k] * plysize[r][k] * x[r][k];
		}
	}

	v4 = v4 / bw;

	double tuu = 1;
	v3 = v3 / tuu;//
	v5 = b0 - v3 - v4 + 0.33;

	model.add(v5 >= 0);//添加Tb的约束

	cout << model;

	obj.end();

	//设置模型迭代结束条件
	IloCplex cplex(model);
	cplex.setParam(cplex.EpAGap, 0.001);
	cplex.setParam(cplex.EpGap, 0.001);


	try
	{
		if (!cplex.solve())
		{
			if ((cplex.getStatus() == IloAlgorithm::Infeasible) ||
				(cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded))
			{
				//cout << endl << "No solution - starting Conflict refinement" << endl;
			}
			//env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

	}
	catch (IloException& e)
	{
		//cerr << "Concert exception caught: " << e << endl;
		//save results
	}
	catch (...)
	{
		//cerr << "Unknown exception caught" << endl;
	}


	//决策变量输出



	for (int r = 0; r < R; r++)
	{
		for (int k = 0; k < K; k++)
		{
			if(CC[k] == 0){
				xx.push_back(0);
				continue;
			}
			xx.push_back(cplex.getValue(x[r][k]));
		}
	}

	for (size_t i = 0; i < K; i++)
	{
		if (CC[i] == 0) {
			*(ee + i) = 0;
			continue;
		}
		*(ee + i) = cplex.getValue(e[i]);
	}
	env.end();
}

void PCLVisualizer::decode_thread() {
	//开启解码线程
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
					int ret = decoder.decode(bitstream, context, reconstructs);//关键
					if (ret) { return ret; }
					vector<PCCPointSet3>::iterator iter;
					map<int, PCCPointSet3>::iterator itermap;
					for (iter = reconstructs.begin(); iter != reconstructs.end(); iter++) {
						int frame_id = frameNumber++ - 999;
						//itermap = fusion[gof_id - 1].find(frame_id);
						itermap = fusion.find(frame_id);
						if (itermap != fusion.end()) {
							//已有,一个个加
							vector<PCCPoint3D> points = iter->getPositions();						
							vector<PCCPoint3D>::iterator iterpoint;
							int i = 0;
							for (iterpoint = points.begin(); iterpoint != points.end(); iterpoint++) {
								fusion_mutex.lock();
								itermap->second.addPoint(*iterpoint,iter->getColor(i++));
								fusion_mutex.unlock();
							}
							gof_tile[gof_id - 1]--;
						}
						else {
							//第一次
							fusion_mutex.lock();
							fusion.insert(pair<int, PCCPointSet3>(frame_id, *iter));
							fusion_mutex.unlock();
							gof_tile[gof_id - 1]--;
						}
					}
				}
				bin_store.erase(iterbin);
			}
		}
	});
}

void PCLVisualizer::writebuffer_thread() {
	QtConcurrent::run(&threadpool,[this]() {
		int i = 0;
		viewer->addCoordinateSystem(1.0);
		while (true) {
			int *p;
			p = std::find(gof_tile, gof_tile + 15, 0);	
			if (p != gof_tile + 15) {
				buffer_mutex.lock();
				buffer.push(fusion);//写入buffer
				buffer_mutex.unlock();
				fusion.clear();//清空
				int pos = p - gof_tile;//第几个gof
				*p = 1;//还原		
			}		
		}
	});
}

void PCLVisualizer::readbuffer_thread() {
	QtConcurrent::run(&threadpool, [this]() {
		while (true)
		{
			if (buffer.size() > 0) {
				map<int, PCCPointSet3>::iterator iter;
				buffer_mutex.lock();
				for (iter = buffer.front().begin(); iter != buffer.front().end(); iter++) {
					Sleep(33);
					cloud->clear();
					int point_n = iter->second.getPointCount();
					cloud->width = point_n;
					cloud->height = 1;
					cloud->is_dense = false;
					cloud->points.resize(point_n);
					
					for (int i = 0; i < point_n; i++) {
						cloud->points[i].x = iter->second[i].x();
						cloud->points[i].y = iter->second[i].y();
						cloud->points[i].z = iter->second[i].z();
						cloud->points[i].r = iter->second[i].r();
						cloud->points[i].g = iter->second[i].g();
						cloud->points[i].b = iter->second[i].b();
					}
					//mymutex.lock();
					//
					
					//mymutex.unlock();
				}
				buffer_mutex.unlock();
				buffer.pop();
			}
		}
	});
}

//play按钮(向服务器下载请求)
void PCLVisualizer::download() {
	//下载manifest.xml
	
	request.setUrl(QUrl(ui.mainfest_url->text()+ "?n="+"mainfest"));
	manager.get(request); start = GetTickCount();

	loadXML("mainfest.xml");//阻塞等待
	//初始摄像头


	viewer->setCameraPosition(208, 500, 600, 208, 500, 300, 0, 0, 0);//pos,focal,
	//输入tiles,buffer,Bw, 得到x e
	viewer->addCoordinateSystem(100);
	vector<string>::iterator tiles_url_iter;
	int tile_n = obj.cuboid_n*obj.cuboid_n*obj.cuboid_m;//切块数
	writebuffer_thread();//开启写入buffer的线程
	decode_thread();//解码线程
	readbuffer_thread();//读取buffer的线程

	obj.gof_n = 1;//测试！！！！！！！
	for (int current_gof = 1; current_gof <= obj.gof_n; current_gof++) {
		
		int tile_trans_n=0;//需要传输的切块数目
		int *e = new int[tile_n];
		vector<vector<int>> xv;//码率等级
		vector<int> ev;//传输形式
		vector<int> cv;//是否传输
		vector<int> x;
		int tmp=0;
		viewer->getCameras(camera);

		//检测每个tile的顶点是否在fov内
		for (int n = 0; n < tile_n; n++) {
			for (int vertex = 0; vertex < 8; vertex++) {
				if (is_tileinfov(camera.front(), obj.gofs.at(current_gof - 1).tiles.at(n).points[vertex])) {
					cv.push_back(1);
					tile_trans_n++;
					break;
				}
				else {
					if (vertex == 7) {//若8个顶点都不在 判断中心点
						 if (is_tileinfov(camera.front(), obj.gofs.at(current_gof - 1).tiles.at(n).focal))
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

		//每次计算a,e的值
		maxQoE(bandwidth, buffer.size(), x, e, cv, tile_n);

		int a[5][12];
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 12; j++) {
				
				a[i][j] = x[12 * i + j];

			}
		}
		//xv ev转化为 a e
		gof_tile[current_gof - 1] = tile_trans_n*10;//乘每个gof包含的帧数
		vector<string> tile_id;//每次请求ply文件和bin文件
		vector<int> frame_id;
		start = GetTickCount();
		//通过a,e得到对应mpd得到tile地址
		vector<string> tiles_url = tile_choose(a, e, mpd, current_gof,tile_id,frame_id);
		int i = 0;
		tile_parse = gof_tile[current_gof];
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
	
	//while (true) {
	//	Sleep(33);
	//	ui.qvtkWidget->update();
	//	viewer->updateText(to_string(gof_tile[0]), 10, 10, "tile_parse");
	//	viewer->updatePointCloud(cloud);

	//}
}

//接收
void PCLVisualizer::replyFinished(QNetworkReply *reply)
{
	if (reply->error() == QNetworkReply::NoError)
	{
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
			double end = GetTickCount();
			//cost = (end - start) / 1000;//s
			//double speed = bytes.size() / 1024 / cost;
			//bandwidth.push_back(speed);
			char* buf;
			buf = bytes.data();
			ofstream outfile("mainfest.xml", ios::app);
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
			QString gof_tile = gof_id + "_" + tile_id;
			vector<uint8_t> temp(bytes.begin(),bytes.end());
			bin_store.insert(pair<QString, vector<uint8_t>>(gof_tile, temp));//写入解压区
		}

		//如果是ply文件,无序,读取frame_id
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
			iter = fusion.find(frame_id);
			if (iter!=fusion.end()) {
				//如果已经有对应的帧  则新增tile的点
				fusion_mutex.lock();
				frame2xyzrgb(ply, iter->second);
				fusion_mutex.unlock();
				gof_tile[gof_id-1]--;
			}
			else {
				//第一次新增
				PCCPointSet3 frame;
				frame2xyzrgb(ply, frame);
				fusion_mutex.lock();
				fusion.insert(pair<int, PCCPointSet3>(frame_id,frame));
				fusion_mutex.unlock();
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