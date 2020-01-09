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
#ifndef abcde//ifndef��Ԥ����������������
#include <ilcplex/ilocplex.h>
typedef IloArray<IloIntVarArray> IntVarMatrix;
typedef IloArray<IloNumVarArray> NumVarMatrix;
typedef IloArray<IloIntArray> IntMatrix;
typedef IloArray<IloNumArray> NumMatrix;
//�����������
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
	int minx;//��Χ��cuboid
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
	int cuboid_n;//�п鷽ʽnxnxm
	int cuboid_m;
	int flag;
	string baseurl;
	vector<gof> gofs;

};

QString stuff = ".pcd";
QString stuff_ply = ".ply";
int buffer_size;
map<string, string>mpd;
object obj;//���ƶ���
vector<pcl::visualization::Camera> camera;

//ģ��hololens2��fov
double fov_y = 29;
double fov_x = 43;
double fovx2fovy = tan(fov_y / 2 * PI / 180) / tan(fov_x / 2 * PI / 180);

QMutex mymutex(QMutex::Recursive);
QMutex fusion_mutex(QMutex::Recursive);
QMutex buffer_mutex(QMutex::Recursive);
vector<vector<float>> xyzs;//һ֡���������꣨���������е�����㣩

typedef pcl::PointXYZRGB PointT;
double start;
vector<vector<int>> x;


queue<map<int, PCCPointSet3>>buffer;
map<int, PCCPointSet3> fusion;//�洢5�� frame_id->
int tile_parse;
int gof_tile[15] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};//��֤�Ƿ��ں���� gof_id->tilenumber

map<QString, vector<uint8_t>> bin_store;//(tile_id,gof_id��->bin

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
//ÿ����п�����
vector<tile_gof> gettile_seg(int j, const int n, float tile_h, float tile_x, float tile_z) {

	tile_gof tile1;
	tile1.tile_id = j * n*n;
	vector<tile_gof> tiles_seg;

	for (int k = 0; k < 4; k++) {//����ÿ��ĳ�ʼ�飨������(0,y,0)����
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
//��ȡmpd�ļ�
bool loadXML(const char *filename)
{
	// XML�ĵ�
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

	// �����ӽڵ�
	for (TiXmlElement *elem = root->FirstChildElement(); elem != NULL; elem = elem->NextSiblingElement())
	{
		// ��ȡԪ����
		gof gof_tmp;

		gof_tmp.gof_id =stoi(elem->Attribute("id"));
		gof_tmp.minx = stoi(elem->Attribute("minx"));
		gof_tmp.miny = stoi(elem->Attribute("miny"));
		gof_tmp.minz = stoi(elem->Attribute("minz"));
		gof_tmp.maxx = stoi(elem->Attribute("maxx"));
		gof_tmp.maxy = stoi(elem->Attribute("maxy"));
		gof_tmp.maxz = stoi(elem->Attribute("maxz"));

			//�ж�adp����
			for (TiXmlElement *AdaptationSet = elem->FirstChildElement(); AdaptationSet != NULL; AdaptationSet = AdaptationSet->NextSiblingElement()) {
				string adp_id = AdaptationSet->Attribute("id");
				// δѹ���ĵ�����Ϣ
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

				//ѹ����ĵ�����Ϣ
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

			//����ÿ��tile�İ˸�����
			size_t relativexyz[3];
			vector<tile_gof> tiles;
			int tile_number = obj.cuboid_n * obj.cuboid_n*obj.cuboid_m;//����
			float relativex = gof_tmp.maxx - gof_tmp.minx;
			float relativey = gof_tmp.maxy - gof_tmp.miny;
			float relativez = gof_tmp.maxz - gof_tmp.minz;
			//flag=1��ʾ��xyƽ��Ϊˮƽ��
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
				//��ȡÿ����п�
				for (int i = 0; i < obj.cuboid_m; i++) {
					//��ȡ�ò��п�
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
	
	// �����ڴ�
	doc.Clear();
	return true;
}

PCLVisualizer::PCLVisualizer(QWidget *parent)
{

	ui.setupUi(this);
	threadpool.setMaxThreadCount(4);
	//��ʼ��
	initialVtkWidget();

	ui.mainfest_url->setText("http://localhost/eghost_loot/mainfest.xml");
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
	//x,eת����tile_info
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

//�жϿ�Ķ�������ĵ��Ƿ���fov������
bool is_tileinfov(pcl::visualization::Camera ca, point3 p) {

	point3 pt_pos; pt_pos.x = ca.pos[0]; pt_pos.y = ca.pos[1]; pt_pos.z = ca.pos[2];
	point3 pt_focal; pt_focal.x = ca.focal[0]; pt_focal.y = ca.focal[1]; pt_focal.z = ca.focal[2];

	point3 vec_fp = subt(pt_focal, pt_pos);//�ӽ�����

	double l_view = distance(pt_pos, pt_focal);//�ӽ���������;

	double l_vertical = l_view * tan(fov_y / 2 * PI / 180);//��ֱ������

	point3 vec_vertical; vec_vertical.x = ca.view[0]; vec_vertical.y = ca.view[1]; vec_vertical.z = ca.view[2];//��ֱ����

	vec_vertical.x = l_vertical / (vlen(vec_vertical))*vec_vertical.x; vec_vertical.y = l_vertical / (vlen(vec_vertical))*vec_vertical.y; vec_vertical.z = l_vertical / (vlen(vec_vertical))*vec_vertical.z;//��������

	double l_horizon = l_vertical / fovx2fovy;

	point3 vec_horizon = OuterProduct(vec_fp, vec_vertical);//ˮƽ����

	vec_horizon.x = l_horizon / (vlen(vec_horizon))*vec_horizon.x; vec_horizon.y = l_horizon / (vlen(vec_horizon))*vec_horizon.y; vec_horizon.z = l_horizon / (vlen(vec_horizon))*vec_horizon.z;//��������

	point3 vec_1 = addv(vec_vertical, vec_horizon);
	point3 vec_2 = inverse(vec_1);
	point3 vec_3 = addv(inverse(vec_vertical), vec_horizon);
	point3 vec_4 = inverse(vec_3);

	point3 focal_1 = addv(pt_focal, vec_1);//�������������潻��
	point3 focal_2 = addv(pt_focal, vec_2);
	point3 focal_3 = addv(pt_focal, vec_3);
	point3 focal_4 = addv(pt_focal, vec_4);

	point3 pty1 = pointinplane(p, pt_pos, focal_1, focal_3);//����ƽ��ͶӰ��
	point3 pty2 = pointinplane(p, pt_pos, focal_2, focal_4);

	point3 vec_ty1 = subt(pty1, p);//����ͶӰ����
	point3 vec_ty2 = subt(pty2, p);

	point3 pty3 = pointinplane(p, pt_pos, focal_1, focal_4);//����ƽ��ͶӰ��
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

	IloEnv env;//���廷��
	IloModel model(env);//����ģ��
	IloExpr obj(env);//���ʽ
	IloInt R = 5;//�ȼ���
	IloInt K = kk;//�п���Ŀ
	IloInt bw = 73.75;//����

	vector<int> CC(c);//��Ҫ����Ŀ�

	IloInt NC = 2;//������
	IloInt C1 = 1;//�п���С������Դ
	IloInt maxNcore = 16;
	IloInt CU12C = 1;//
	vector <double>u(16);//ת��Ч��
	vector<IloInt>CU(16);//����������������Դ
	for (int i = 0; i < maxNcore; i++)
	{
		u[i] = exp(0.02*(-i));
	}

	for (size_t i = 0; i < maxNcore; i++)
	{
		CU[i] = i * u[i] * CU12C;
	}
	IloInt(*tile_pn)[12] = new IloInt[5][12]();//12x5�ľ��� ÿ���п���ܵ���
	double(*tile_C)[12] = new double[5][12]();//ÿ���п��C
	double(*binsize)[12] = new double[5][12]();
	double(*plysize)[12] = new double[5][12]();

	IloInt b0 = 4;//��ʼbuffer
	IloInt minp = LONG_MAX;
	size_t tile_all = 0;//�ܵ���
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



	vector<double> w(K);//ÿ���п��Ȩ��
	for (size_t k = 0; k < K; k++)
	{
		size_t s = tile_pn[4][k];
		double t = s * 1.0 / tile_all;
		w[k] = t;

	}

	vector<double> dist(K);//ÿ����ľ���
	for (size_t k = 0; k < K; k++)
	{
		dist[k] = k + 1;
	}

	double(*A)[12] = new double[5][12];//����Ȩ�� 5��12��
	for (size_t r = 0; r < R; r++)
	{
		for (size_t k = 0; k < K; k++)
		{
			if (CC[k] == 0) {//���Ǵ����벻��
				A[r][k] = 0;
			}
			else {
				A[r][k] = (1.0 / dist[k])*(r + 1) * 80 * w[k];
			}
		}
	}

	//ϵ������
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
	NumMatrix b(env, 5);//ϵ������ IloNum����double ����doule���͵ľ���
	for (int i = 0; i < 5; i++)
	{
		b[i] = IloNumArray(env, 12);
	}
	b[0] = b_1; b[1] = b_2; b[2] = b_3; b[3] = b_4; b[4] = b_5;




	IntVarMatrix x(env, 5);//���0,1��������
	IloIntVarArray e(env, 12, 0, 1);//������0,1����


	for (int i = 0; i < 5; i++)
	{
		x[i] = IloIntVarArray(env, 12, 0, 1);
	}


	IloExpr v1(env);//Ŀ�꺯��
	IloExpr v2(env);//x[1][i]֮��Ϊ1
	IloExpr v3(env);//Td�ı��ʽ
	IloExpr v4(env);//Ts�ı��ʽ
	IloExpr v5(env);//Tb��Լ��
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			v1 += -(x[i][j] * b[i][j]);//����С ȡ����
		}
	}

	model.add(IloMinimize(env, v1));//Ŀ��

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
	double tdd = 3 * (NC*CU[NC] * CU12C);//Td��ϵ��
	v3 = v3 / tdd;

	//Ts�˴�ֻ����ply��size bin�Ŀ��Ժ��Բ���
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

	model.add(v5 >= 0);//���Tb��Լ��

	cout << model;

	obj.end();

	//����ģ�͵�����������
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


	//���߱������



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
						//itermap = fusion[gof_id - 1].find(frame_id);
						itermap = fusion.find(frame_id);
						if (itermap != fusion.end()) {
							//����,һ������
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
							//��һ��
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
				buffer.push(fusion);//д��buffer
				buffer_mutex.unlock();
				fusion.clear();//���
				int pos = p - gof_tile;//�ڼ���gof
				*p = 1;//��ԭ		
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

//play��ť(���������������)
void PCLVisualizer::download() {
	//����manifest.xml
	
	request.setUrl(QUrl(ui.mainfest_url->text()+ "?n="+"mainfest"));
	manager.get(request); start = GetTickCount();

	loadXML("mainfest.xml");//�����ȴ�
	//��ʼ����ͷ


	viewer->setCameraPosition(208, 500, 600, 208, 500, 300, 0, 0, 0);//pos,focal,
	//����tiles,buffer,Bw, �õ�x e
	viewer->addCoordinateSystem(100);
	vector<string>::iterator tiles_url_iter;
	int tile_n = obj.cuboid_n*obj.cuboid_n*obj.cuboid_m;//�п���
	writebuffer_thread();//����д��buffer���߳�
	decode_thread();//�����߳�
	readbuffer_thread();//��ȡbuffer���߳�

	obj.gof_n = 1;//���ԣ�������������
	for (int current_gof = 1; current_gof <= obj.gof_n; current_gof++) {
		
		int tile_trans_n=0;//��Ҫ������п���Ŀ
		int *e = new int[tile_n];
		vector<vector<int>> xv;//���ʵȼ�
		vector<int> ev;//������ʽ
		vector<int> cv;//�Ƿ���
		vector<int> x;
		int tmp=0;
		viewer->getCameras(camera);

		//���ÿ��tile�Ķ����Ƿ���fov��
		for (int n = 0; n < tile_n; n++) {
			for (int vertex = 0; vertex < 8; vertex++) {
				if (is_tileinfov(camera.front(), obj.gofs.at(current_gof - 1).tiles.at(n).points[vertex])) {
					cv.push_back(1);
					tile_trans_n++;
					break;
				}
				else {
					if (vertex == 7) {//��8�����㶼���� �ж����ĵ�
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

		//ÿ�μ���a,e��ֵ
		maxQoE(bandwidth, buffer.size(), x, e, cv, tile_n);

		int a[5][12];
		for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 12; j++) {
				
				a[i][j] = x[12 * i + j];

			}
		}
		//xv evת��Ϊ a e
		gof_tile[current_gof - 1] = tile_trans_n*10;//��ÿ��gof������֡��
		vector<string> tile_id;//ÿ������ply�ļ���bin�ļ�
		vector<int> frame_id;
		start = GetTickCount();
		//ͨ��a,e�õ���Ӧmpd�õ�tile��ַ
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

//����
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
		//�����xml�ļ�
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

		//����ƥ��
		reg = ".+(.bin)$";
		QRegExp rx(reg);
		match = rx.exactMatch(fileName);
		
		
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
			iter = fusion.find(frame_id);
			if (iter!=fusion.end()) {
				//����Ѿ��ж�Ӧ��֡  ������tile�ĵ�
				fusion_mutex.lock();
				frame2xyzrgb(ply, iter->second);
				fusion_mutex.unlock();
				gof_tile[gof_id-1]--;
			}
			else {
				//��һ������
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