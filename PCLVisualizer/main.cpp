#include "PCLVisualizer.h"
#include <QtWidgets/QApplication>


using namespace std;
using namespace boost;
using namespace pcl;
typedef pcl::PointXYZRGBA PointType;


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	PCLVisualizer w;
	w.show();
	w.setWindowTitle(QString("NarWhal"));
	//while (true)
	//{
	//	
	//	//w.getviewer()->updatePointCloud(w.getcloud(), "cloud");
	//}
	return a.exec();
}