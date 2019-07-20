#include "PCLVisualizer.h"
#include <QtWidgets/QApplication>
#include <malloc.h>
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	PCLVisualizer w;
	w.show();
	return a.exec();
}

