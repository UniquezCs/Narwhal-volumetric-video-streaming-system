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



void test(PCLVisualizer &w) {
	while (1) {

		size_t size = w.buffer.size();
	}


}