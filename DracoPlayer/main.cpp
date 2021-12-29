#include "DracoPlayer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DracoPlayer w;
    w.show();
    return a.exec();
}
