#include "DaracoPlayer.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DaracoPlayer w;
    w.show();
    return a.exec();
}
