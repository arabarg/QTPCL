#include "viewer.h"
#include <QApplication>
#include <QLayout>



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Viewer w;



w.show();
 //w.show();
//w.setWindowState(Qt::WindowFullScreen);


    return a.exec();
}


