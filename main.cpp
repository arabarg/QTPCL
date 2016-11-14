#include "viewer.h"
#include "viewer2.h"
#include <QApplication>
#include <QLayout>



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Viewer w;
    Viewer2 w2;


w2.show();
 //w.show();
//w.setWindowState(Qt::WindowFullScreen);


    return a.exec();
}


