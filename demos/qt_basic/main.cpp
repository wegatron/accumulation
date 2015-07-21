#include <iostream>
#include <QApplication>
#include <QFile>

#include "mainwindow.h"
#include "cmenu.h"

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);
   QFile qss("/home/wegatron/workspace/accumulation/qt/basic/stylesheet.qss");
   qss.open(QFile::ReadOnly);
   app.setStyleSheet(qss.readAll());
   qss.close();
  MainWindow window;
  window.show();
  return app.exec();
}
