#include "mainwindow.h"
#include <QApplication>
#include <QFile>

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  QFile qss("/home/wegatron/tmp_code/qt_designer/qttest/stylesheet.qss");

  qss.open(QFile::ReadOnly);
  a.setStyleSheet(qss.readAll());
  qss.close();
  MainWindow w;
  w.show();

  return a.exec();
}
