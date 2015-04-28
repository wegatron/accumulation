#ifndef MainWindow_h__
#define MainWindow_h__

#include <QMainWindow>
#include <QString>
#include "osgwidget.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow( QWidget* parent = 0, Qt::WindowFlags flags = 0 );
  ~MainWindow();

private slots:
  void onOpen();

 private:
    OSGWidget *widget_;
};

#endif
