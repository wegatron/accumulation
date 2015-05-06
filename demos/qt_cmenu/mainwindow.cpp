#include "mainwindow.h"

#include <iostream>
#include <QImage>
#include <QBitmap>
#include <QtGui/QLineEdit>
#include <QHBoxLayout>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent), cmenu_(NULL)
{

  QWidget *center_widget = new QWidget(this);
  this->setCentralWidget(center_widget);

  cmenu_.setParent(center_widget);
  QSignalMapper *signal_mapper = new QSignalMapper(this);
  cmenu_.addItem( signal_mapper, "button_0", "../button_0.png", "tool_tip");
  cmenu_.addItem( signal_mapper, "button_1", "../button_0.png", "tool_tip");
  cmenu_.addItem( signal_mapper, "button_2", "../button_0.png", "tool_tip");
  cmenu_.addItem( signal_mapper, "button_3", "../button_0.png", "tool_tip");

  connect(signal_mapper, SIGNAL(mapped(const QString&)),  &qevent_adapter_, SLOT(buttonPressed(const QString&)));

  center_widget->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(center_widget, SIGNAL(customContextMenuRequested(const QPoint&)),
      this, SLOT(ShowContextMenu(const QPoint&)));
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{

}

void MainWindow::ShowContextMenu(const QPoint& pos) // this is a slot
{
  std::cout << "show!!!" << std::endl;
  cmenu_.show(pos);
}

void MainWindow::on_pushButton_clicked()
{
  cmenu_.hide();
    //std::cout << "clicked!" << std::endl;
}
