#include "mainwindow.h"

#include <iostream>
#include <QMouseEvent>
#include "cwidget.h"

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent), cmenu_(NULL)
{
  // QWidget *center_widget = new QWidget(this);
  CWidget *center_widget = new CWidget(this);
  this->setCentralWidget(center_widget);
  cmenu_.setParent(center_widget);
  cmenu_.addItem("item1");
  cmenu_.addItem("item2");
  cmenu_.addItem("item3");
  cmenu_.addItem("item4");
  cmenu_.addItem("item5");
  cmenu_.addItem("item6");
  center_widget->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(center_widget, SIGNAL(customContextMenuRequested(const QPoint&)),
          this, SLOT(ShowContextMenu(const QPoint&)));

  connect(center_widget, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(hideContextMenu(QMouseEvent *)));
}

void MainWindow::ShowContextMenu(const QPoint &pos)
{
  // std::cout << "!!!!" << std::endl;
  cmenu_.show(pos);
}

void MainWindow::hideContextMenu(QMouseEvent *event)
{
  if(event->button() != Qt::RightButton) {
    cmenu_.hide();
  }
}
