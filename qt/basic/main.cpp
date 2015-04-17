#include <QMainWindow>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <iostream>

#include "cmenu.h"

class MainWindow : public QMainWindow
{
public:
  MainWindow(QWidget *parent = 0);

public slots:
  void ShowContextMenu(const QPoint& pos);

private:
  CMenu cmenu_;
};

MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent), cmenu_(NULL)
{
  // QWidget *center_widget = new QWidget(this);
  // this->setCentralWidget(center_widget);
  // cmenu_.setParent(center_widget);
  // cmenu_.addItem("item1");
  // cmenu_.addItem("item2");
  // cmenu_.addItem("item3");
  // cmenu_.addItem("item4");
  // center_widget->setContextMenuPolicy(Qt::CustomContextMenu);
  // connect(center_widget, SIGNAL(customContextMenuRequested(const QPoint&)),
  //         this, SLOT(ShowContextMenu(const QPoint&)));

  QWidget *center_widget = new QWidget(this);
  this->setCentralWidget(center_widget);

  cmenu_.setParent(center_widget);
  cmenu_.addItem("item1");

  center_widget->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(center_widget, SIGNAL(customContextMenuRequested(const QPoint&)),
          this, SLOT(ShowContextMenu(const QPoint&)));

}

void MainWindow::ShowContextMenu(const QPoint &pos)
{
  std::cout << "!!!!" << std::endl;
  cmenu_.show(pos);
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
