#include "MainWindow.h"
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <osgUtil/SmoothingVisitor>

MainWindow::MainWindow( QWidget* parent, Qt::WindowFlags flags )
  : QMainWindow( parent, flags )
{
  QMenuBar* menuBar = this->menuBar();
  QMenu* menu = menuBar->addMenu( "File" );
  menu->addAction( "Open...", this, SLOT( onOpen() ) );
  widget_ = new OSGWidget(this);
  setCentralWidget(widget_);
}

MainWindow::~MainWindow()
{
}

  void MainWindow::onOpen()
{
  QString file_name = QFileDialog::getOpenFileName(this, "load 3d model", ".", "*");
  osg::ref_ptr<osgViewer::Viewer> viewer = widget_->getViewer();
  osg::ref_ptr<osg::Node> scene_node = osgDB::readNodeFile(file_name.toStdString());
  osgUtil::SmoothingVisitor sv; 
  scene_node->accept(sv); 
  viewer->setSceneData(scene_node);
  widget_->updateGL();
 }
