#ifndef OSGWIDGET_H
#define OSGWIDGET_H

#include <QtOpenGL>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>
#include "text_camera.h"

class OSGWidget : public QGLWidget
{
 public:
  OSGWidget(QWidget *parent);
  osg::ref_ptr<osgViewer::Viewer> getViewer() { return viewer_; }
 protected:
  virtual void paintEvent( QPaintEvent* paintEvent );
  virtual void paintGL();
  virtual void resizeGL( int width, int height );
  virtual void mouseMoveEvent( QMouseEvent* event );
  virtual void mousePressEvent( QMouseEvent* event );
  virtual void mouseReleaseEvent( QMouseEvent* event );
  virtual void wheelEvent( QWheelEvent* event );
  virtual bool event( QEvent* event );

 private:
  text_camera tc_;
  osgGA::EventQueue* getEventQueue() const;
  osg::ref_ptr<osgViewer::Viewer> viewer_;
  osg::ref_ptr<osgViewer::GraphicsWindowEmbedded> graphics_window_;
};

#endif /* OSGWIDGET_H */
