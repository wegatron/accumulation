#include "osgwidget.h"

#include <iostream>

#include <osgDB/ReadFile>
#include <osgGA/OrbitManipulator>


OSGWidget::OSGWidget(QWidget *parent) : QGLWidget(parent)
{
  viewer_ = new osgViewer::Viewer;
  graphics_window_ = new osgViewer::GraphicsWindowEmbedded(this->x(),this->y(),
                                                           this->width(),this->height());
  float aspectRatio = static_cast<float>(this->width()) / static_cast<float>(this->height());
  osg::ref_ptr<osg::Camera> camera = viewer_->getCamera();
  camera->setViewport( 0, 0, this->width(), this->height());
  camera->setProjectionMatrixAsPerspective( 30.f, aspectRatio, 1.0f, 1000.f );
  camera->setGraphicsContext( graphics_window_ );
  osgGA::OrbitManipulator *manipulator =  new osgGA::OrbitManipulator;
  viewer_->setCameraManipulator(manipulator);
  viewer_->getCameraManipulator()->home(1.0);
}

void OSGWidget::paintEvent( QPaintEvent* paintEvent )
{
  QPainter painter( this );
  painter.setRenderHint( QPainter::Antialiasing );

  this->paintGL();
  painter.end();
}

void OSGWidget::paintGL()
{
  viewer_->frame();
}

void OSGWidget::resizeGL( int width, int height )
{
  osg::ref_ptr<osg::Camera> camera = viewer_->getCamera();
  graphics_window_->resized( this->x(), this->y(), width, height );
  camera->setViewport( 0, 0, this->width(), this->height());
}

osgGA::EventQueue* OSGWidget::getEventQueue() const
{
  osgGA::EventQueue* eventQueue = graphics_window_->getEventQueue();
  if( eventQueue )
    return( eventQueue );
  else
    throw( std::runtime_error( "Unable to obtain valid event queue") );
}

void OSGWidget::mouseMoveEvent( QMouseEvent* event )
{
  {
    this->getEventQueue()->mouseMotion( static_cast<float>( event->x() ),
                                        static_cast<float>( event->y() ) );
  }
}

void OSGWidget::mousePressEvent( QMouseEvent* event )
{
  unsigned int button = 0;
  switch( event->button() ) {
  case Qt::LeftButton:
    button = 1;
    break;

  case Qt::MiddleButton:
    button = 2;
    break;

  case Qt::RightButton:
    button = 3;
    break;

  default:
    break;
  }

  this->getEventQueue()->mouseButtonPress( static_cast<float>( event->x() ),
                                           static_cast<float>( event->y() ),
                                           button );
}

void OSGWidget::mouseReleaseEvent( QMouseEvent* event )
{
      unsigned int button = 0;

    switch( event->button() ) {
    case Qt::LeftButton:
      button = 1;
      break;

    case Qt::MiddleButton:
      button = 2;
      break;

    case Qt::RightButton:
      button = 3;
      break;

    default:
      break;
    }

    this->getEventQueue()->mouseButtonRelease( static_cast<float>( event->x() ),
        static_cast<float>( event->y() ),
        button );
  }

void OSGWidget::wheelEvent( QWheelEvent* event )
{
  event->accept();
  int delta = event->delta();

  osgGA::GUIEventAdapter::ScrollingMotion motion = delta > 0 ?   osgGA::GUIEventAdapter::SCROLL_UP
    : osgGA::GUIEventAdapter::SCROLL_DOWN;

  this->getEventQueue()->mouseScroll( motion );
}

bool OSGWidget::event( QEvent* event )
{
  bool handled = QGLWidget::event( event );

  // This ensures that the OSG widget is always going to be repainted after the
  // user performed some interaction. Doing this in the event handler ensures
  // that we don't forget about some event and prevents duplicate code.
  switch( event->type() ) {
  case QEvent::KeyPress:
  case QEvent::KeyRelease:
  case QEvent::MouseButtonDblClick:
  case QEvent::MouseButtonPress:
  case QEvent::MouseButtonRelease:
  case QEvent::MouseMove:
  case QEvent::Wheel:
    this->update();
    break;

  default:
    break;
  }

  return( handled );
}
