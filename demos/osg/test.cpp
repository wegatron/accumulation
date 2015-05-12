#include <osg/Group>
#include <osg/Geode>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osgText/Text3D>

osg::Node * text3d()
{
  osgText::Text3D* text3D = new osgText::Text3D;
  osg::ref_ptr<osgText::Font> font = osgText::readFontFile("/home/wegatron/workspace/accumulation/demos/osg/MSYH.TTF");
  osg::ref_ptr<osgText::Style> style = new osgText::Style;
  style->setThicknessRatio(1.0);
  text3D->setFont(font.get());
  text3D->setStyle(style.get());
  text3D->setCharacterSize(10);
  text3D->setDrawMode(osgText::Text3D::TEXT | osgText::Text3D::BOUNDINGBOX);
  text3D->setAxisAlignment(osgText::Text3D::XZ_PLANE);
  text3D->setText("Hellow");

  osg::Group *group = new osg::Group;
  osg::Geode* geode = new osg::Geode;
  geode->addDrawable(text3D);
  group->addChild(geode);
  return group;
}

int main()
{
  osgViewer::Viewer viewer;

  viewer.setSceneData( text3d() );

  return (viewer.run());
}
