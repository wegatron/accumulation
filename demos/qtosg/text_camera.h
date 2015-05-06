#ifndef TEXT_CAMERA_H
#define TEXT_CAMERA_H

#include <osgText/Text>
#include <osg/CameraNode>
#include <osg/Switch>
#include <osg/PositionAttitudeTransform>
#include <string>

class text_camera
{
public:
  text_camera():help_camera_on_(true) {}

  void update_text(const std::string& st) {
    if(text_ != NULL)
      text_->setText("nothing");
  }

  void change_visibility() {
    help_camera_on_ = !help_camera_on_;
    (help_camera_on_) ? help_camera_switch_->setAllChildrenOn()
    : help_camera_switch_->setAllChildrenOff();
  }

  void adjust_size(int width,  int height) {
    camera_->setViewport(0,height-600, 800,600);
  }

  osg::Node* createHUD() {
    //文字
    text_ = new osgText::Text;
    text_->setLineSpacing(1.1);
    //设置文字显示的位置
    osg::Vec3 position(6.0f,590.0f,0.0f);

    text_->setPosition(position);
    text_->setCharacterSize(14);
    text_->setColor( osg::Vec4( 1, 1, 1, 1));
    text_->setFont("SIMSUN.TTC");
    text_->setFontResolution(600,800);
    text_->setText("Default text!!!");//设置显示的文字

    //几何体节点
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable( text_ );//将文字Text作这drawable加入到Geode节点中
    //设置状态
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);//关闭灯光
    stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);//关闭深度测试

    //打开GL_BLEND混合模式（以保证Alpha纹理正确）
    stateset->setMode(GL_BLEND,osg::StateAttribute::ON);

    //相机
    camera_ = new osg::Camera;
    //设置透视矩阵
    camera_->setProjectionMatrix(osg::Matrix::ortho2D(0,600,0,600));//正交投影
    camera_->setViewport(0,0, 800,600);

    //设置绝对参考坐标系，确保视图矩阵不会被上级节点的变换矩阵影响
    camera_->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    //视图矩阵为默认的
    camera_->setViewMatrix(osg::Matrix::identity());
    //设置背景为透明，否则的话可以设置ClearColor
    camera_->setClearMask(GL_DEPTH_BUFFER_BIT);
    camera_->setAllowEventFocus( false);//不响应事件，始终得不到焦点

    //设置渲染顺序，必须在最后渲染
    camera_->setRenderOrder(osg::CameraNode::POST_RENDER);
    camera_->addChild(geode);//将要显示的Geode节点加入到相机
    help_camera_switch_ = new osg::Switch;
    (help_camera_on_) ? help_camera_switch_->setAllChildrenOn()
    : help_camera_switch_->setAllChildrenOff();
    help_camera_switch_->addChild(camera_);
    return help_camera_switch_;
  }

private:
  osg::Switch *help_camera_switch_;
  osg::Camera *camera_;
  osgText::Text* text_;
  bool help_camera_on_;
};


#endif // TEXT_CAMERA_H
