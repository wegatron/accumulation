#include "cmenu.h"
#include <iostream>

using namespace std;

#define PI 3.1415926

void CMenu::addItem(QSignalMapper *signal_mapper, const std::string &map_name, const std::string &icon_file, const std::string &tool_tip)
{
  QIcon icon(QString::fromStdString(icon_file));
  QPushButton *button = new QPushButton(icon, "", parent_);
  button->setToolTip(QString::fromUtf8(tool_tip.c_str()));
  button->hide();
  button_list_.push_back(button);
  signal_mapper->setMapping(button, QString::fromStdString(map_name));
  connect(button, SIGNAL(released()), this, SLOT(hide()));
  connect(button, SIGNAL(released()), signal_mapper, SLOT(map()));
}

void CMenu::show(const QPoint &pos)
{
  // std::cout << "show on " << pos.x() << "  " << pos.y() << std::endl;
  size_t n = button_list_.size();
  double step = 2*PI/n;
  double angle = PI/2;
  for(size_t i =0; i<button_list_.size(); ++i) {
    button_list_[i]->setGeometry(pos.x()+radius_*cos(angle), pos.y()+radius_*sin(angle), 35, 35);
    button_list_[i]->show();
    angle+= step;
  }
}

void CMenu::hide()
{
  for(QPushButton *button : button_list_) {
    button->hide();
  }
}
