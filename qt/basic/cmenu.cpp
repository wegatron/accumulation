#include "cmenu.h"

#include <iostream>

using namespace std;

#define PI 3.1415926

void CMenu::addItem(const QString &text)
{
  QPushButton *button = new QPushButton(text, parent_);  button->hide();
  button_list_.push_back(button);
  connect(button, SIGNAL(released()), this, SLOT(hide()));
}

void CMenu::show(const QPoint &pos)
{
  // std::cout << "show on " << pos.x() << "  " << pos.y() << std::endl;
  size_t n = button_list_.size();
  double step = 2*PI/n;
  double angle = PI/2;
  for(size_t i =0; i<button_list_.size(); ++i) {
    button_list_[i]->setGeometry(pos.x()+radius_*cos(angle), pos.y()+radius_*sin(angle), 50, 25);
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
