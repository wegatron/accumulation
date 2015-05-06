#ifndef CMENU_H
#define CMENU_H

#include <QList>
#include <QPushButton>
#include <QObject>
#include <QtGui>

class CMenu : public QObject
{
  Q_OBJECT
public:
  CMenu(QWidget *parent=NULL, double radius=100) : parent_(parent), radius_(radius) {}
  void setParent(QWidget *parent) {
    parent_ = parent;
  }
  void setRadius(const double radius) {
    radius_ = radius;
  }

  void addItem(QSignalMapper *signal_mapper, const std::string &map_name, const std::string &icon_file, const std::string &tool_tip);
  void show(const QPoint &pos);
public slots:
  void hide();
private:
  QList<QPushButton*> button_list_;
  QWidget *parent_;
  double radius_;
};

#endif // CMENU_H
