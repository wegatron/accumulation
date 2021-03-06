#ifndef CMENU_H
#define CMENU_H

#include <QList>
#include <QPushButton>
#include <QObject>

class CMenu : public QObject
{
  Q_OBJECT
public:
 CMenu(QWidget *parent, double radius=100) : parent_(parent), radius_(radius) {}
  void setParent(QWidget *parent) { parent_ = parent; }
  void setRadius(const double radius) { radius_ = radius; }
  void addItem(const QString &text);
  void show(const QPoint &pos);
public slots:
  void hide();
private:
  QList<QPushButton*> button_list_;
  QWidget *parent_;
  double radius_;
};

#endif // CMENU_H
