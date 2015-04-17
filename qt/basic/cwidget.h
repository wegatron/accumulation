#ifndef CWIDGET_H
#define CWIDGET_H

#include <QWidget>

class CWidget : public QWidget
{
  Q_OBJECT
 public:
  CWidget(QWidget *parent=0) : QWidget(parent) {}

 protected:
  void mousePressEvent(QMouseEvent * event);
 signals:
  void mousePressed(QMouseEvent *event);
};


#endif /* CWIDGET_H */
