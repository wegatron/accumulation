#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "cmenu.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  MainWindow(QWidget *parent = 0);

public slots:
  void ShowContextMenu(const QPoint& pos);
  void hideContextMenu(QMouseEvent *event);
private:
  CMenu cmenu_;
};

#endif /* MAINWINDOW_H */
