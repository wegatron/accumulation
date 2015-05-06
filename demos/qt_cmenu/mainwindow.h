#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStyleOptionButton>
#include <QPainter>
#include <QAction>
#include <QMenu>
#include <QPushButton>
#include "qevent_adapter.h"

#include "cmenu.h"

namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

public slots:
  void ShowContextMenu(const QPoint& pos);
private slots:
  void on_horizontalSlider_sliderMoved(int position);

  void on_pushButton_clicked();

private:
  CMenu cmenu_;
  QEventAdapter qevent_adapter_;
};

class CButton : public QPushButton
{
  Q_OBJECT
public:
  explicit CButton(QWidget *parent=0) : QPushButton(parent) {}
  explicit CButton(const QString &text, QWidget *parent=0) : QPushButton(text, parent) {}
  //CButton(const QIcon& icon, const QString &text, QWidget *parent=0) : QPushButton(icon, text, parent) {}
  void setIcons(const std::string &icon_in_file, const std::string &icon_out_file)
  {
    icon_in = QIcon(icon_in_file.c_str());
    icon_out = QIcon(icon_out_file.c_str());
    setIcon(icon_out);
  }

protected:

  void enterEvent(QEvent *event)
  {
    this->QPushButton::enterEvent(event);
    setIcon(icon_in);
  }
  void leaveEvent(QEvent *event)
  {
    this->QPushButton::enterEvent(event);
    setIcon(icon_out);
  }
private:
  QIcon icon_in;
  QIcon icon_out;
};

#endif // MAINWINDOW_H
