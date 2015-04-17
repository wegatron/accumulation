#include <QMainWindow>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <iostream>


class MainWindow : public QMainWindow
{
public:
    MainWindow(QWidget *parent = 0);
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow window;
    window.show();
    return app.exec();
}
