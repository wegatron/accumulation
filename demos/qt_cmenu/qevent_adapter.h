#ifndef QEVENT_ADAPTER_H
#define QEVENT_ADAPTER_H

#include <QObject>

class QEventAdapter : public QObject
{
    Q_OBJECT

public slots:

  void buttonPressed(const QString &button_name);
};

#endif /* QEVENT_ADAPTER_H */
