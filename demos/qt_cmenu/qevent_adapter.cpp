#include "qevent_adapter.h"

#include <iostream>

void QEventAdapter::buttonPressed(const QString &button_name)
{
  std::cout << button_name.toStdString() << std::endl;
}
