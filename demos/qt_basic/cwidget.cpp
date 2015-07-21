#include "cwidget.h"

void CWidget::mousePressEvent(QMouseEvent * event)
{
  QWidget::mousePressEvent(event);
  emit mousePressed(event);
}
