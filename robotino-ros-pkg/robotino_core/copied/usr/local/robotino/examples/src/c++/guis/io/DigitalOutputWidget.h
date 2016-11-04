#ifndef _DigitalOutputWidget_H_
#define _DigitalOutputWidget_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/all.h>

class DigitalOutputWidget : public QWidget, rec::robotino::api2::DigitalOutputArray
{
	Q_OBJECT
public:
	DigitalOutputWidget();

private Q_SLOTS:
	void on_button_clicked();

private:
	QVector< QCheckBox* > _buttons;
};

#endif
