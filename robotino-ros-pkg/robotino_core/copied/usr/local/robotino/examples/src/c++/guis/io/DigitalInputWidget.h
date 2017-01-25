#ifndef _DigitalInputWidget_H_
#define _DigitalInputWidget_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/all.h>

class DigitalInputWidget : public QWidget, rec::robotino::api2::DigitalInputArray
{
    Q_OBJECT
public:
	DigitalInputWidget();

private:
	void valuesChangedEvent( const int* values, unsigned int size );

	QVector< QLCDNumber* > _numbers;
};

#endif
