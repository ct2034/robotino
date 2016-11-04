#ifndef _BumperWidget_H_
#define _BumperWidget_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/all.h>

class BumperWidget : public QWidget, rec::robotino::api2::Bumper
{
	Q_OBJECT
public:
	BumperWidget();

private:
	void bumperEvent( bool hasContact );

	QLCDNumber* _number;
};

#endif
