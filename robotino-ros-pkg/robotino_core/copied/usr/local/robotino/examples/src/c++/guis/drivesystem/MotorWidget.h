//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _MOTORWIDGET_H_
#define _MOTORWIDGET_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/Motor.h>

class MyCom;

class MotorWidget : public QWidget, public rec::robotino::api2::Motor
{
	Q_OBJECT
public:
	MotorWidget( QWidget* parent, int number );

private:
	QPushButton* _resetPosButton;
	QLabel* _velLabel;
	QLabel* _posLabel;
	QLabel* _currentLabel;

	void motorReadingsChanged( float velocity, int position, float current );

private Q_SLOTS:
	void on_resetPosButton_clicked();
};

#endif //_MOTORWIDGET_H_
