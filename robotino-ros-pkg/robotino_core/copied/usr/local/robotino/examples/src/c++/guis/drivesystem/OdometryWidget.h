//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _OdometryWidget_H_
#define _OdometryWidget_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/Odometry.h>

class MyCom;

class OdometryWidget : public QWidget, public rec::robotino::api2::Odometry
{
	Q_OBJECT
public:
	OdometryWidget( QWidget* parent );

private Q_SLOTS:
	void on_setButton_clicked();

private:
	void readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence );

	QPushButton* _setButton;
	QCheckBox* _setBlocking;
	QLineEdit* _setx;
	QLineEdit* _sety;
	QLineEdit* _setphi;
	QLabel* _odomtery;
	QLabel* _speed;
	QLabel* _sequence;
};

#endif //_OdometryWidget_H_
