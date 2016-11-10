//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _CONTROLPANELWIDGET_H_
#define _CONTROLPANELWIDGET_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/OmniDrive.h>

class MyCom;

class ControlPanelWidget : public QWidget
{
	Q_OBJECT
public:
	ControlPanelWidget( QWidget* parent );

private Q_SLOTS:
	void on_clicked();
	void on_speed_valueChanged(int);
	void setVelocity();

private:

	QSlider* _speed;
	QPushButton* _ccl;
	QPushButton* _cl;
	QPushButton* _n;
	QPushButton* _s;
	QPushButton* _o;
	QPushButton* _w;
	QPushButton* _nw;
	QPushButton* _no;
	QPushButton* _sw;
	QPushButton* _so;
	QPushButton* _stop;

	int _vx;
	int _vy;
	int _omega;

	rec::robotino::api2::OmniDrive _omnidrive;
};

#endif //_CONTROLPANELWIDGET_H_
