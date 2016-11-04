//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _Settings_H_
#define _Settings_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class MyKinect;

class Settings : public QWidget
{
	Q_OBJECT
public:
	Settings( QWidget* parent, MyKinect* kinect );

private Q_SLOTS:
	//void on_kinectSelector_activated( int );

	void on_tilt_valueChanged( int );
	void on_led_valueChanged( int );
	void on_depthFormat_activated( int );
	void on_videoFormat_activated( int );
	void on_kinectSelector_activated( int );

	void on_kinect_tiltReceived( double );
	void on_kinect_accelReceived( double, double, double );

private:
	MyKinect* _kinect;

	QComboBox* _kinectSelector;

	QSpinBox* _tilt;
	QSpinBox* _led;
	QComboBox* _depthFormat;
	QComboBox* _videoFormat;

	QLabel* _currentTilt;
	QLabel* _currentAccel;
};

#endif //_Settings_H_
