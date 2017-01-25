//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _MYCOM_H_
#define _MYCOM_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/Com.h>

class MyCom : public QObject, public rec::robotino::api2::Com
{
	Q_OBJECT
public:
	static MyCom* singleton();

	static void init();
	static void done();

Q_SIGNALS:
	void connected();
	void disconnected();

private Q_SLOTS:
	void on_spinTimer_timeout();
	void on_aboutToQuit();

private:
	MyCom();
	
	void errorEvent(const char* errorString);
	void connectedEvent();
	void connectionClosedEvent();

	static MyCom* _impl;
};

#endif //_MYCOM_H_
