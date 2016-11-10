//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _MYCAMERA_H_
#define _MYCAMERA_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/Camera.h>

class MyCamera : public QObject, public rec::robotino::api2::Camera
{
	Q_OBJECT
public:
	MyCamera();

Q_SIGNALS:
	void imageReceived( const QImage& image );

	void capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& capablities );

	void settingsChanged( unsigned int width, unsigned int height, const QString& format );

private:
	void imageReceivedEvent( const unsigned char* data,
							 unsigned int dataSize,
							 unsigned int width,
							 unsigned int height,
							 unsigned int step );

	void capabilitiesChangedEvent( const rec::robotino::api2::CameraCapabilities& capablities );

	void settingsChangedEvent( unsigned int width, unsigned int height, const char* format );
};

#endif //_MYCAMERA_H_
