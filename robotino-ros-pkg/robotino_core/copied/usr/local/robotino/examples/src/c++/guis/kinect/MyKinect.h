//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _MYKinect_H_
#define _MYKinect_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/Kinect.h>

class MyKinect : public QObject, public rec::robotino::api2::Kinect
{
	Q_OBJECT
public:
	MyKinect();

Q_SIGNALS:
	void depthReceived( const QImage& image );
	void objectReceived( const QImage& image );
	void videoReceived( const QImage& image );
	void tiltReceived( double );
	void accelReceived( double, double, double );

private:
	void depthEvent( const unsigned short* data, unsigned int dataSize, const unsigned short* object_data, unsigned int object_dataSize, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp );
	void videoEvent( const unsigned char* data, unsigned int dataSize, unsigned int width, unsigned int height, unsigned int step, unsigned int format, unsigned int stamp );

	void tiltEvent( double angleDeg );
	void accelEvent( double x, double y, double z );

	QImage from11BitDepthData( const unsigned short* data, unsigned int dataSize, unsigned int width, unsigned int height );
	QImage fromMMDepthData( const unsigned short* data, unsigned int dataSize, unsigned int width, unsigned int height );
	QImage fromObjectData( const unsigned short* data, unsigned int dataSize, unsigned int width, unsigned int height );

	quint16 _gamma[2048];
	quint16 _gamma2[0xFFFF];
	QVector<QColor*> _objectColors;
};

#endif //_MYKinect_H_
