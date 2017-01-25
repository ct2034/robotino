//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MyCamera.h"

MyCamera::MyCamera()
{
	setCameraNumber( -1 );
}

void MyCamera::imageReceivedEvent( const unsigned char* data,
	unsigned int dataSize,
	unsigned int width,
	unsigned int height,
	unsigned int step )
{
	QImage image( width, height, QImage::Format_RGB888 );

	for( unsigned int line=0; line<height; ++line )
	{
		const unsigned char* psrc = (const unsigned char*)data + step * line;
		unsigned char* pdst = image.scanLine( line );
		memcpy( pdst, psrc, width*3 );
	}

	Q_EMIT imageReceived( image );
}

void MyCamera::capabilitiesChangedEvent( const rec::robotino::api2::CameraCapabilities& capablities )
{
	Q_EMIT capabilitiesChanged( capablities );
}

void MyCamera::settingsChangedEvent( unsigned int width, unsigned int height, const char* format )
{
	Q_EMIT settingsChanged( width, height, QString( format ) );
}
