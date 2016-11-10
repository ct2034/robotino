//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MyKinect.h"

MyKinect::MyKinect()
	: _objectColors( 256 )
{
	//setKinectNumber( -1 );

	for ( int i=0; i<2048; i++)
	{
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		_gamma[i] = v*6*256;
	}

	for ( int i=0; i<0xFFFF; i++)
	{
		float v = i;
		v /= 4096;
		v = powf(v, 3)* 6;
		_gamma2[i] = v*6*256;
	}

	for( int i=0; i<_objectColors.size(); ++i )
	{
		_objectColors[i] = new QColor( 255-(i<<7)%255, 255-(i<<6)%255, 255-(i<<5)%255 );
	}
}

QImage MyKinect::from11BitDepthData( const unsigned short* data, unsigned int dataSize, unsigned int width, unsigned int height )
{
	QImage image( width, height, QImage::Format_RGB888 );

	for( unsigned int line=0; line<height; ++line )
	{
		const quint16* psrc = data + width * line;
		unsigned char* pdst = image.scanLine( line );

		for ( unsigned int i=0; i<width; i++)
		{
			int pval = _gamma[*(psrc++)];
			int lb = pval & 0xff;
			switch (pval>>8) {
			case 0:
				*(pdst++) = 255;
				*(pdst++) = 255-lb;
				*(pdst++) = 255-lb;
				break;
			case 1:
				*(pdst++) = 255;
				*(pdst++) = lb;
				*(pdst++) = 0;
				break;
			case 2:
				*(pdst++) = 255-lb;
				*(pdst++) = 255;
				*(pdst++) = 0;
				break;
			case 3:
				*(pdst++) = 0;
				*(pdst++) = 255;
				*(pdst++) = lb;
				break;
			case 4:
				*(pdst++) = 0;
				*(pdst++) = 255-lb;
				*(pdst++) = 255;
				break;
			case 5:
				*(pdst++) = 0;
				*(pdst++) = 0;
				*(pdst++) = 255-lb;
				break;
			default:
				*(pdst++) = 0;
				*(pdst++) = 0;
				*(pdst++) = 0;
				break;
			}
		}
	}

	return image;
}

QImage MyKinect::fromMMDepthData( const unsigned short* data, unsigned int dataSize, unsigned int width, unsigned int height )
{
	QImage image( width, height, QImage::Format_RGB888 );

	for( unsigned int line=0; line<height; ++line )
	{
		const quint16* psrc = data + width * line;
		unsigned char* pdst = image.scanLine( line );

		for ( unsigned int i=0; i<width; i++)
		{
			int pval = _gamma2[*(psrc++)];
			int lb = pval & 0xff;
			switch (pval>>8) {
			case 0:
				*(pdst++) = 255;
				*(pdst++) = 255-lb;
				*(pdst++) = 255-lb;
				break;
			case 1:
				*(pdst++) = 255;
				*(pdst++) = lb;
				*(pdst++) = 0;
				break;
			case 2:
				*(pdst++) = 255-lb;
				*(pdst++) = 255;
				*(pdst++) = 0;
				break;
			case 3:
				*(pdst++) = 0;
				*(pdst++) = 255;
				*(pdst++) = lb;
				break;
			case 4:
				*(pdst++) = 0;
				*(pdst++) = 255-lb;
				*(pdst++) = 255;
				break;
			case 5:
				*(pdst++) = 0;
				*(pdst++) = 0;
				*(pdst++) = 255-lb;
				break;
			default:
				*(pdst++) = 0;
				*(pdst++) = 0;
				*(pdst++) = 0;
				break;
			}
		}
	}

	return image;
}

QImage MyKinect::fromObjectData( const unsigned short* data, unsigned int dataSize, unsigned int width, unsigned int height )
{
	QImage image( width, height, QImage::Format_RGB888 );

	for( unsigned int line=0; line<height; ++line )
	{
		const float* psrc = reinterpret_cast<const float*>(data) + 3 * width * line;
		unsigned char* pdst = image.scanLine( line );

		for ( unsigned int i=0; i<width; i++)
		{
			float r = *(psrc++);
			r *= 255;
			if( r < 0 ) r = 0;
			if( r > 255 ) r = 255;

			float g = *(psrc++);
			g *= 255;
			if( g < 0 ) g = 0;
			if( g > 255 ) g = 255;
			
			float b = *(psrc++);
			b *= 255;
			if( b < 0 ) b = 0;
			if( b > 255 ) b = 255;

			*(pdst++) = static_cast<unsigned char>( r );
			*(pdst++) = static_cast<unsigned char>( g );
			*(pdst++) = static_cast<unsigned char>( b );
		}
	}

	return image;
}

void MyKinect::depthEvent( const unsigned short* data, unsigned int dataSize, const unsigned short* object_data, unsigned int object_dataSize, unsigned int width, unsigned int height, unsigned int format, unsigned int stamp )
{
	switch( format )
	{
	case 0:
		Q_EMIT depthReceived( from11BitDepthData( data, dataSize, width, height ) );
		break;

	default:
		Q_EMIT depthReceived( fromMMDepthData( data, dataSize, width, height ) );
	}	

	if( object_dataSize == width*height*6 )
	{
		Q_EMIT objectReceived( fromObjectData( object_data, object_dataSize, width, height ) );
	}
}

void MyKinect::videoEvent( const unsigned char* data, unsigned int dataSize, unsigned int width, unsigned int height, unsigned int step, unsigned int format, unsigned int stamp )
{
	QImage image( width, height, QImage::Format_RGB888 );

	for( unsigned int line=0; line<height; ++line )
	{
		const unsigned char* psrc = (const unsigned char*)data + step * line;
		unsigned char* pdst = image.scanLine( line );
		memcpy( pdst, psrc, width*3 );
	}

	Q_EMIT videoReceived( image );
}

void MyKinect::tiltEvent( double angleDeg )
{
	Q_EMIT tiltReceived( angleDeg );
}

void MyKinect::accelEvent( double x, double y, double z )
{
	Q_EMIT accelReceived( x, y, z );
}
