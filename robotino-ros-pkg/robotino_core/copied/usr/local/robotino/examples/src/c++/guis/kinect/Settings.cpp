//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "Settings.h"
#include "MyKinect.h"

Settings::Settings( QWidget* parent, MyKinect* kinect )
	: QWidget( parent )
	, _kinect( kinect )
	, _kinectSelector( new QComboBox( this ) )
	, _tilt( new QSpinBox( this ) )
	, _led( new QSpinBox( this ) )
	, _depthFormat( new QComboBox( this ) )
	, _videoFormat( new QComboBox( this ) )
	, _currentTilt( new QLabel( this ) )
	, _currentAccel( new QLabel( this ) )
{
	_kinectSelector->addItems( QStringList() << "Kinect 0" << "Kinect 1" << "Kinect 2" << "Kinect 3" );
	_depthFormat->addItems( QStringList() << "11 bit" << "10 bit" << "11 bit packed" << "10 bit packed" << "mm aligned" << "mm unaligned" );
	_videoFormat->addItems( QStringList() << "Decompressed RGB" << "Bayer compressed" << "8-bit IR" << "10-bit IR" << "10-bit IR packed" << "YUV RGB" << "YUV Raw" );

	QFormLayout* flayout = new QFormLayout;
	setLayout( flayout );
	flayout->addRow( _kinectSelector );
	flayout->addRow( "depth format", _depthFormat );
	flayout->addRow( "video format", _videoFormat );
	flayout->addRow( "tilt", _tilt );
	flayout->addRow( "led", _led );
	flayout->addRow( "current tilt", _currentTilt );
	flayout->addRow( "current accel", _currentAccel );

	_tilt->setRange( -30, 30 );
	_led->setRange( 0, 6 );

	bool ok = connect( _kinectSelector, SIGNAL( activated( int ) ), SLOT( on_kinectSelector_activated( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _depthFormat, SIGNAL( activated( int ) ), SLOT( on_depthFormat_activated( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _videoFormat, SIGNAL( activated( int ) ), SLOT( on_videoFormat_activated( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _tilt, SIGNAL( valueChanged( int ) ), SLOT( on_tilt_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _led, SIGNAL( valueChanged( int ) ), SLOT( on_led_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _kinect, SIGNAL( tiltReceived( double ) ), SLOT( on_kinect_tiltReceived( double ) ) );
	Q_ASSERT( ok );

	ok = connect( _kinect, SIGNAL( accelReceived( double, double, double ) ), SLOT( on_kinect_accelReceived( double, double, double ) ) );
	Q_ASSERT( ok );
}

void Settings::on_depthFormat_activated( int index )
{
	_kinect->setDepthFormat( index );
}

void Settings::on_videoFormat_activated( int index )
{
	_kinect->setVideoFormat( index );
}

void Settings::on_kinectSelector_activated( int index )
{
	_kinect->setKinectNumber( index );
}

void Settings::on_tilt_valueChanged( int value )
{
	_kinect->setTilt( value );
}

void Settings::on_led_valueChanged( int value )
{
	_kinect->setLED( value );
}

void Settings::on_kinect_tiltReceived( double deg )
{
	_currentTilt->setText( QString( "%1" ).arg( deg ) );
}

void Settings::on_kinect_accelReceived( double x, double y, double z )
{
	_currentAccel->setText( QString( "%1 %2 %3" ).arg( x ).arg( y ).arg( z ) );
}