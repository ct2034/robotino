//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MyCom.h"
#include <rec/robotino/api2/utils.h>

MyCom* MyCom::_impl = NULL;

MyCom* MyCom::singleton()
{
	Q_ASSERT(_impl);
	return _impl;
}

void MyCom::init()
{
	Q_ASSERT(NULL == _impl);
	_impl = new MyCom;
}

void MyCom::done()
{
	delete _impl;
	_impl = NULL;
}

MyCom::MyCom()
	: rec::robotino::api2::Com( QApplication::applicationName().toLatin1().constData() )
{
	QTimer* spinTimer = new QTimer( this );
	spinTimer->setSingleShot( false );
	spinTimer->setInterval( 10 );
	bool ok = connect( spinTimer, SIGNAL( timeout() ), SLOT( on_spinTimer_timeout() ) );
	Q_ASSERT( ok );

	ok = connect( qApp, SIGNAL( aboutToQuit() ), SLOT( on_aboutToQuit() ) );
	Q_ASSERT( ok );

	spinTimer->start();
}

void MyCom::on_spinTimer_timeout()
{
	processEvents();
}

void MyCom::on_aboutToQuit()
{
	rec::robotino::api2::shutdown();
}


void MyCom::errorEvent( const char* errorString )
{
}

void MyCom::connectedEvent()
{
	Q_EMIT connected();
}

void MyCom::connectionClosedEvent()
{
	Q_EMIT disconnected();
}
