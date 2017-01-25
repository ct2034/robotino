//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#include "AddressInput.h"
#include "MyCom.h"

AddressInput::AddressInput( QWidget* parent )
: QToolBar( parent )
, _lineEdit( new QLineEdit )
, _connectButton( new QPushButton )
, _connectTimer( new QTimer( this ) )
, _connectCounter( 0 )
{
	setMinimumSize( 200, 46 );
	setFloatable( false );
	setMovable( false );

	QSettings settings;

	_connectButton->setFixedSize( 40, 40 );
	_connectButton->setIconSize( QSize( 40, 40 ) ); 
	_connectButton->setIcon( QIcon( ":/icons/wifi_disconnect.png" ) );
	_lineEdit->setText( settings.value( "ipaddress", "172.26.1.1" ).toString() );

	addWidget( _lineEdit );
	addWidget( _connectButton );

	_connectTimer->setSingleShot( false );
	_connectTimer->setInterval( 400 );
	
	bool connected = true;
	connected &= (bool)connect( _connectButton, SIGNAL( clicked() ), SLOT( on_connectButton_clicked() ) );
	Q_ASSERT( connected );

	connected &= (bool)connect( _lineEdit, SIGNAL( returnPressed() ), SLOT( on_connectButton_clicked() ) );
	Q_ASSERT( connected );

	connected &= (bool)connect( _connectTimer, SIGNAL( timeout() ), SLOT( on_connectTimer_timeout() ) );
	Q_ASSERT( connected );

	connected &= (bool)connect( MyCom::singleton(), SIGNAL( connected() ), SLOT( on_com_connected() ) );
	Q_ASSERT( connected );

	connected &= (bool)connect(MyCom::singleton(), SIGNAL(disconnected()), SLOT(on_com_disconnected()));
	Q_ASSERT( connected );
}

void AddressInput::on_connectButton_clicked()
{
	if (MyCom::singleton()->isConnected())
	{
		MyCom::singleton()->disconnectFromServer();
		_lineEdit->setEnabled( true );
	}
	else
	{
		_lineEdit->setEnabled( false );

		_connectCounter = 0;
		MyCom::singleton()->setAddress(_lineEdit->text().toLatin1().constData());
		MyCom::singleton()->connectToServer(false);

		on_connectTimer_timeout();
		_connectTimer->start();
	}
}

void AddressInput::on_connectTimer_timeout()
{
	_connectButton->setIcon( QIcon( QString( ":/icons/wifi_connect%1.png" ).arg( _connectCounter+1 ) ) );

	++_connectCounter;
	if( _connectCounter > 2 )
	{
		_connectCounter = 0;
	}
}

void AddressInput::on_com_connected()
{
	_connectTimer->stop();
	_connectButton->setIcon( QIcon( ":/icons/wifi_connect3.png" ) );

	QSettings settings;
	settings.setValue("ipaddress", QString(MyCom::singleton()->address()));
}

void AddressInput::on_com_disconnected()
{
	_lineEdit->setEnabled( true );
	_connectTimer->stop();
	_connectButton->setIcon( QIcon( ":/icons/wifi_disconnect.png" ) );
}
