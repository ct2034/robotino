//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#include "ControlBar.h"

ControlBar::ControlBar( QWidget* parent )
	: QToolBar( parent )
	, _startButton( new QPushButton( "Start", this ) )
	, _stopButton( new QPushButton( "Stop", this ) )
	, _beltSelector( new QSpinBox( this ) )
{
	setMinimumSize( 200, 46 );
	setFloatable( false );
	setMovable( false );

	addWidget( _startButton );
	addWidget( _stopButton );
	addWidget( _beltSelector );

	_stopButton->setEnabled( false );

	bool ok;
	ok = connect( _startButton, SIGNAL( clicked() ), SLOT( on_startButton_clicked() ) );
	Q_ASSERT( ok );

	ok = connect( _stopButton, SIGNAL( clicked() ), SLOT( on_stopButton_clicked() ) );
	Q_ASSERT( ok );
}

void ControlBar::on_startButton_clicked()
{
	_startButton->setEnabled( false );
	_stopButton->setEnabled( true );

	Q_EMIT start( _beltSelector->value() );
}

void ControlBar::on_stopButton_clicked()
{
	_startButton->setEnabled( true );
	_stopButton->setEnabled( false );

	Q_EMIT stop();
}
