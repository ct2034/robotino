//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MotorWidget.h"
#include "MyCom.h"
#include <rec/robotino/api2/utils.h>

MotorWidget::MotorWidget( QWidget* parent, int number )
	: QWidget( parent )
	, _resetPosButton( new QPushButton( this ) )
	, _velLabel( new QLabel( this ) )
	, _posLabel( new QLabel( this ) )
	, _currentLabel( new QLabel( this ) )
{
	setMotorNumber( number );

	QFormLayout* layout = new QFormLayout;
	setLayout( layout );

	layout->addRow( tr("Velocity"), _velLabel );
	layout->addRow( tr("Position"), _posLabel );
	layout->addRow( tr("Current"), _currentLabel );

	_resetPosButton->setText( tr("Reset position") );
	layout->addRow( _resetPosButton );

	bool ok;
	ok = connect( _resetPosButton, SIGNAL( clicked() ), SLOT( on_resetPosButton_clicked() ) );
	Q_ASSERT( ok );
}

void MotorWidget::motorReadingsChanged( float velocity, int position, float current )
{
	_velLabel->setText( QString("%1").arg( velocity ) );
	_posLabel->setText( QString("%1").arg( position ) );
	_currentLabel->setText( QString("%1").arg( current ) );
}

void MotorWidget::on_resetPosButton_clicked()
{
	this->resetPosition( 0 );
}

