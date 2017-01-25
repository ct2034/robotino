//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "OdometryWidget.h"
#include "MyCom.h"
#include <rec/robotino/api2/utils.h>

OdometryWidget::OdometryWidget( QWidget* parent )
: QWidget( parent )
, _setButton( new QPushButton( this ) )
, _setBlocking( new QCheckBox( this ) )
, _setx( new QLineEdit( this ) )
, _sety( new QLineEdit( this ) )
, _setphi( new QLineEdit( this ) )
, _odomtery( new QLabel( this ) )
, _speed( new QLabel( this ) )
, _sequence( new QLabel( this ) )
{
	QFormLayout* layout = new QFormLayout;
	setLayout( layout );

	layout->addRow( _setButton );
	layout->addRow( _setBlocking );
	layout->addRow( tr("set x[m]:"), _setx );
	layout->addRow( tr("set y[m]:"), _sety );
	layout->addRow( tr("set phi[deg]:"), _setphi );
	layout->addRow( tr("x[m] y[m] phi[deg]:"), _odomtery );
	layout->addRow( tr("vx[m/s] vy[m/s] omega[deg/s]:"), _speed );
	layout->addRow( tr("Sequence:"), _sequence );

	_setButton->setText( tr("Set") );
	_setBlocking->setText( tr("Blocking") );

	bool connected = true;
	connected &= (bool)connect( _setButton, SIGNAL( clicked() ), SLOT( on_setButton_clicked() ) );
	Q_ASSERT( connected );
}

void OdometryWidget::on_setButton_clicked()
{
	double x = _setx->text().toDouble();
	double y = _sety->text().toDouble();
	double phi = rec::robotino::api2::deg2rad( _setphi->text().toDouble() );
	if( false == set( x, y, phi, _setBlocking->isChecked() ) )
	{
		qDebug() << "set failed";
	}
}

void OdometryWidget::readingsEvent( double x, double y, double phi, float vx, float vy, float omega, unsigned int sequence )
{
	_odomtery->setText( QString("%1 %2 %3").arg( x, 4, 'f', 2 ).arg( y, 4, 'f', 2 ).arg( rec::robotino::api2::rad2deg( phi ), 4, 'f', 2 ) );
	_speed->setText( QString("%1 %2 %3").arg( vx, 4, 'f', 2 ).arg( vy, 4, 'f', 2 ).arg( rec::robotino::api2::rad2deg( omega ), 4, 'f', 2 ) );
	_sequence->setText( QString("%1").arg( sequence ) );
}
