#include "BumperWidget.h"

BumperWidget::BumperWidget()
	: _number( new QLCDNumber( 1, this ) )
{
	QHBoxLayout* layout = new QHBoxLayout;
	setLayout( layout );

	layout->addWidget( _number );
}

void BumperWidget::bumperEvent( bool hasContact )
{
	_number->display( hasContact ? 1 : 0 );
}

