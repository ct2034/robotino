#include "DigitalInputWidget.h"

DigitalInputWidget::DigitalInputWidget()
	: _numbers( 8 )
{

	QHBoxLayout* layout = new QHBoxLayout;
	setLayout( layout );

	for( int i=0; i<_numbers.size(); ++i )
	{
		_numbers[i] = new QLCDNumber( 1, this );
		layout->addWidget( _numbers[i] );
	}
}

void DigitalInputWidget::valuesChangedEvent( const int* values, unsigned int size )
{
	for( unsigned int i=0; i < static_cast< unsigned int >( _numbers.size() ) && i<size; ++i )
	{
		_numbers[i]->display( values[i] );
	}
}
