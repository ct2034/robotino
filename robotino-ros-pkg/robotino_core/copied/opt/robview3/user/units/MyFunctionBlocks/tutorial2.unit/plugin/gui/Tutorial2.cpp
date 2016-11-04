/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "Tutorial2.h"
#include "plugin/simulation/UnitCallback.h"

/******************************************************************************/
/***   Implementation of CounterDialog                                      ***/
/******************************************************************************/
Tutorial2::Tutorial2 (plugin::gui::UnitDelegate& del )
: plugin::gui::UnitDialog(del)
, _n( new QSpinBox( this ) )
{
	_n->setRange( 1, 1000 );

	QVBoxLayout* layout = new QVBoxLayout;
	setLayout( layout );

	layout->addWidget( _n );
	layout->addStretch();

	_n->setValue( readInput( "n" ).toInt() );

	del.registerInput ("n", 1);

	connect( _n, SIGNAL( valueChanged( int ) ), SLOT( on_n_valueChanged( int ) ) );
}

void Tutorial2::update (const plugin::gui::UnitHistoryBundle& buf)
{
	plugin::gui::VariableHistoryBuffer* n = buf.getInputHistory ("n");
	if( n )
	{
		int v = n->last().toInt();
		_n->blockSignals( true );
		_n->setValue( v );
		_n->blockSignals( false );
	}
}

void Tutorial2::translate()
{
}

void Tutorial2::on_n_valueChanged( int value )
{
	writeInput( "n", value );
}
