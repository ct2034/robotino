#include "DigitalOutputWidget.h"

DigitalOutputWidget::DigitalOutputWidget()
	: _buttons( 8 )
{
	QHBoxLayout* layout = new QHBoxLayout;
	setLayout( layout );

	for( int i=0; i<_buttons.size(); ++i )
	{
		_buttons[i] = new QCheckBox( this );
		layout->addWidget( _buttons[i] );

		bool ok = connect( _buttons[i], SIGNAL( clicked() ), SLOT( on_button_clicked() ) );
		Q_ASSERT( ok );
	}
}

void DigitalOutputWidget::on_button_clicked()
{
	QVector< int > values( _buttons.size() );
	for( int i=0; i<_buttons.size(); ++i )
	{
		values[i] = _buttons[i]->isChecked() ? 1 : 0;
	}

	this->setValues( values.constData(), values.size() );
}
